#include "al_VideoReader.hpp"

using namespace al;

VideoReader::VideoReader() { init(); }

void VideoReader::init() {
  pFormatCtx = nullptr;

  audio_st_idx = -1;
  audio_st = nullptr;
  audio_ctx = nullptr;
  audio_frame = nullptr;
  audio_pkt = nullptr;
  audio_sampleRate = 0;
  audio_channels = 0;

  video_st_idx = -1;
  video_st = nullptr;
  video_ctx = nullptr;
  sws_ctx = nullptr;
  video_width = 0;
  video_height = 0;
  video_fps = 0;

  decode_thread = nullptr;
  video_thread = nullptr;

  quit = 0;

  // to be removed
  pFrame = nullptr;
  pFrameRGB = nullptr;
  buffer = nullptr;
  numBytes = 0;
  pPacket = nullptr;
}

bool VideoReader::load(const char *url) {
  // open file
  if (avformat_open_input(&pFormatCtx, url, NULL, NULL) < 0) {
    std::cerr << "Could not open file: " << url << std::endl;
    return false;
  }

  // retrieve stream information
  if (avformat_find_stream_info(pFormatCtx, NULL) < 0) {
    std::cerr << "Could not find stream info: " << url << std::endl;
    return false;
  }

  // print info
  av_dump_format(pFormatCtx, 0, url, 0);

  // find video stream
  for (int i = 0; i < pFormatCtx->nb_streams; ++i) {
    if (pFormatCtx->streams[i]->codecpar->codec_type == AVMEDIA_TYPE_VIDEO &&
        video_st_idx < 0) {
      video_st_idx = i;
    }
    if (pFormatCtx->streams[i]->codecpar->codec_type == AVMEDIA_TYPE_AUDIO &&
        audio_st_idx < 0) {
      audio_st_idx = i;
    }
  }

  if (video_st_idx == -1) {
    std::cerr << "Could not find video stream" << std::endl;
    return false;
  } else if (!stream_component_open(video_st_idx)) {
    std::cerr << "Could not open video codec" << std::endl;
    return false;
  }
  currentFrame = 0;

  if (audio_st_idx == -1) {
    std::cerr << "Could not find audio stream" << std::endl;
  } else if (!stream_component_open(audio_st_idx)) {
    std::cerr << "Could not open audio codec" << std::endl;
    return false;
  }

  for (int i = 0; i < audio_channels; i++) {
    audio_buffer.emplace_back(SingleRWRingBuffer{8192 * 8});
  }

  return true;
}

bool VideoReader::stream_component_open(int stream_index) {
  // check if stream index is valid
  if (stream_index < 0 || stream_index >= pFormatCtx->nb_streams) {
    std::cerr << "Invalid stream index" << std::endl;
    return false;
  }

  // retrieve codec
  const AVCodec *codec = nullptr;
  codec = avcodec_find_decoder(
      pFormatCtx->streams[stream_index]->codecpar->codec_id);
  if (!codec) {
    std::cerr << "Unsupported codec" << std::endl;
    return false;
  }

  // retrieve codec context
  AVCodecContext *codecCtx = nullptr;
  codecCtx = avcodec_alloc_context3(codec);
  if (avcodec_parameters_to_context(
          codecCtx, pFormatCtx->streams[stream_index]->codecpar) != 0) {
    std::cerr << "Could not copy codec context" << std::endl;
    return false;
  }

  // initialize the AVCOdecContext to use the given AVCodec
  if (avcodec_open2(codecCtx, codec, NULL) < 0) {
    std::cerr << "Could not open codec" << std::endl;
    return false;
  }

  switch (codecCtx->codec_type) {
  case AVMEDIA_TYPE_AUDIO: {
    audio_ctx = codecCtx;

    audio_sampleRate = audio_ctx->sample_rate;
    audio_channels = audio_ctx->channels;

    // allocate a new frame, used to decode audio packets
    audio_frame = av_frame_alloc();
    if (!audio_frame) {
      std::cerr << "Could not allocate audio frame" << std::endl;
      return false;
    }

    // allocate audio packet
    audio_pkt = av_packet_alloc();
    if (!audio_pkt) {
      std::cerr << "Could not allocate audio packet" << std::endl;
      return false;
    }

    // initialize audio packet queue
    packet_queue_init(&audioq);
  } break;

  case AVMEDIA_TYPE_VIDEO: {
    video_ctx = codecCtx;

    video_width = video_ctx->width;
    video_height = video_ctx->height;

    // allocate space for video frame
    pFrame = av_frame_alloc();
    if (!pFrame) {
      std::cerr << "Could not allocate video frame" << std::endl;
      return false;
    }

    // allocate space for converted frame
    pFrameRGB = av_frame_alloc();
    if (!pFrameRGB) {
      std::cerr << "Could not allocate converted frame" << std::endl;
      return false;
    }

    // allocate space to convert raw data
    numBytes = av_image_get_buffer_size(AV_PIX_FMT_RGBA, video_ctx->width,
                                        video_ctx->height, 32);
    buffer = (uint8_t *)av_malloc(numBytes * sizeof(uint8_t));

    // Setup the parameters for pFrameRGB
    av_image_fill_arrays(pFrameRGB->data, pFrameRGB->linesize, buffer,
                         AV_PIX_FMT_RGBA, video_ctx->width, video_ctx->height,
                         32);

    // allocate video packet
    pPacket = av_packet_alloc();
    if (!pPacket) {
      std::cerr << "Could not allocate video packet" << std::endl;
      return false;
    }

    // initialize SWS context for software scaling
    sws_ctx =
        sws_getContext(video_ctx->width, video_ctx->height, video_ctx->pix_fmt,
                       video_ctx->width, video_ctx->height, AV_PIX_FMT_RGBA,
                       SWS_FAST_BILINEAR, NULL, NULL, NULL);

    // read first frame
    readFrame();

    video_fps = av_q2d(pFormatCtx->streams[video_st_idx]->r_frame_rate);
  } break;
  default: {
    break;
  }
  }
  return true;
}

bool VideoReader::readFrame() {
  while (true) {
    // free the packet that was allocated by av_read_frame
    av_packet_unref(pPacket);

    // read the next frame
    if (av_read_frame(pFormatCtx, pPacket) < 0) {
      quit = 1;
      // no more frames. end playback
      return false;
    }
    // is this from the video stream?
    if (pPacket->stream_index == video_st_idx) {
      // send next packet for decoding
      if (avcodec_send_packet(video_ctx, pPacket) < 0) {
        std::cerr << "Error sending packet for decoding" << std::endl;
        quit = -1;
        return false;
      }

      // successfully got packet
      currentFrame++;
      return true;
    } else if (pPacket->stream_index == audio_st_idx) {
      packet_queue_put(&audioq, pPacket);
    }
  }
}

uint8_t *VideoReader::getFrame(uint64_t frameNum) {

  if (frameNum == currentFrame) {
    return nullptr;
  }
  if (frameNum != UINT64_MAX && frameNum != currentFrame + 1) {
    // FIXME seek is not working.
    // Fluxh mode:
    // https://libav.org/documentation/doxygen/master/group__lavc__encdec.html
    //    avcodec_send_packet(pCodecCtx, nullptr);
    if (av_seek_frame(pFormatCtx, -1, frameNum,
                      AVSEEK_FLAG_FRAME | AVSEEK_FLAG_BACKWARD) != 0) {
      std::cerr << "Error seeking to frame " << frameNum << std::endl;
    }
    currentFrame = frameNum - 1;

    avcodec_flush_buffers(pCodecCtx);
  }

  // receive frame
  int ret = avcodec_receive_frame(video_ctx, pFrame);

  if (ret == AVERROR(EAGAIN)) {
    if (!readFrame()) {
      // no more frames
      return nullptr;
    }

    // receive new frame
    ret = avcodec_receive_frame(video_ctx, pFrame);
  }

  if (ret == AVERROR(EAGAIN)) {
    std::cerr << "Error receiving frame" << std::endl;
    quit = -1;
    return nullptr;
  } else if (ret == AVERROR_EOF) {
    // end of file
    quit = 1;
    return nullptr;
  } else if (ret < 0) {
    std::cerr << "Error while decoding" << std::endl;
    quit = -1;
    return nullptr;
  }

  // Convert the image from its native format to RGB
  sws_scale(sws_ctx, (uint8_t const *const *)pFrame->data, pFrame->linesize, 0,
            video_ctx->height, pFrameRGB->data, pFrameRGB->linesize);

  return (uint8_t *)*pFrameRGB->extended_data; // same as data[0]
}

void VideoReader::readAudioBuffer() {
  while (audio_buffer[0].writeSpace() > 8192) {
    if (quit != 0) {
      return;
    }

    // get more audio data
    if (audio_decode_frame() < 0) {
      std::cerr << "audio_decode_frame() failed" << std::endl;
      quit = -1;
      return;
    }
  }
}

int VideoReader::audio_decode_frame() {
  int data_size = 0;

  // check global quit flag
  if (quit != 0) {
    return 0;
  }

  // run while data is still left in audio packet
  while (audio_pkt->data) {
    // retrieve audio frame
    int ret = avcodec_receive_frame(audio_ctx, audio_frame);

    // need more data
    if (ret == AVERROR(EAGAIN)) {
      break;
    } else if (ret == AVERROR_EOF) {
      // end of file. let the video play handle quitting
      // TODO: check where global quit flag is being set
      return 0;
    } else if (ret < 0) {
      std::cerr << "Error receiving audio frame" << std::endl;
      return -1;
    }

    int sample_size = av_get_bytes_per_sample(audio_ctx->sample_fmt);
    if (sample_size < 0) {
      std::cerr << "Failed to calculate data size" << std::endl;
      return -1;
    } else if (sample_size * audio_frame->nb_samples > 8192) {
      std::cerr << "Audio overrun" << std::endl;
      // TODO: will this ever happen? adjust writing as needed
    }

    for (int i = 0; i < audio_frame->nb_samples; ++i) {
      for (int ch = 0; ch < audio_ctx->channels; ++ch) {
        data_size += audio_buffer[ch].write(
            (const char *)audio_frame->data[ch] + sample_size * i, sample_size);
      }
    }

    // audio buffer sufficiently filled. come back later
    // TODO: see if this can be more optimized
    if (audio_buffer[0].writeSpace() < 8192) {
      return data_size;
    }
  }

  // ideally this shouldn't happen
  if (audio_pkt->data) {
    // wipe the packet
    av_packet_unref(audio_pkt);
  }

  // get new audio packet from queue
  if (packet_queue_get(&audioq, audio_pkt, 1) < 0) {
    // global quit flag has been set
    return 0;
  }

  // send packet to codec
  if (avcodec_send_packet(audio_ctx, audio_pkt) < 0) {
    std::cerr << "Error sending audio packet for decoding" << std::endl;
    return -1;
  }

  return data_size;
}

void VideoReader::packet_queue_init(PacketQueue *pq) { pq->dataSize = 0; }

void VideoReader::packet_queue_put(PacketQueue *pq, AVPacket *packet) {
  std::unique_lock<std::mutex> lk(pq->mutex);

  // need to clone the packet to properly allocate and hand over refs
  pq->queue.push(av_packet_clone(packet));
  pq->dataSize += packet->size;

  // notify packet_queue_get
  pq->cond.notify_one();
}

int VideoReader::packet_queue_get(PacketQueue *pq, AVPacket *packet,
                                  int block) {
  int ret;

  std::unique_lock<std::mutex> lk(pq->mutex);

  while (true) {
    // global quit
    if (quit != 0) {
      ret = -1;
      break;
    }

    if (!pq->queue.empty()) {
      // copy contents from queue
      *packet = *(pq->queue.front());
      pq->dataSize -= packet->size;

      // pop handles deallocation
      pq->queue.pop();

      // successful packet retrieval
      ret = 1;
      break;
    } else if (!block) {
      // avoid waiting if block = 0
      ret = 0;
      break;
    } else {
      // wait until packet is available
      pq->cond.wait(lk);
    }
  }

  return ret;
}

void VideoReader::cleanup() {
  // TODO: check order & missing allocations
  av_packet_unref(pPacket);
  av_packet_free(&pPacket);
  av_packet_unref(audio_pkt);
  av_packet_free(&audio_pkt);

  // Free the RGB image
  av_free(buffer);
  av_frame_free(&pFrameRGB);
  av_free(pFrameRGB);

  // Free the video frame
  av_frame_free(&pFrame);
  av_free(pFrame);

  // Free the audio frame
  av_frame_free(&audio_frame);
  av_free(audio_frame);

  // Close the codecs
  avcodec_close(video_ctx);
  avcodec_close(audio_ctx);

  // Close the video file
  avformat_close_input(&pFormatCtx);
}
