#include "al_VideoReader.hpp"

using namespace al;

VideoReader::VideoReader() {}

void VideoReader::init() {
  pFormatCtx = nullptr;
  video_codec_width = 0;
  video_codec_height = 0;
  r_fps = 0;
  audio_sample_rate = 0;
  audio_channels = 0;
  videoStream = -1;
  audioStream = -1;
  pCodecCtx = nullptr;
  aCodecCtx = nullptr;
  pFrame = nullptr;
  pFrameRGB = nullptr;
  aFrame = nullptr;
  buffer = nullptr;
  numBytes = 0;
  sws_ctx = nullptr;
  pPacket = nullptr;
  aPacket = nullptr;
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
        videoStream < 0) {
      videoStream = i;
    }
    if (pFormatCtx->streams[i]->codecpar->codec_type == AVMEDIA_TYPE_AUDIO &&
        audioStream < 0) {
      audioStream = i;
    }
  }

  if (videoStream == -1) {
    std::cerr << "Could not find video stream" << std::endl;
    return false;
  } else if (!stream_component_open(videoStream)) {
    std::cerr << "Could not open video codec" << std::endl;
    return false;
  }

  if (audioStream == -1) {
    std::cerr << "Could not find audio stream" << std::endl;
  } else if (!stream_component_open(audioStream)) {
    std::cerr << "Could not open audio codec" << std::endl;
    return false;
  }

  for (int i = 0; i < audio_channels; i++) {
    mAudioBuffer.emplace_back(SingleRWRingBuffer{8192 * 8});
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
    aCodecCtx = codecCtx;

    audio_sample_rate = aCodecCtx->sample_rate;
    audio_channels = aCodecCtx->channels;

    // allocate a new frame, used to decode audio packets
    aFrame = av_frame_alloc();
    if (!aFrame) {
      std::cerr << "Could not allocate audio frame" << std::endl;
      return false;
    }

    // allocate audio packet
    aPacket = av_packet_alloc();
    if (!aPacket) {
      std::cerr << "Could not allocate audio packet" << std::endl;
      return false;
    }

    packet_queue_init(&audioq);
  } break;
  case AVMEDIA_TYPE_VIDEO: {
    pCodecCtx = codecCtx;

    video_codec_width = pCodecCtx->width;
    video_codec_height = pCodecCtx->height;

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
    numBytes = av_image_get_buffer_size(AV_PIX_FMT_RGBA, pCodecCtx->width,
                                        pCodecCtx->height, 32);
    buffer = (uint8_t *)av_malloc(numBytes * sizeof(uint8_t));

    // Setup the parameters for pFrameRGB
    av_image_fill_arrays(pFrameRGB->data, pFrameRGB->linesize, buffer,
                         AV_PIX_FMT_RGBA, pCodecCtx->width, pCodecCtx->height,
                         32);

    // allocate video packet
    pPacket = av_packet_alloc();
    if (!pPacket) {
      std::cerr << "Could not allocate video packet" << std::endl;
      return false;
    }

    // initialize SWS context for software scaling
    sws_ctx =
        sws_getContext(pCodecCtx->width, pCodecCtx->height, pCodecCtx->pix_fmt,
                       pCodecCtx->width, pCodecCtx->height, AV_PIX_FMT_RGBA,
                       SWS_FAST_BILINEAR, NULL, NULL, NULL);

    // read first frame
    readFrame();

    r_fps = av_q2d(pFormatCtx->streams[videoStream]->r_frame_rate);
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
    if (pPacket->stream_index == videoStream) {
      // send next packet for decoding
      if (avcodec_send_packet(pCodecCtx, pPacket) < 0) {
        std::cerr << "Error sending packet for decoding" << std::endl;
        quit = -1;
        return false;
      }

      // successfully got packet
      return true;
    } else if (pPacket->stream_index == audioStream) {
      packet_queue_put(&audioq, pPacket);
    }
  }
}

uint8_t *VideoReader::getFrame() {
  // receive frame
  int ret = avcodec_receive_frame(pCodecCtx, pFrame);

  if (ret == AVERROR(EAGAIN)) {
    if (!readFrame()) {
      // no more frames
      return nullptr;
    }

    // receive new frame
    ret = avcodec_receive_frame(pCodecCtx, pFrame);
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
            pCodecCtx->height, pFrameRGB->data, pFrameRGB->linesize);

  return buffer;
}

void VideoReader::readAudioBuffer() {
  while (mAudioBuffer[0].writeSpace() > 8192) {
    if (quit != 0) {
      return;
    }

    // get more audio data
    if (audio_decode_frame(aCodecCtx) < 0) {
      std::cerr << "audio_decode_frame() failed" << std::endl;
      quit = -1;
      return;
    }
  }
}

int VideoReader::audio_decode_frame(AVCodecContext *aCodecCtx) {
  int data_size = 0;

  // check global quit flag
  if (quit != 0) {
    return 0;
  }

  while (aPacket->data) {
    int ret = avcodec_receive_frame(aCodecCtx, aFrame);
    if (ret == AVERROR(EAGAIN)) {
      break;
    } else if (ret == AVERROR_EOF) {
      return 0;
    } else if (ret < 0) {
      std::cerr << "Error receiving audio frame" << std::endl;
      return -1;
    }

    int sample_size = av_get_bytes_per_sample(aCodecCtx->sample_fmt);
    if (sample_size < 0) {
      std::cerr << "Failed to calculate data size" << std::endl;
      return -1;
    } else if (sample_size * aFrame->nb_samples > 8192) {
      std::cerr << "Audio overrun" << std::endl;
    }

    for (int i = 0; i < aFrame->nb_samples; ++i) {
      for (int ch = 0; ch < aCodecCtx->channels; ++ch) {
        data_size += mAudioBuffer[ch].write(
            (const char *)aFrame->data[ch] + sample_size * i, sample_size);
      }
    }

    if (mAudioBuffer[0].writeSpace() < 8192) {
      return data_size;
    }
  }

  if (aPacket->data) {
    // wipe the packet
    av_packet_unref(aPacket);
  }

  // get new audio packet from queue
  if (packet_queue_get(&audioq, aPacket, 1) < 0) {
    return 0; // global quit
  }

  if (avcodec_send_packet(aCodecCtx, aPacket) < 0) {
    std::cerr << "Error sending audio packet for decoding" << std::endl;
    return -1;
  }

  return data_size;
}

void VideoReader::packet_queue_init(PacketQueue *pq) { pq->dataSize = 0; }

void VideoReader::packet_queue_put(PacketQueue *pq, AVPacket *packet) {
  std::unique_lock<std::mutex> lk(pq->mutex);

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
    if (quit != 0) {
      ret = -1;
      break;
    }

    if (!pq->queue.empty()) {
      *packet = *(pq->queue.front());
      pq->dataSize -= packet->size;

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
  av_packet_unref(pPacket);
  av_packet_free(&pPacket);
  av_packet_unref(aPacket);
  av_packet_free(&aPacket);

  // Free the RGB image
  av_free(buffer);
  av_frame_free(&pFrameRGB);
  av_free(pFrameRGB);

  // Free the video frame
  av_frame_free(&pFrame);
  av_free(pFrame);

  // Free the audio frame
  av_frame_free(&aFrame);
  av_free(aFrame);

  // Close the codecs
  avcodec_close(pCodecCtx);
  avcodec_close(aCodecCtx);

  // Close the video file
  avformat_close_input(&pFormatCtx);
}
