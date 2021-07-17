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

  video_st_idx = -1;
  video_st = nullptr;
  video_ctx = nullptr;
  sws_ctx = nullptr;
  pictq.write_index = 0;
  pictq.read_index = 0;
  pictq.size = 0;

  decode_thread = nullptr;
  video_thread = nullptr;
  audio_thread = nullptr;

  global_quit = 0;

  currentFrame = 0;
}

int VideoReader::audioSampleRate() {
  if (audio_st)
    return audio_ctx->sample_rate;
  else {
    std::cerr << "No audio stream" << std::endl;
    return 0;
  }
}

int VideoReader::audioNumChannels() {
  if (audio_st)
    return audio_ctx->channels;
  else {
    std::cerr << "No audio stream" << std::endl;
    return 0;
  }
}

int VideoReader::width() { return video_ctx->width; }
int VideoReader::height() { return video_ctx->height; }
double VideoReader::fps() {
  double guess = av_q2d(av_guess_frame_rate(pFormatCtx, video_st, NULL));
  if (guess == 0) {
    std::cerr << "Could not guess frame rate" << std::endl;
    guess = av_q2d(pFormatCtx->streams[video_st_idx]->r_frame_rate);
  }
  return guess;
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

  if (audio_st_idx == -1) {
    // std::cerr << "Could not find audio stream" << std::endl;
  } else if (!stream_component_open(audio_st_idx)) {
    std::cerr << "Could not open audio codec" << std::endl;
    return false;
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

  // initialize the AVCodecContext to use the given AVCodec
  if (avcodec_open2(codecCtx, codec, NULL) < 0) {
    std::cerr << "Could not open codec" << std::endl;
    return false;
  }

  switch (codecCtx->codec_type) {
  case AVMEDIA_TYPE_AUDIO: {
    audio_st = pFormatCtx->streams[stream_index];
    audio_ctx = codecCtx;

    // allocate audio packet
    audio_pkt = av_packet_alloc();
    if (!audio_pkt) {
      std::cerr << "Could not allocate audio packet" << std::endl;
      return false;
    }

    // allocate a new frame, used to decode audio packets
    audio_frame = av_frame_alloc();
    if (!audio_frame) {
      std::cerr << "Could not allocate audio frame" << std::endl;
      return false;
    }

    // allocate audio buffer
    for (int i = 0; i < audio_ctx->channels; i++) {
      audio_buffer.emplace_back(SingleRWRingBuffer{AUDIO_BUFFER_SIZE});
    }

    // initialize audio packet queue
    packet_queue_init(&audioq);

    // TODO: add initialization notice to videoapp
  } break;

  case AVMEDIA_TYPE_VIDEO: {
    video_st = pFormatCtx->streams[stream_index];
    video_ctx = codecCtx;

    // initialize video packet queue
    packet_queue_init(&videoq);

    // initialize SWS context for software scaling
    sws_ctx =
        sws_getContext(video_ctx->width, video_ctx->height, video_ctx->pix_fmt,
                       video_ctx->width, video_ctx->height, AV_PIX_FMT_RGBA,
                       SWS_FAST_BILINEAR, NULL, NULL, NULL);
  } break;
  default: {
    break;
  }
  }

  return true;
}

void VideoReader::start() {
  // if threads are already running, close them
  if (decode_thread != nullptr || video_thread != nullptr ||
      audio_thread != nullptr) {
    stop();
  }

  // check if video streams is valid
  if (video_st != nullptr) {
    decode_thread = new std::thread(decodeThreadFunction, this);
    video_thread = new std::thread(videoThreadFunction, this);
    if (!mAudioEnabled) {
      audio_thread = new std::thread(audioThreadFunction, this);
    }
  }

  if (!decode_thread) {
    std::cerr << "Could not start decoding thread" << std::endl;
    stop();
  }

  if (!video_thread) {
    std::cerr << "Could not start video thread" << std::endl;
    stop();
  }

  // might need to populate audio buffer here
}

void VideoReader::decodeThreadFunction(VideoReader *reader) {
  // allocate packet
  AVPacket *packet = av_packet_alloc();

  while (true) {
    // check global quit flag
    if (reader->global_quit != 0) {
      break;
    }

    // if queues are full, wait 10ms and retry
    if (reader->audioq.dataSize > MAX_AUDIOQ_SIZE ||
        reader->videoq.dataSize > MAX_VIDEOQ_SIZE) {
      al_sleep_nsec(10000000); // 10 ms
      continue;
    }

    // read the next frame
    if (av_read_frame(reader->pFormatCtx, packet) < 0) {
      // no read error; wait for file
      if (reader->pFormatCtx->pb->error == 0) {
        al_sleep_nsec(1000000000); // 100 ms
        continue;
      } else {
        std::cerr << "Error reading frame" << std::endl;
        break;
      }
    }

    // put packet in correct queue
    if (packet->stream_index == reader->video_st_idx) {
      reader->packet_queue_put(&reader->videoq, packet);
    } else if (packet->stream_index == reader->audio_st_idx) {
      reader->packet_queue_put(&reader->audioq, packet);
    } else {
      av_packet_unref(packet);
    }
  }

  // free the memory
  av_packet_unref(packet);
  av_packet_free(&packet);

  // wait for rest of program to end
  while (reader->global_quit == 0) {
    al_sleep_nsec(1000000000); // 100 ms
  }

  return;
}

void VideoReader::videoThreadFunction(VideoReader *reader) {
  // allocate video packet
  AVPacket *video_pkt = av_packet_alloc();
  if (!video_pkt) {
    std::cerr << "Could not allocate video packet" << std::endl;
    reader->global_quit = -1;
    return;
  }

  // allocate space for video frame
  AVFrame *video_frame = av_frame_alloc();
  if (!video_frame) {
    std::cerr << "Could not allocate video frame" << std::endl;
    reader->global_quit = -1;
    return;
  }

  bool should_quit = false;

  // resize picture queue
  reader->pictq.queue.resize(PICTQ_SIZE, nullptr);

  // allocate frames in picture queue
  for (int index = 0; index < PICTQ_SIZE; ++index) {
    // check if frame has been allocated
    if (reader->pictq.queue[index]) {
      av_frame_free(&(reader->pictq.queue[index]));
      av_free(reader->pictq.queue[index]);
    }

    // allocate image data buffer
    int numBytes =
        av_image_get_buffer_size(AV_PIX_FMT_RGBA, reader->video_ctx->width,
                                 reader->video_ctx->height, 32);
    uint8_t *buffer = (uint8_t *)av_malloc(numBytes * sizeof(uint8_t));

    // allocate the frame later used to contain the scaled frame
    reader->pictq.queue[index] = av_frame_alloc();
    if (!reader->pictq.queue[index]) {
      std::cerr << "Could not allocate frame" << std::endl;
      should_quit = true;
      break;
    }

    // Setup pointers and linesize for dst frame and image data buffer
    av_image_fill_arrays(reader->pictq.queue[index]->data,
                         reader->pictq.queue[index]->linesize, buffer,
                         AV_PIX_FMT_RGBA, reader->video_ctx->width,
                         reader->video_ctx->height, 32);
  }

  while (!should_quit) {
    if (reader->global_quit != 0) {
      break;
    }
    //<<<<<<< HEAD
    //    // is this from the video stream?
    //    if (pPacket->stream_index == video_st_idx) {
    //      // send next packet for decoding
    //      if (avcodec_send_packet(video_ctx, pPacket) < 0) {
    //        std::cerr << "Error sending packet for decoding" << std::endl;
    //        quit = -1;
    //        return false;
    //=======

    // get video packet from the queue
    if (reader->packet_queue_get(&reader->videoq, video_pkt, 1) < 0) {
      // global quit flag has been set
      break;
    }

    // send raw compressed video data in AVPacket to decoder
    if (avcodec_send_packet(reader->video_ctx, video_pkt) < 0) {
      std::cerr << "Error sending video packet for decoding" << std::endl;
      break;
    }

    while (true) {
      // get decoded output data from decoder
      int ret = avcodec_receive_frame(reader->video_ctx, video_frame);

      // check if entire frame was decoded
      if (ret == AVERROR(EAGAIN)) {
        // need more data
        break;
      } else if (ret == AVERROR_EOF) {
        should_quit = true;
        break;
      } else if (ret < 0) {
        std::cerr << "Error while decoding" << std::endl;
        reader->global_quit = -1;
        should_quit = true;
        break;
        //>>>>>>> d8f50aa8cf9a95e8b25586cbbf913dbde96529f9
      }

      if (!reader->queue_picture(video_frame)) {
        // global quit flag has been set
        should_quit = true;
        break;
      }
    }

    // wipe the packet
    av_packet_unref(video_pkt);
  }

  for (int index = 0; index < PICTQ_SIZE; ++index) {
    if (reader->pictq.queue[index]) {
      av_frame_free(&(reader->pictq.queue[index]));
      av_free(reader->pictq.queue[index]);
    }
  }

  av_frame_free(&video_frame);
  av_free(video_frame);

  av_packet_unref(video_pkt);
  av_packet_free(&video_pkt);

  return;
}

void VideoReader::audioThreadFunction(VideoReader *reader) {

  while (reader->global_quit == 0) {
    // get more audio data
    reader->audio_buffer[0].clear();
    if (reader->audio_decode_frame() < 0) {
      //        std::cerr << "audio_decode_frame() failed" << std::endl;
      //        return;
    }
  }
}

bool VideoReader::queue_picture(AVFrame *qFrame) {
  // acquire picture queue mutex
  std::unique_lock<std::mutex> lk(pictq.mutex);

  // wait until picture queue has space
  while (pictq.size >= PICTQ_SIZE && !global_quit) {
    pictq.cond.wait(lk);
  }

  lk.unlock();

  if (global_quit != 0) {
    return false;
  }

  // check if frame has been correctly allocated
  if (pictq.queue[pictq.write_index]) {
    // set frame info using last decoded frame
    pictq.queue[pictq.write_index]->pict_type = qFrame->pict_type;
    pictq.queue[pictq.write_index]->pts = qFrame->pts;
    pictq.queue[pictq.write_index]->pkt_dts = qFrame->pkt_dts;
    pictq.queue[pictq.write_index]->key_frame = qFrame->key_frame;
    pictq.queue[pictq.write_index]->coded_picture_number =
        qFrame->coded_picture_number;
    pictq.queue[pictq.write_index]->display_picture_number =
        qFrame->display_picture_number;
    pictq.queue[pictq.write_index]->width = qFrame->width;
    pictq.queue[pictq.write_index]->height = qFrame->height;

    // scale image in qFrame->data and put results in pictq frame's data
    sws_scale(sws_ctx, (uint8_t const *const *)qFrame->data, qFrame->linesize,
              0, video_ctx->height, pictq.queue[pictq.write_index]->data,
              pictq.queue[pictq.write_index]->linesize);

    // update write index
    ++pictq.write_index;
    if (pictq.write_index == PICTQ_SIZE) {
      pictq.write_index = 0;
    }

    lk.lock();

    // increase picture queue size
    ++pictq.size;
  } else {
    std::cerr << "Picture queue hasn't been allocated" << std::endl;
    global_quit = -1;
    return false;
    //=======
    //    avcodec_flush_buffers(audio_ctx);
    //>>>>>>> Stashed changes
  }

  return true;
}

//<<<<<<< HEAD
// uint8_t *VideoReader::getFrame(uint64_t frameNum) {

//  if (frameNum == currentFrame) {
//    return nullptr;
//  }
//  if (frameNum != UINT64_MAX && frameNum != currentFrame + 1) {
//    // FIXME seek is not working.
//    // Fluxh mode:
//    // https://libav.org/documentation/doxygen/master/group__lavc__encdec.html
//    //    avcodec_send_packet(pCodecCtx, nullptr);
//    if (av_seek_frame(pFormatCtx, -1, frameNum,
//                      AVSEEK_FLAG_FRAME | AVSEEK_FLAG_BACKWARD) != 0) {
//      std::cerr << "Error seeking to frame " << frameNum << std::endl;
//    }
//    currentFrame = frameNum - 1;

//    avcodec_flush_buffers(pCodecCtx);
//  }

//  // receive frame
//  int ret = avcodec_receive_frame(video_ctx, pFrame);
//=======
uint8_t *VideoReader::getFrame(uint64_t frameNum) {
  // check if video stream was correctly opened
  if (video_st) {
    // check picture queue contains decoded frames
    if (pictq.size == 0) {
      // skip updating texture this frame
      return nullptr;
    } else {
      return (uint8_t *)*pictq.queue[pictq.read_index]
          ->extended_data; // same as data[0]
    }
  }

  // no video stream
  return nullptr;
}

void VideoReader::gotFrame() {
  // update read index
  if (++pictq.read_index == PICTQ_SIZE) {
    pictq.read_index = 0;
  }

  ++currentFrame;

  // acquire picture queue mutex
  std::unique_lock<std::mutex> lk(pictq.mutex);

  // decrease picture queue size
  pictq.size--;

  pictq.cond.notify_one();
}

void VideoReader::readAudioBuffer() {
  if (audio_st) {
    while (audio_buffer[0].writeSpace() > AUDIO_BUFFER_REFRESH_THRESHOLD) {
      if (global_quit != 0) {
        return;
      }

      // get more audio data
      if (audio_decode_frame() < 0) {
        std::cerr << "audio_decode_frame() failed" << std::endl;
        global_quit = -1;
        return;
      }
    }
  }
  return;
}

int VideoReader::audio_decode_frame() {
  int data_size = 0;

  // check global quit flag
  if (global_quit != 0) {
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
    } else if (sample_size * audio_frame->nb_samples >
               AUDIO_BUFFER_REFRESH_THRESHOLD) {
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
    if (audio_buffer[0].writeSpace() < AUDIO_BUFFER_REFRESH_THRESHOLD) {
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

  // send packet to audio codec
  if (avcodec_send_packet(audio_ctx, audio_pkt) < 0) {
    std::cerr << "Error sending audio packet for decoding" << std::endl;
    return -1;
  }

  return data_size;
}

void VideoReader::packet_queue_init(PacketQueue *pktq) { pktq->dataSize = 0; }

void VideoReader::packet_queue_put(PacketQueue *pktq, AVPacket *packet) {
  // acquire packet queue mutex
  std::unique_lock<std::mutex> lk(pktq->mutex);

  // need to clone the packet to properly allocate and hand over refs
  pktq->queue.push(av_packet_clone(packet));
  pktq->dataSize += packet->size;

  // notify packet_queue_get
  pktq->cond.notify_one();
}

int VideoReader::packet_queue_get(PacketQueue *pktq, AVPacket *packet,
                                  int blocking) {
  // acquire packet queue mutex
  std::unique_lock<std::mutex> lk(pktq->mutex);

  while (true) {
    // global quit
    if (global_quit != 0) {
      return -1;
    }

    if (!pktq->queue.empty()) {
      // copy contents from queue
      *packet = *(pktq->queue.front());
      pktq->dataSize -= packet->size;

      // pop handles deallocation
      pktq->queue.pop();

      // successful packet retrieval
      return 1;
    } else if (!blocking) {
      // avoid waiting if block = 0
      return 0;
    } else {
      // temporarily release mutex & wait until packet is available
      pktq->cond.wait(lk);
    }
  }
}

void VideoReader::cleanup() {
  if (audio_st) {
    // Free the audio frame
    av_frame_free(&audio_frame);
    av_free(audio_frame);

    // Free the audio packet
    av_packet_unref(audio_pkt);
    av_packet_free(&audio_pkt);

    // Close the audio codec
    avcodec_close(audio_ctx);
  }

  // Close the video codec
  avcodec_close(video_ctx);

  // Close the video file
  avformat_close_input(&pFormatCtx);
}

void VideoReader::stop() {
  global_quit = 1;

  if (decode_thread) {
    audioq.cond.notify_one();
    videoq.cond.notify_one();
    decode_thread->join();
  }

  if (video_thread) {
    pictq.cond.notify_one();
    video_thread->join();
  }
  if (audio_thread) {
    // TODO audio thread only flushes audio for now. It should be used to queue
    // audio
    //      pictq.cond.notify_one();
    audio_thread->join();
  }

  std::thread *dth = decode_thread;
  decode_thread = nullptr;
  delete dth;

  std::thread *vth = video_thread;
  video_thread = nullptr;
  delete vth;

  cleanup();

  audio_buffer.clear();
  pictq.queue.clear();
}
