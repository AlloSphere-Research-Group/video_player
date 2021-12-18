#include "al_VideoDecoder.hpp"

using namespace al;

void VideoDecoder::init() {
  video_state.format_ctx = nullptr;

  video_state.video_st_idx = -1;
  video_state.video_st = nullptr;
  video_state.video_ctx = nullptr;
  video_state.sws_ctx = nullptr;
  video_state.video_buffer = nullptr;

  video_state.audio_enabled = true;
  video_state.audio_st_idx = -1;
  video_state.audio_st = nullptr;
  video_state.audio_ctx = nullptr;
  video_state.audio_buffer = nullptr;
  video_state.audio_sample_size = 0;
  video_state.audio_data_rate = 0;

  video_state.master_sync = MasterSync::AV_SYNC_EXTERNAL;
  video_state.video_clock = 0;
  video_state.audio_clock = 0;
  video_state.master_clock = 0;
  video_state.last_frame_pts = 0;
  video_state.last_frame_delay = 40e-3;

  video_state.seek_requested = 0;
  video_state.seek_flags = 0;
  video_state.seek_pos = 0;

  video_state.global_quit = 0;
}

bool VideoDecoder::load(const char *url) {
  // open file
  if (avformat_open_input(&video_state.format_ctx, url, NULL, NULL) < 0) {
    std::cerr << "Could not open file: " << url << std::endl;
    return false;
  }

  // retrieve stream information
  if (avformat_find_stream_info(video_state.format_ctx, NULL) < 0) {
    std::cerr << "Could not find stream info: " << url << std::endl;
    return false;
  }

  // print info
  av_dump_format(video_state.format_ctx, 0, url, 0);

  // find video & audio stream
  // TODO: choose stream in case of multiple streams
  for (int i = 0; i < video_state.format_ctx->nb_streams; ++i) {
    if (video_state.format_ctx->streams[i]->codecpar->codec_type ==
            AVMEDIA_TYPE_VIDEO &&
        video_state.video_st_idx < 0) {
      video_state.video_st_idx = i;
      if (video_state.audio_st_idx > 0)
        break;
    }

    if (video_state.format_ctx->streams[i]->codecpar->codec_type ==
            AVMEDIA_TYPE_AUDIO &&
        video_state.audio_st_idx < 0) {
      video_state.audio_st_idx = i;
      if (video_state.video_st_idx > 0)
        break;
    }
  }

  if (video_state.video_st_idx == -1) {
    std::cerr << "Could not find video stream" << std::endl;
    return false;
  } else if (!stream_component_open(&video_state, video_state.video_st_idx)) {
    std::cerr << "Could not open video codec" << std::endl;
    return false;
  }

  if (video_state.audio_st_idx == -1) {
    // no audio stream
    // TODO: consider audio only files
    video_state.audio_enabled = false;
  } else if (video_state.audio_enabled) {
    if (!stream_component_open(&video_state, video_state.audio_st_idx)) {
      std::cerr << "Could not open audio codec" << std::endl;
      return false;
    }
  }

  // TODO: add initialization notice to videoapp
  return true;
}

bool VideoDecoder::stream_component_open(VideoState *vs, int stream_index) {
  // check if stream index is valid
  if (stream_index < 0 || stream_index >= vs->format_ctx->nb_streams) {
    std::cerr << "Invalid stream index" << std::endl;
    return false;
  }

  // retrieve codec
  const AVCodec *codec = nullptr;
  codec = avcodec_find_decoder(
      vs->format_ctx->streams[stream_index]->codecpar->codec_id);
  if (!codec) {
    std::cerr << "Unsupported codec" << std::endl;
    return false;
  }

  // retrieve codec context
  AVCodecContext *codecCtx = nullptr;
  codecCtx = avcodec_alloc_context3(codec);
  if (avcodec_parameters_to_context(
          codecCtx, vs->format_ctx->streams[stream_index]->codecpar) != 0) {
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
    vs->audio_st = vs->format_ctx->streams[stream_index];
    vs->audio_ctx = codecCtx;

    // allocate audio buffer
    // TODO: allocate mediabuffer
    // for (int i = 0; i < audio_ctx->channels; i++) {
    //   audio_buffer.emplace_back(SingleRWRingBuffer{AUDIO_BUFFER_SIZE});
    // }

    // set parameters
    vs->audio_sample_size = av_get_bytes_per_sample(vs->audio_ctx->sample_fmt);
    if (vs->audio_sample_size < 0) {
      std::cerr << "Failed to calculate data size" << std::endl;
      return false;
    }

    vs->audio_data_rate = vs->audio_sample_size * vs->audio_ctx->channels *
                          vs->audio_ctx->sample_rate;

  } break;

  case AVMEDIA_TYPE_VIDEO: {
    vs->video_st = vs->format_ctx->streams[stream_index];
    vs->video_ctx = codecCtx;

    // initialize SWS context for software scaling
    vs->sws_ctx = sws_getContext(vs->video_ctx->width, vs->video_ctx->height,
                                 vs->video_ctx->pix_fmt, vs->video_ctx->width,
                                 vs->video_ctx->height, AV_PIX_FMT_RGBA,
                                 SWS_FAST_BILINEAR, NULL, NULL, NULL);
  } break;
  default: {
    break;
  }
  }

  return true;
}

void VideoDecoder::start() {
  // if threads are already running, close them
  if (decode_thread != nullptr) {
    stop();
  }

  // check if video streams is valid
  if (video_state.video_st != nullptr) {
    decode_thread = new std::thread(decodeThreadFunction, &video_state);
  }

  if (!decode_thread) {
    std::cerr << "Could not start decoding thread" << std::endl;
    stop();
  }
}

void VideoDecoder::decodeThreadFunction(VideoState *vs) {
  // // allocate packet
  AVPacket *packet = av_packet_alloc();
  if (!packet) {
    std::cerr << "Could not allocate packet" << std::endl;
    vs->global_quit = -1;
    return;
  }

  // allocate frame
  AVFrame *frame = av_frame_alloc();
  if (!frame) {
    std::cerr << "Could not allocate frame" << std::endl;
    vs->global_quit = -1;
    return;
  }

  int numBytes = av_image_get_buffer_size(AV_PIX_FMT_RGBA, vs->video_ctx->width,
                                          vs->video_ctx->height, 32);
  uint8_t *buffer = (uint8_t *)av_malloc(numBytes * sizeof(uint8_t));

  // check global quit flag
  while (vs->global_quit == 0) {
    // if (reader->seek_requested) {
    //   int video_stream_index = -1;
    //   int audio_stream_index = -1;
    //   int64_t video_seek_target = reader->seek_pos;
    //   int64_t audio_seek_target = reader->seek_pos;

    //   if (reader->video_st)
    //     video_stream_index = reader->video_st_idx;
    //   if (reader->audio_st && reader->audio_enabled)
    //     audio_stream_index = reader->audio_st_idx;

    //   if (video_stream_index >= 0) {
    //     video_seek_target = av_rescale_q(
    //         video_seek_target, AV_TIME_BASE_Q,
    //         reader->formatCtx->streams[video_stream_index]->time_base);
    //   }
    //   if (audio_stream_index >= 0) {
    //     audio_seek_target = av_rescale_q(
    //         audio_seek_target, AV_TIME_BASE_Q,
    //         reader->formatCtx->streams[audio_stream_index]->time_base);
    //   }

    //   int ret = av_seek_frame(reader->formatCtx, video_stream_index,
    //                           video_seek_target, reader->seek_flags);
    //   if (reader->audio_st && reader->audio_enabled)
    //     ret &= av_seek_frame(reader->formatCtx, audio_stream_index,
    //                          audio_seek_target, reader->seek_flags);

    //   if (ret < 0) {
    //     std::cerr << "Error while seeking" << std::endl;
    //   } else {
    //     if (reader->video_st) {
    //       reader->packet_queue_flush(&reader->videoq);
    //       reader->packet_queue_put(&reader->videoq, reader->flush_pkt);
    //     }
    //     if (reader->audio_st && reader->audio_enabled) {
    //       reader->packet_queue_flush(&reader->audioq);
    //       reader->packet_queue_put(&reader->audioq, reader->flush_pkt);
    //     }
    //   }

    //   reader->seek_requested = 0;
    // }

    // if queues are full, wait 10ms and retry
    // if (reader->audioq.dataSize > MAX_AUDIOQ_SIZE ||
    //     reader->videoq.dataSize > MAX_VIDEOQ_SIZE) {
    //   al_sleep_nsec(10000000); // 10 ms
    //   std::cout << "decode thread" << std::endl;
    //   continue;
    // }

    // read the next frame
    if (av_read_frame(vs->format_ctx, packet) < 0) {
      // no read error; wait for file
      if (vs->format_ctx->pb->error == 0) {
        al_sleep_nsec(100000000); // 10 ms
        continue;
      } else {
        std::cerr << "Error reading frame" << std::endl;
        break;
      }
    }

    // put packet in correct queue
    if (packet->stream_index == vs->video_st_idx) {
      // reader->packet_queue_put(&reader->videoq, packet);
    } else if (packet->stream_index == vs->audio_st_idx) {
      // audio output has been disabled
      if (!vs->audio_enabled) {
        av_packet_unref(packet);
        continue;
      }
      // reader->packet_queue_put(&reader->audioq, packet);
    }

    // wipe the packet
    av_packet_unref(packet);
  }

  // free the memory
  av_freep(buffer);
  av_frame_free(&frame);
  av_packet_free(&packet);

  return;
}

// void VideoDecoder::videoThreadFunction(VideoDecoder *reader) {
//   // TODO: check buffer release
//   // Setup pointers and linesize for dst frame and image data buffer
//   av_image_fill_arrays(reader->pictq.queue[index]->data,
//                        reader->pictq.queue[index]->linesize, buffer,
//                        AV_PIX_FMT_RGBA, reader->video_ctx->width,
//                        reader->video_ctx->height, 32);
// }

// // PTS of the video frame
// double pts = 0;

// while (!should_quit) {
//   if (reader->global_quit != 0) {
//     break;
//   }

//   // get video packet from the queue
//   if (reader->packet_queue_get(&reader->videoq, video_pkt, 1) < 0) {
//     // global quit flag has been set
//     break;
//   }

//   if (video_pkt->data == reader->flush_pkt->data) {
//     avcodec_flush_buffers(reader->video_ctx);
//     // // TODO: this might not be needed
//     std::unique_lock<std::mutex> lk(reader->pictq.mutex);
//     reader->pictq.read_index = 0;
//     reader->pictq.write_index = 0;
//     reader->pictq.size = 0;
//     // reader->pictq.cond.notify_one();
//     continue;
//   }

//   // send raw compressed video data in AVPacket to decoder
//   if (avcodec_send_packet(reader->video_ctx, video_pkt) < 0) {
//     std::cerr << "Error sending video packet for decoding" << std::endl;
//     break;
//   }

//   while (true) {
//     // get decoded output data from decoder
//     int ret = avcodec_receive_frame(reader->video_ctx, video_frame);

//     // check if entire frame was decoded
//     if (ret == AVERROR(EAGAIN)) {
//       // need more data
//       break;
//     } else if (ret == AVERROR_EOF) {
//       should_quit = true;
//       break;
//     } else if (ret < 0) {
//       std::cerr << "Error while decoding" << std::endl;
//       reader->global_quit = -1;
//       should_quit = true;
//       break;
//     }

//     // atempt to guess proper time stamp
//     pts = reader->guess_correct_pts(reader->video_ctx, video_frame->pts,
//                                     video_frame->pkt_dts);

//     // if guess failed
//     if (pts == AV_NOPTS_VALUE) {
//       // set pts to the default value of 0
//       pts = 0;
//     }

//     // convert pts using video stream's time base
//     pts *= av_q2d(reader->video_st->time_base);

//     // sync video frame using pts
//     pts = reader->synchronize_video(video_frame, pts);

//     if (!reader->queue_picture(video_frame, pts)) {
//       // global quit flag has been set
//       should_quit = true;
//       break;
//     }
//   }
//   // wipe the frame
//   av_frame_unref(video_frame);

//   // wipe the packet
//   av_packet_unref(video_pkt);
// }

// for (int index = 0; index < PICTQ_SIZE; ++index) {
//   if (reader->pictq.queue[index]) {
//     av_frame_free(&(reader->pictq.queue[index]));
//     av_free(reader->pictq.queue[index]);
//   }
// }

// av_frame_free(&video_frame);
// av_free(video_frame);

// av_packet_unref(video_pkt);
// av_packet_free(&video_pkt);

// return;
// }

// int64_t VideoDecoder::guess_correct_pts(AVCodecContext *ctx,
//                                         int64_t &reordered_pts, int64_t
//                                         &dts) {
//   int64_t pts = AV_NOPTS_VALUE;

//   if (dts != AV_NOPTS_VALUE) {
//     ctx->pts_correction_num_faulty_dts += dts <=
//     ctx->pts_correction_last_dts; ctx->pts_correction_last_dts = dts;
//   } else if (reordered_pts != AV_NOPTS_VALUE) {
//     ctx->pts_correction_last_dts = reordered_pts;
//   }

//   if (reordered_pts != AV_NOPTS_VALUE) {
//     ctx->pts_correction_num_faulty_pts +=
//         reordered_pts <= ctx->pts_correction_last_pts;
//     ctx->pts_correction_last_pts = reordered_pts;
//   } else if (dts != AV_NOPTS_VALUE) {
//     ctx->pts_correction_last_pts = dts;
//   }

//   if ((ctx->pts_correction_num_faulty_pts <=
//            ctx->pts_correction_num_faulty_dts ||
//        dts == AV_NOPTS_VALUE) &&
//       reordered_pts != AV_NOPTS_VALUE) {
//     pts = reordered_pts;
//   } else {
//     pts = dts;
//   }

//   return pts;
// }

// double VideoDecoder::synchronize_video(AVFrame *src_frame, double &pts) {
//   double frame_delay;

//   if (pts != 0) {
//     // if we have pts, set the video_clock to it
//     video_clock = pts;
//   } else {
//     // if we don't have a pts, set it to the clock
//     pts = video_clock;
//   }

//   // RESOLVE BEFORE COMMIT
//   // base time between frames
//   frame_delay = av_q2d(video_st->time_base);
//   // calculate how much frame must be delayed
//   frame_delay += 0.5 * frame_delay * src_frame->repeat_pict;

//   // update video clock
//   video_clock += frame_delay;

//   return pts;
// }

// bool VideoDecoder::queue_picture(AVFrame *qFrame, double &pts) {
//   // acquire picture queue mutex
//   std::unique_lock<std::mutex> lk(pictq.mutex);

//   // wait until picture queue has space
//   while (pictq.size >= PICTQ_SIZE && !global_quit) {
//     pictq.cond.wait(lk);
//   }

//   lk.unlock();

//   if (global_quit != 0) {
//     return false;
//   }

//   // check if frame has been correctly allocated
//   if (pictq.queue[pictq.write_index]) {
//     // set updated pts value
//     pictq.pts[pictq.write_index] = pts;

//     // set frame info using last decoded frame
//     pictq.queue[pictq.write_index]->pict_type = qFrame->pict_type;
//     pictq.queue[pictq.write_index]->pts = qFrame->pts;
//     pictq.queue[pictq.write_index]->pkt_dts = qFrame->pkt_dts;
//     pictq.queue[pictq.write_index]->key_frame = qFrame->key_frame;
//     pictq.queue[pictq.write_index]->coded_picture_number =
//         qFrame->coded_picture_number;
//     pictq.queue[pictq.write_index]->display_picture_number =
//         qFrame->display_picture_number;
//     pictq.queue[pictq.write_index]->width = qFrame->width;
//     pictq.queue[pictq.write_index]->height = qFrame->height;

//     // scale image in qFrame->data and put results in pictq frame's data
//     sws_scale(sws_ctx, (uint8_t const *const *)qFrame->data,
//     qFrame->linesize,
//               0, video_ctx->height, pictq.queue[pictq.write_index]->data,
//               pictq.queue[pictq.write_index]->linesize);

//     // update write index
//     ++pictq.write_index;
//     if (pictq.write_index == PICTQ_SIZE) {
//       pictq.write_index = 0;
//     }

//     lk.lock();

//     // increase picture queue size
//     ++pictq.size;
//   } else {
//     std::cerr << "Picture queue hasn't been allocated" << std::endl;
//     global_quit = -1;
//     return false;
//   }

//   return true;
// }

// uint8_t *VideoDecoder::getFrame(double &external_clock) {
//   // check picture queue contains decoded frames
//   if (pictq.size == 0) {
//     // skip updating texture this frame
//     return nullptr;
//   } else {
//     AVFrame *frame = pictq.queue[pictq.read_index];
//     double &pts = pictq.pts[pictq.read_index];
//     // std::cout << " pts(read): " << pts << std::endl;

//     // get last frame pts
//     double pts_delay = pts - frame_last_pts;

//     // check if obtained delay is invalid
//     if (pts_delay <= 0 || pts_delay >= 1.0) {
//       // use previous calculated delay
//       pts_delay = frame_last_delay;
//     }

//     // save delay information
//     frame_last_pts = pts;
//     frame_last_delay = pts_delay;

//     if (master_sync != MasterSync::AV_SYNC_VIDEO) {
//       if (master_sync == MasterSync::AV_SYNC_AUDIO) {
//         master_clock = get_audio_clock();
//       } else {
//         master_clock = external_clock;
//       }

//       double video_diff = pts - master_clock;

//       double sync_threshold =
//           (pts_delay > AV_SYNC_THRESHOLD) ? pts_delay : AV_SYNC_THRESHOLD;

//       // check if audio video delay is below sync threshold
//       if (fabs(video_diff) < AV_NOSYNC_THRESHOLD) {
//         if (video_diff <= -sync_threshold) {
//           gotFrame();
//           return getFrame(external_clock);
//         } else if (video_diff >= sync_threshold) {
//           return nullptr;
//         }
//       } else {
//         // std::cout << video_diff << std::endl;
//         // TODO: utilize dts to handle edge cases
//         if (video_diff > 0) {
//           ++seek_diff_count;
//           if (seek_diff_count > PICTQ_SIZE + 2)
//             return nullptr;
//         }

//         gotFrame();
//         return getFrame(external_clock);
//       }
//     }

//     return (uint8_t *)*frame->extended_data; // same as data[0]
//     // return (uint8_t *)*pictq.queue[pictq.read_index]
//     //     ->extended_data; // same as data[0]
//   }
// }

// void VideoDecoder::gotFrame() {
//   // update read index
//   if (++pictq.read_index == PICTQ_SIZE) {
//     pictq.read_index = 0;
//   }

//   ++currentFrame;

//   // acquire picture queue mutex
//   std::unique_lock<std::mutex> lk(pictq.mutex);

//   // decrease picture queue size
//   pictq.size--;

//   pictq.cond.notify_one();
// }

// void VideoDecoder::audioThreadFunction(VideoDecoder *reader) {
//   while (reader->global_quit == 0) {
//     // check if audio buffer needs to be written into
//     if (reader->audio_buffer[0].writeSpace() >
//     AUDIO_BUFFER_REFRESH_THRESHOLD) {
//       // get more audio data
//       if (reader->audio_decode_frame() < 0) {
//         std::cerr << "audio_decode_frame() failed" << std::endl;
//         reader->global_quit = -1;
//         return;
//       }
//     }
//   }
// }

// int VideoDecoder::audio_decode_frame() {
//   int data_size = 0;

//   // check global quit flag
//   if (global_quit != 0) {
//     return 0;
//   }

// // allocate audio packet
// audio_pkt = av_packet_alloc();
// if (!audio_pkt) {
//   std::cerr << "Could not allocate audio packet" << std::endl;
//   return false;
// }

// // allocate a new frame, used to decode audio packets
// audio_frame = av_frame_alloc();
// if (!audio_frame) {
//   std::cerr << "Could not allocate audio frame" << std::endl;
//   return false;
// }

//   // run while data is still left in audio packet
//   while (audio_pkt->data) {
//     // retrieve audio frame
//     int ret = avcodec_receive_frame(audio_ctx, audio_frame);

//     // need more data
//     if (ret == AVERROR(EAGAIN)) {
//       break;
//     } else if (ret == AVERROR_EOF) {
//       // end of file. let the video play handle quitting
//       // TODO: check where global quit flag is being set
//       return 0;
//     } else if (ret < 0) {
//       std::cerr << "Error receiving audio frame" << std::endl;
//       return -1;
//     }

//     // does this ever happen?
//     // if (audio_sample_size * audio_frame->nb_samples >
//     //     AUDIO_BUFFER_REFRESH_THRESHOLD) {
//     //   std::cerr << "Audio buffer frame size too small" << std::endl;
//     // }

//     for (int i = 0; i < audio_frame->nb_samples; ++i) {
//       for (int ch = 0; ch < audio_ctx->channels; ++ch) {
//         data_size += audio_buffer[ch].write(
//             (const char *)audio_frame->data[ch] + audio_sample_size * i,
//             audio_sample_size);
//       }
//     }

//     // update audio clock as we read through the packet
//     audio_clock += (double)data_size / audio_data_rate;

//     // audio buffer sufficiently filled. come back later
//     // TODO: see if this can be more optimized
//     if (audio_buffer[0].writeSpace() < AUDIO_BUFFER_REFRESH_THRESHOLD) {
//       return data_size;
//     }
//   }

//   // ideally this shouldn't happen
//   if (audio_pkt->data) {
//     // wipe the packet
//     av_packet_unref(audio_pkt);
//   }

//   // get new audio packet from queue
//   if (packet_queue_get(&audioq, audio_pkt, 1) < 0) {
//     // global quit flag has been set
//     return 0;
//   }

//   // flush buffer if flush packet
//   if (audio_pkt->data == flush_pkt->data) {
//     avcodec_flush_buffers(audio_ctx);
//     return data_size;
//   }

//   // send packet to audio codec
//   if (avcodec_send_packet(audio_ctx, audio_pkt) < 0) {
//     std::cerr << "Error sending audio packet for decoding" << std::endl;
//     return -1;
//   }

//   // update audio clock from new packet
//   if (audio_pkt->pts != AV_NOPTS_VALUE) {
//     audio_clock = av_q2d(audio_st->time_base) * audio_pkt->pts;
//   }

//   return data_size;
// }

// double VideoDecoder::get_audio_clock() {
//   return audio_clock - (double)audio_buffer[0].readSpace() *
//                            audio_ctx->channels / audio_data_rate;
// }

// void VideoDecoder::packet_queue_init(PacketQueue *pktq) { pktq->dataSize =
// 0;
// }

// void VideoDecoder::packet_queue_put(PacketQueue *pktq, AVPacket *packet) {
//   // acquire packet queue mutex
//   std::unique_lock<std::mutex> lk(pktq->mutex);

//   // TODO: check unref, av_packet_ref
//   // need to clone the packet to properly allocate and hand over refs
//   pktq->queue.push(av_packet_clone(packet));
//   pktq->dataSize += packet->size;

//   // notify packet_queue_get
//   pktq->cond.notify_one();
// }

// int VideoDecoder::packet_queue_get(PacketQueue *pktq, AVPacket *packet,
//                                   int blocking) {
//   // acquire packet queue mutex
//   std::unique_lock<std::mutex> lk(pktq->mutex);

//   while (global_quit == 0) {
//     if (!pktq->queue.empty()) {
//       AVPacket *queue_pkt = pktq->queue.front();
//       // copy contents from queue
//       av_packet_ref(packet, queue_pkt);
//       pktq->dataSize -= packet->size;

//       // pop handles deallocation
//       pktq->queue.pop();
//       av_packet_free(&queue_pkt);

//       // successful packet retrieval
//       return 1;
//     } else if (!blocking) {
//       // avoid waiting if block = 0
//       return 0;
//     } else {
//       // temporarily release mutex & wait until packet is available
//       pktq->cond.wait(lk);
//     }
//   }

//   // global quit flag has been set
//   return -1;
// }

// void VideoDecoder::packet_queue_flush(PacketQueue *pktq) {
//   // acquire packet queue mutex
//   std::unique_lock<std::mutex> lk(pktq->mutex);

//   while (!pktq->queue.empty()) {
//     AVPacket *pkt = pktq->queue.front();
//     pktq->queue.pop();
//     av_packet_free(&pkt);
//   }

//   pktq->dataSize = 0;
// }

// void VideoDecoder::stream_seek(int64_t pos, int rel) {
//   if (!seek_requested) {
//     seek_pos = pos;
//     seek_flags = (rel < 0) ? AVSEEK_FLAG_BACKWARD : 0;
//     seek_requested = 1;
//     seek_diff_count = 0;
//   }
// }

int VideoDecoder::audioSampleRate() {
  if (video_state.audio_ctx)
    return video_state.audio_ctx->sample_rate;
  return 0;
}
int VideoDecoder::audioNumChannels() {
  if (video_state.audio_ctx)
    return video_state.audio_ctx->channels;
  return 0;
}

int VideoDecoder::width() {
  if (video_state.video_ctx)
    return video_state.video_ctx->width;
  return 0;
}

int VideoDecoder::height() {
  if (video_state.video_ctx)
    return video_state.video_ctx->height;
  return 0;
}

double VideoDecoder::fps() {
  double guess = av_q2d(
      av_guess_frame_rate(video_state.format_ctx, video_state.video_st, NULL));
  if (guess == 0) {
    std::cerr << "Could not guess frame rate" << std::endl;
    guess = av_q2d(video_state.format_ctx->streams[video_state.video_st_idx]
                       ->r_frame_rate);
  }
  return guess;
}

void VideoDecoder::cleanup() {
  // Close the audio codec
  avcodec_close(video_state.audio_ctx);
  // Close the video codec
  avcodec_close(video_state.video_ctx);
  // Close the video file
  avformat_close_input(&video_state.format_ctx);
}

void VideoDecoder::stop() {
  video_state.global_quit = 1;

  if (decode_thread) {
    decode_thread->join();
  }

  std::thread *dth = decode_thread;
  decode_thread = nullptr;
  delete dth;

  cleanup();
}
