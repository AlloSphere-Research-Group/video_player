#include "al_VideoDecoder.hpp"

using namespace al;

void VideoDecoder::init() {
  video_state.format_ctx = nullptr;

  video_state.video_st_idx = -1;
  video_state.video_st = nullptr;
  video_state.video_ctx = nullptr;
  video_state.sws_ctx = nullptr;
  video_state.video_frames = nullptr;

  video_state.audio_enabled = true;
  video_state.audio_st_idx = -1;
  video_state.audio_st = nullptr;
  video_state.audio_ctx = nullptr;
  video_state.audio_frames = nullptr;
  video_state.audio_sample_size = 0;
  video_state.audio_channel_size = 0;
  video_state.audio_frame_size = 0;

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

    video_state.audio_frames = &audio_buffer;
  }

  video_state.video_frames = &video_buffer;

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

    // set parameters
    vs->audio_sample_size = av_get_bytes_per_sample(vs->audio_ctx->sample_fmt);
    if (vs->audio_sample_size < 0) {
      std::cerr << "Failed to calculate data size" << std::endl;
      return false;
    }

    vs->audio_channel_size = vs->audio_sample_size * vs->audio_ctx->frame_size;

    vs->audio_frame_size = vs->audio_sample_size * vs->audio_ctx->frame_size *
                           vs->audio_ctx->channels;
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

  // allocate frame
  AVFrame *frameRGB = av_frame_alloc();
  if (!frameRGB) {
    std::cerr << "Could not allocate frameRGB" << std::endl;
    vs->global_quit = -1;
    return;
  }

  int numBytes = av_image_get_buffer_size(AV_PIX_FMT_RGBA, vs->video_ctx->width,
                                          vs->video_ctx->height, 32);
  uint8_t *buffer = (uint8_t *)av_malloc(numBytes * sizeof(uint8_t));

  // Setup pointers and linesize for dst frame and image data buffer
  av_image_fill_arrays(frameRGB->data, frameRGB->linesize, buffer,
                       AV_PIX_FMT_RGBA, vs->video_ctx->width,
                       vs->video_ctx->height, 32);

  uint8_t *audio_out =
      (uint8_t *)av_malloc(vs->audio_frame_size * sizeof(uint8_t));

  // check global quit flag
  while (vs->global_quit == 0) {
    // seeking
    if (vs->seek_requested) {
      int video_stream_index = -1;
      int audio_stream_index = -1;
      int64_t video_seek_target = vs->seek_pos;
      int64_t audio_seek_target = vs->seek_pos;

      if (vs->video_st)
        video_stream_index = vs->video_st_idx;
      if (vs->audio_st && vs->audio_enabled)
        audio_stream_index = vs->audio_st_idx;

      if (video_stream_index >= 0) {
        video_seek_target = av_rescale_q(
            video_seek_target, AV_TIME_BASE_Q,
            vs->format_ctx->streams[video_stream_index]->time_base);
      }
      if (audio_stream_index >= 0) {
        audio_seek_target = av_rescale_q(
            audio_seek_target, AV_TIME_BASE_Q,
            vs->format_ctx->streams[audio_stream_index]->time_base);
      }

      int ret = av_seek_frame(vs->format_ctx, video_stream_index,
                              video_seek_target, vs->seek_flags);
      if (vs->audio_st && vs->audio_enabled)
        ret &= av_seek_frame(vs->format_ctx, audio_stream_index,
                             audio_seek_target, vs->seek_flags);

      if (ret < 0) {
        std::cerr << "Error while seeking" << std::endl;
      } else {
        if (vs->video_st) {
          vs->video_frames->flush();
        }
        if (vs->audio_st && vs->audio_enabled) {
          vs->audio_frames->flush();
        }
      }

      vs->seek_requested = 0;
    }

    // read the next frame
    if (av_read_frame(vs->format_ctx, packet) < 0) {
      // no read error; wait for file
      if (vs->format_ctx->pb->error == 0) {
        // al_sleep_nsec(100000000); // 10 ms
        continue;
      } else {
        std::cerr << "Error reading frame" << std::endl;
        break;
      }
    }

    // check the type of packet
    if (packet->stream_index == vs->video_st_idx) {
      // send raw compressed video data in AVPacket to decoder
      if (avcodec_send_packet(vs->video_ctx, packet) < 0) {
        std::cerr << "Error sending video packet for decoding" << std::endl;
        break;
      }

      while (vs->global_quit == 0) {
        // get decoded output data from decoder
        int ret = avcodec_receive_frame(vs->video_ctx, frame);

        // check if entire frame was decoded
        if (ret == AVERROR(EAGAIN)) {
          // need more data
          break;
        } else if (ret == AVERROR_EOF) {
          vs->global_quit = 1;
          break;
        } else if (ret < 0) {
          std::cerr << "Error while decoding" << std::endl;
          vs->global_quit = -1;
          break;
        }

        // get the estimated time stamp
        double pts = frame->best_effort_timestamp;

        // if guess failed
        if (pts == AV_NOPTS_VALUE) {
          // if we don't have a pts, use the video clock
          pts = vs->video_clock;
        } else {
          // convert pts using video stream's time base
          pts *= av_q2d(vs->video_st->time_base);

          // if we have pts, set the video_clock to it
          vs->video_clock = pts;
        }

        // update video clock if frame is delayed
        vs->video_clock +=
            0.5 * av_q2d(vs->video_st->time_base) * frame->repeat_pict;

        // if (vs->video_clock < vs->master_clock) {
        //   break;
        // }

        // scale image in frame and put results in frameRGB
        sws_scale(vs->sws_ctx, (uint8_t const *const *)frame->data,
                  frame->linesize, 0, vs->video_ctx->height, frameRGB->data,
                  frameRGB->linesize);

        std::unique_lock<std::mutex> lk(vs->video_frames->mutex);

        while (!vs->video_frames->put(MediaFrame(buffer, numBytes, pts))) {
          if (vs->global_quit != 0 || vs->seek_requested) {
            break;
          }

          vs->video_frames->cond.wait(lk);
        }
      }
    } else if (packet->stream_index == vs->audio_st_idx) {
      // skip if audio has been disabled
      if (!vs->audio_enabled) {
        av_packet_unref(packet);
        continue;
      }

      if (avcodec_send_packet(vs->audio_ctx, packet) < 0) {
        std::cerr << "Error sending audio packet for decoding" << std::endl;
        break;
      }

      while (vs->global_quit == 0) {
        // get decoded output data from decoder
        int ret = avcodec_receive_frame(vs->audio_ctx, frame);

        // check if entire frame was decoded
        if (ret == AVERROR(EAGAIN)) {
          // need more data
          break;
        } else if (ret == AVERROR_EOF) {
          vs->global_quit = 1;
          break;
        } else if (ret < 0) {
          std::cerr << "Error while decoding" << std::endl;
          vs->global_quit = -1;
          break;
        }

        // get the estimated time stamp
        double pts = frame->best_effort_timestamp;

        // if guess failed
        if (pts == AV_NOPTS_VALUE) {
          // if we don't have a pts, use the audio clock
          pts = vs->audio_clock;
        } else {
          // convert pts using video stream's time base
          pts *= av_q2d(vs->audio_st->time_base);

          // if we have a pts, set the audio clock
          vs->audio_clock = pts;
        }

        // update audio clock if frame is delayed
        vs->audio_clock +=
            0.5 * av_q2d(vs->audio_st->time_base) * frame->repeat_pict;

        // if (vs->audio_clock < vs->master_clock) {
        //   break;
        // }

        for (int i = 0; i < vs->audio_ctx->channels; ++i) {
          memcpy(audio_out + i * vs->audio_channel_size, frame->data[i],
                 vs->audio_channel_size);
        }

        std::unique_lock<std::mutex> lk(vs->audio_frames->mutex);

        while (!vs->audio_frames->put(
            MediaFrame(audio_out, vs->audio_frame_size, pts))) {
          if (vs->global_quit != 0 || vs->seek_requested) {
            break;
          }

          vs->audio_frames->cond.wait(lk);
        }
      }
    }

    // wipe the packet
    av_packet_unref(packet);
  }
  // free the memory
  av_freep(&audio_out);
  av_freep(&buffer);
  av_frame_free(&frameRGB);
  av_frame_free(&frame);
  av_packet_free(&packet);
}

uint8_t *VideoDecoder::getVideoFrame(double external_clock) {
  // get next video frame
  if (!video_buffer.get(video_output) || video_state.seek_requested) {
    return nullptr;
  }

  // get current/last frame pts and delay
  double &pts = video_output.pts;
  double pts_delay = pts - video_state.last_frame_pts;

  // check if obtained delay is invalid
  if (pts_delay <= 0 || pts_delay >= 1.0) {
    // use previous calculated delay
    pts_delay = video_state.last_frame_delay;
  }

  // save pts/delay information
  video_state.last_frame_pts = pts;
  video_state.last_frame_delay = pts_delay;

  if (video_state.master_sync == MasterSync::AV_SYNC_VIDEO) {
    // update master clock if video sync
    video_state.master_clock = pts;
  } else {
    // update master clock if external sync
    if (video_state.master_sync == MasterSync::AV_SYNC_EXTERNAL) {
      video_state.master_clock = external_clock;
    }

    // // difference between target pts and current master clock
    // double video_diff = pts - video_state.master_clock;

    // sync video if needed
    // if (fabs(video_diff) > AV_SYNC_THRESHOLD) {
    //   std::cout << "video_diff: " << video_diff << std::endl;
    // }
  }

  return video_output.data.data();
}

uint8_t *VideoDecoder::getAudioFrame(double external_clock) {
  // get next audio frame
  if (!audio_buffer.get(audio_output) || video_state.seek_requested) {
    return nullptr;
  }

  if (video_state.master_sync == MasterSync::AV_SYNC_AUDIO) {
    // update master clock if audio sync
    video_state.master_clock = audio_output.pts;
  }
  // else {
  //   // difference between target pts and current master clock
  //   double audio_diff = audio_output.pts - video_state.master_clock;

  //   // sync audio if needed
  //   if (fabs(audio_diff) > AV_SYNC_THRESHOLD) {
  //     std::cout << " audio_diff: " << audio_diff << std::endl;
  //   }
  // }

  return audio_output.data.data();
}

void VideoDecoder::stream_seek(int64_t pos, int rel) {
  if (!video_state.seek_requested) {
    video_state.seek_pos = pos;
    // TODO: check which flag to use
    video_state.seek_flags = (rel < 0) ? AVSEEK_FLAG_BACKWARD : 0;
    video_state.seek_requested = 1;
  }
}

unsigned int VideoDecoder::audioSampleRate() {
  if (video_state.audio_ctx)
    return video_state.audio_ctx->sample_rate;
  return 0;
}

unsigned int VideoDecoder::audioNumChannels() {
  if (video_state.audio_ctx)
    return video_state.audio_ctx->channels;
  return 0;
}

unsigned int VideoDecoder::audioSamplesPerChannel() {
  if (video_state.audio_ctx)
    return video_state.audio_ctx->frame_size;
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
  avcodec_free_context(&video_state.audio_ctx);
  // Close the video codec
  avcodec_free_context(&video_state.video_ctx);
  // free the sws context
  sws_freeContext(video_state.sws_ctx);
  // Close the video file
  avformat_close_input(&video_state.format_ctx);
}

void VideoDecoder::stop() {
  video_state.global_quit = 1;
  video_buffer.cond.notify_one();
  audio_buffer.cond.notify_one();

  if (decode_thread) {
    decode_thread->join();
  }
  std::thread *dth = decode_thread;
  decode_thread = nullptr;
  delete dth;

  cleanup();
}
