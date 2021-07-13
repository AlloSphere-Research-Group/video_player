
#include <atomic>
#include <condition_variable>
#include <iostream>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

//#include "allocore/system/al_Thread.hpp"

#include "al_VideoPlayer.hpp"

using namespace al;

// --------------------------- Implementation

VideoFileReader::VideoFileReader(const char *path,
                                 VideoFileReader::SyncMode syncMode,
                                 int outputWidth, int outputHeight) {
  mFormatContext = nullptr;
  mPictureCodecContext = nullptr;
  mCodecContextOpened = false;
  mPictureCodec = nullptr;
  frame_rgb_ = nullptr;
  mOptionsDict = nullptr;
  mCurrentVideoFrame = 0;
  mRunning.store(false);
  mSeek.store(-1);
  mSkipFrames.store(0);
  mReaderThread = nullptr;
  mOutputWidth = outputWidth;
  mOutputHeight = outputHeight;
  mSyncMode = syncMode;
  mPlayMode = PlayMode::PLAY_ONESHOT;
  mPicBufSize = 15;

  ffmpeg_initialize();

  if (avformat_open_input(&mFormatContext, path, NULL, NULL) != 0) {
    std::cerr << "File not found!! " << path << std::endl;
    cleanup();
    throw; // invalid_argument("could not open input file");
  }

  av_dump_format(mFormatContext, 0, path, 0);

  intializeVideoStream();

  // Open Audio Stream

  for (int i = 0; i < mFormatContext->nb_streams; i++) {
    if (mFormatContext->streams[i]->codec->codec_type == AVMEDIA_TYPE_AUDIO) {
      mAudioStreamIndex = i;
      break;
    }
  }
  mAudioCodecContextOpened = false;
  if (mSyncMode == SYNC_AUDIO) {
    if (mAudioStreamIndex >= 0) {
      mAudioCodecContext = mFormatContext->streams[mAudioStreamIndex]->codec;
      mAudioCodec = avcodec_find_decoder(mAudioCodecContext->codec_id);
    }
    if (mAudioCodec != NULL) {
      if (avcodec_open2(mAudioCodecContext, mAudioCodec, &mOptionsDict) < 0) {
        std::cout << "Error opening audio stream" << std::endl;
      } else {
        mSampleRate = mAudioCodecContext->sample_rate;
        mAudioChannels = mAudioCodecContext->channels;
        mAudioCodecContextOpened = true;
        //        std::cout << " sr: " << mSampleRate << " channels: " <<
        //        mAudioChannels
        //                  << std::endl;
      }
    }
    if (mSyncMode == SYNC_AUDIO && !mAudioCodec) {
      std::cerr
          << "ERROR: Audio Stream not found and sync mode set to SYNC_AUDIO."
          << std::endl;
      cleanup();
      throw;
    } else {
      mSyncMode = SYNC_AUDIO;
    }
  }
}

void VideoFileReader::start() {
  if (mReaderThread) {
    stop();
  }
  if (mSyncMode == SYNC_AUDIO) {
    mRunning.store(true);
    mReaderThread = new std::thread(readFunction, this);
    while (mAudioBuffer->readSpace() < 8192) {
      mCondVar.notify_one();
    }
  }

  // TODO if syncing to wall clock then spawn reader thread
  if (!mReaderThread && mCodecContextOpened &&
      (mSyncMode == SYNC_INTERNAL || mSyncMode == SYNC_FREE)) {
    mRunning.store(true);
    mReaderThread = new std::thread(readFunctionNoAudio, this);
  }
}

void VideoFileReader::stop() {
  mRunning.store(false);
  if (mReaderThread) {
    mCondVar.notify_one();
    mReaderThread->join();
  }
  std::thread *th = mReaderThread;
  mReaderThread = nullptr;
  delete th;
  mAudioBuffer->clear();
  mPicBufRead = 0;
  mPicBufWrite = 1;
}

void VideoFileReader::cleanup() {
  mRunning = false;
  if (mCodecContextOpened) {
    avcodec_close(mPictureCodecContext);
  }

  // clean up
  if (frame_) {
    av_frame_free(&frame_);
    frame_ = nullptr;
  }
  if (frame_rgb_) {
    av_free(frame_rgb_);
    frame_rgb_ = NULL;
  }
  if (mFormatContext) {
    avformat_close_input(&mFormatContext);
    mFormatContext = NULL;
  }
  // Audio
  if (mAudioCodecContextOpened) {
    avcodec_close(mAudioCodecContext);
  }
  if (mReaderThread) {
    mCondVar.notify_one();
    mReaderThread->join();
  }

  //  for (auto picBuffer : mPictureBuffer) {
  //    delete picBuffer;
  //  }
}

bool VideoFileReader::nextFrame() {
  bool gotFrame = false;
  if (mSyncMode == SYNC_AUDIO) {
    if (mRunning.load()) {
      std::unique_lock<std::mutex> videoLock(mVideoFrameMutex);
      // If separate thread running, it should do its thing.
      int delta = mPicBufWrite - mPicBufRead;

      //        if (mPicBufWrite != mPicBufRead) {
      if (delta > 1 || delta <= 3 - mPicBufSize) {
        int newReadIndex = mPicBufRead + 1;
        if (newReadIndex == mPicBufSize) {
          newReadIndex = 0;
        }
        mPicBufRead = newReadIndex;
        gotFrame = true;
      } else {
        gotFrame = false;
      }

      std::cout << "nextFrame() " << gotFrame << " " << mPicBufRead
                << std::endl;
    }
  } else {
    while (!gotFrame) {
      if (av_read_frame(mFormatContext, &packet) >= 0) {
        if (packet.stream_index == mVideoStreamIndex) {
          readVideoFromPacket(this, packet, frame_);
          if (!assigned_buffer_) {
            std::cout << "Decode into index " << mPicBufWrite << std::endl;
            avpicture_fill((AVPicture *)frame_rgb_,
                           mPictureBuffer[mPicBufWrite].data(), AV_PIX_FMT_RGBA,
                           mPictureCodecContext->width,
                           mPictureCodecContext->height);
            int newIndex = mPicBufWrite + 1;
            if (newIndex == mPicBufSize) {
              newIndex = 0;
            }
            mPicBufWrite = newIndex;
          }
          gotFrame = true;
        }
      } else {
        break;
      }
    }
  }
  return gotFrame;
}

int VideoFileReader::readAudio(int channel, float *buf, size_t numElements) {
  size_t bytesRead =
      mAudioBuffer[channel].read((char *)buf, numElements * sizeof(float));
  //         if (channel == 0 && bytesRead > 0) {
  mCondVar.notify_one();
  //         }
  return bytesRead / sizeof(float);
}

// setPixelBuffer gives avcodec a pointer to write future video data to
//
void VideoFileReader::setPixelBuffer(void *buffer) {
  //    std::unique_lock<std::mutex> videoLock(mVideoFrameMutex);
  std::cout << mPictureCodecContext->width << ","
            << mPictureCodecContext->height << std::endl;

  assigned_buffer_ = (uint8_t *)buffer;
  if (buffer) {
    avpicture_fill((AVPicture *)frame_rgb_, assigned_buffer_, AV_PIX_FMT_RGBA,
                   mPictureCodecContext->width, mPictureCodecContext->height);
  } else {
    avpicture_fill((AVPicture *)frame_rgb_, nullptr, AV_PIX_FMT_RGBA,
                   mPictureCodecContext->width, mPictureCodecContext->height);
  }
}

void VideoFileReader::seek(int64_t frame) {
  if (mRunning.load()) {
    mSeek.store(frame);
  } else { // Synchronous reading, set directly
    //        int64_t seek_target = av_rescale_q(seek_target, AV_TIME_BASE_Q,
    //        mTimeBase);
    if (av_seek_frame(mFormatContext, mVideoStreamIndex, frame,
                      AVSEEK_FLAG_FRAME | AVSEEK_FLAG_ANY) != 0) {
      std::cerr << "Error seeking to frame " << frame << std::endl;
    }
    avcodec_flush_buffers(mPictureCodecContext);
  }
}

void VideoFileReader::intializeVideoStream() {

  if (avformat_find_stream_info(mFormatContext, NULL) < 0) {
    cleanup();
    throw; // invalid_argument("could not find stream information");
  }

  mVideoStreamIndex = -1;
  for (int i = 0; i < mFormatContext->nb_streams; i++) {
    if (mFormatContext->streams[i]->codec->codec_type == AVMEDIA_TYPE_VIDEO) {
      mVideoStreamIndex = i;
      break;
    }
  }

  if (mVideoStreamIndex == -1) {
    cleanup();
    throw; // invalid_argument("could not find video stream");
  }

  mPictureCodecContext = mFormatContext->streams[mVideoStreamIndex]->codec;
  mPictureCodec = avcodec_find_decoder(mPictureCodecContext->codec_id);

  if (mPictureCodec == NULL) {
    cleanup();
    throw;
    //             throw invalid_argument("could not find video decoder");
  }

  if (avcodec_open2(mPictureCodecContext, mPictureCodec, &mOptionsDict) < 0) {
    cleanup();
    throw;
    //             throw invalid_argument("could not open codec");
  }
  mCodecContextOpened = true;

  frame_rgb_ = av_frame_alloc();
  frame_ = av_frame_alloc();
  //  if (!frame_ or !frame_rgb_) {
  //    fprintf(stderr, "Error allocating the decode frame\n");
  //    cleanup();
  //    throw;
  //  }

  av_init_packet(&packet);

  num_bytes_ = avpicture_get_size(AV_PIX_FMT_RGBA, mPictureCodecContext->width,
                                  mPictureCodecContext->height);

  mPictureBuffer.resize(mPicBufSize);
  for (auto &picBuffer : mPictureBuffer) {
    picBuffer.resize(num_bytes_);
  }

  if (mOutputHeight == -1) {
    mOutputHeight = mPictureCodecContext->height;
  }
  if (mOutputWidth == -1) {
    mOutputWidth = mPictureCodecContext->width;
  }
  mScalingContext =
      sws_getContext(mPictureCodecContext->width, mPictureCodecContext->height,
                     mPictureCodecContext->pix_fmt, mOutputWidth, mOutputHeight,
                     AV_PIX_FMT_RGBA, SWS_FAST_BILINEAR, NULL, NULL, NULL);

  // Get movie file information
  mTimeBase = (int64_t(mPictureCodecContext->time_base.num) * AV_TIME_BASE) /
              int64_t(mPictureCodecContext->time_base.den);
  mFps = (double)mFormatContext->streams[mVideoStreamIndex]->r_frame_rate.num /
         (double)mFormatContext->streams[mVideoStreamIndex]->r_frame_rate.den;
  mDuration = (double)mFormatContext->duration / (double)AV_TIME_BASE;
}

void VideoFileReader::readFunction(VideoFileReader *obj) {
  AVPacket packet;
  av_init_packet(&packet);
  AVFrame *frame = av_frame_alloc();
  if (!frame) {
    std::cerr << "Error allocating the decode frame" << std::endl;
    return;
  }

  while (obj->mRunning.load()) {
    std::unique_lock<std::mutex> lk(obj->mLock);
    bool videoFinished = false;
    obj->mCondVar.wait(lk);
    //    if (obj->mAudioBuffer[0].writeSpace() < 8193) {
    //      // Audio has not been consumed
    //      continue;
    //    }
    int seekFrame = obj->mSeek.load();
    if (seekFrame >= 0) { // Process seek request
      if (avformat_seek_file(obj->mFormatContext, obj->mVideoStreamIndex, 0,
                             seekFrame, seekFrame, AVSEEK_FLAG_FRAME) != 0) {
        std::cerr << "Error seeking to frame " << seekFrame << std::endl;
      }
      //            if (av_seek_frame(obj->mFormatContext,
      //            obj->mVideoStreamIndex, seekFrame, AVSEEK_FLAG_FRAME) != 0)
      //            {
      //                std::cerr << "Error seeking to frame " << seekFrame <<
      //                std::endl;
      //            }
      avcodec_flush_buffers(obj->mPictureCodecContext);
      avcodec_flush_buffers(obj->mAudioCodecContext);
      obj->mSeek.store(-1);
      for (size_t i = 0; i < obj->mAudioChannels; i++) {
        obj->mAudioBuffer[i].clear();
        obj->mPicBufRead = obj->mPicBufWrite;
        obj->mPicBufWrite++;
      }
    }
    while (obj->mAudioBuffer[0].writeSpace() > 8192 && !videoFinished) {
      if (av_read_frame(obj->mFormatContext, &packet) >= 0) {
        if (packet.stream_index == obj->mVideoStreamIndex) {
          obj->readVideoFromPacket(obj, packet, frame);
          if (!obj->assigned_buffer_) {
            std::cout << "Decode into index " << obj->mPicBufWrite << std::endl;
            avpicture_fill((AVPicture *)obj->frame_rgb_,
                           obj->mPictureBuffer[obj->mPicBufWrite].data(),
                           AV_PIX_FMT_RGBA, obj->mPictureCodecContext->width,
                           obj->mPictureCodecContext->height);
            int newIndex = obj->mPicBufWrite + 1;
            if (newIndex == obj->mPicBufSize) {
              newIndex = 0;
            }
            obj->mPicBufWrite = newIndex;
          }
          //          if (!obj->assigned_buffer_) {
          //          }

        } else if (packet.stream_index == obj->mAudioStreamIndex &&
                   obj->mAudioCodecContext) {
          while (packet.size > 0) { // Audio frames can span multiple packets
            int frame_finished;
            if (avcodec_decode_audio4(obj->mAudioCodecContext, frame,
                                      &frame_finished, &packet) >= 0) {
              if (frame_finished) {
                int maxFramesToRead = obj->mAudioBuffer[0].writeSpace() /
                                      (obj->mAudioChannels * sizeof(float));
                size_t unpadded_linesize =
                    frame->nb_samples *
                    av_get_bytes_per_sample(
                        obj->mAudioCodecContext->sample_fmt);
                int data_size = av_samples_get_buffer_size(
                    NULL, obj->mAudioCodecContext->channels, frame->nb_samples,
                    obj->mAudioCodecContext->sample_fmt, 1);

                int byteDataSize = unpadded_linesize;
                for (size_t i = 0; i < obj->mAudioChannels; i++) {
                  int written = obj->mAudioBuffer[i].write(
                      (const char *)frame->extended_data[i], byteDataSize);
                  if (written < byteDataSize) {
                    std::cerr << "Audio Overrun!" << std::endl;
                  }
                  //                                        std::cout << i << ":
                  //                                        Data size: " <<
                  //                                        data_size << " Write
                  //                                        space: " <<
                  //                                        obj->mAudioBuffer[i].writeSpace()
                  //                                        << std::endl;
                }
                int decoded = FFMIN(maxFramesToRead, packet.size);
                packet.data += decoded;
                packet.size -= decoded;
              } else {
                std::cerr << "frame not done" << std::endl;
                break;
              }
              //                         // resample frames
              //                         double* buffer;
              //                         av_samples_alloc((uint8_t**) &buffer,
              //                         NULL, 1, frame->nb_samples,
              //                         AV_SAMPLE_FMT_DBL, 0); int frame_count
              //                         = swr_convert(swr, (uint8_t**) &buffer,
              //                         frame->nb_samples, (const uint8_t**)
              //                         frame->data, frame->nb_samples);
              //                         // append resampled frames to data
              //                         *data = (double*) realloc(*data, (*size
              //                         + frame->nb_samples) * sizeof(double));
              //                         memcpy(*data + *size, buffer,
              //                         frame_count * sizeof(double)); *size +=
              //                         frame_count;
            }
          }
        }
      } else { // No packet read
        videoFinished = true;
      }
    }
    ////		 if (obj->mReadCallback) {
    ////		     obj->mReadCallback(obj->mFileBuffer,
    /// obj->mSf.channels(), framesRead, obj->mCallbackData); /		 }
    ///
    if (videoFinished) {
      if (obj->mPlayMode == VideoFileReader::PLAY_LOOPING) {
        obj->mSeek = 0;
        std::cout << "Starting over." << std::endl;
      } else {
        obj->mRunning = false;
      }
    }
  }

  // clean up
  av_frame_free(&frame);
}

void VideoFileReader::readFunctionNoAudio(VideoFileReader *obj) {
  std::cout << "starting..................." << std::endl;
  while (obj->mRunning.load()) {
    auto start = std::chrono::high_resolution_clock::now();
    int seekFrame = obj->mSeek.load();
    if (seekFrame >= 0) { // Process seek request
      if (avformat_seek_file(obj->mFormatContext, obj->mVideoStreamIndex, 0,
                             seekFrame, seekFrame, AVSEEK_FLAG_FRAME) != 0) {
        std::cerr << "Error seeking to frame " << seekFrame << std::endl;
      }
      //            if (av_seek_frame(obj->mFormatContext,
      //            obj->mVideoStreamIndex, seekFrame, AVSEEK_FLAG_FRAME |
      //            AVSEEK_FLAG_ANY) != 0) {
      //                std::cerr << "Error seeking to frame " << seekFrame <<
      //                std::endl;
      //            }
      avcodec_flush_buffers(obj->mPictureCodecContext);
      avcodec_flush_buffers(obj->mAudioCodecContext);
      obj->mSeek.store(-1);
      for (size_t i = 0; i < obj->mAudioChannels; i++) {
        obj->mAudioBuffer[i].clear();
        obj->mPicBufRead = obj->mPicBufWrite;
        //                obj->mPictureBuffer->clear();
      }
    }

    bool gotFrame = false;
    bool videoFinished = false;
    while (!gotFrame) {
      if (av_read_frame(obj->mFormatContext, &obj->packet) >= 0) {
        if (obj->packet.stream_index == obj->mVideoStreamIndex) {
          if (obj->mSkipFrames.load() > 0) {
            obj->mSkipFrames--;
            continue;
          }
          obj->readVideoFromPacket(obj, obj->packet, obj->frame_);
          gotFrame = true;
        }
      } else {
        videoFinished = true;
      }
    }

    auto duration = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::high_resolution_clock::now() - start);
    double frameDur = 1000000000.0 / obj->fps();
    //            std::cout <<  "Decode time:" << duration.count() << std::endl;
    if (duration.count() > frameDur) {
      double overflow = duration.count() - frameDur;
      while (int(overflow / 1000.0) > 0) {
        obj->mSkipFrames++;
        overflow -= frameDur;
      }
      std::cout << obj->mSkipFrames.load() << " frame(s) skipped!" << std::endl;
    }
    al_sleep(
        (1.0 / obj->fps()) -
        duration.count() /
            1000000000.0); // acummulates a little drift but is cheaper than the
                           // alternative of counting against absolute time
    if (videoFinished) {
      if (obj->mPlayMode == VideoFileReader::PLAY_LOOPING) {
        obj->mSeek = 0;
        std::cout << "Starting over." << std::endl;
      } else {
        obj->mRunning = false;
      }
    }
  }
  std::cout << "ending..................." << std::endl;
}

void VideoFileReader::ffmpeg_initialize() {
  static bool ffmpeg_initialized = false;
  if (!ffmpeg_initialized) {
    av_register_all();
    ffmpeg_initialized = true;
  }
}

// -------------------------------------------

void setVideoFile(const char *filename) {}

bool VideoTexture::init(const char *path, VideoFileReader::SyncMode syncMode,
                        int outputWidth, int outputHeight) {

  mVideoReader = std::make_unique<VideoFileReader>(path, syncMode, outputWidth,
                                                   outputHeight);
  this->resize(mVideoReader->width(), mVideoReader->height());
  // TODO return false on failure
  return true;
}

void VideoTexture::readFrame(uint64_t framenum) {
  // FIXME need to make sure frame rate at which this is called matches video
  // frame rate
  if (framenum != UINT64_MAX) {
    if (framenum >= 0 && framenum != currentFrame()) {
      mVideoReader->seek(framenum);
    }
  }
  if (mVideoReader->nextFrame()) {

    //    for (auto i = 0; i < 300000; i++) {
    //      mVideoReader->pixels()[i] = 255;
    //    }
    this->submit(mVideoReader->pixels());
  }
}

int VideoTexture::readAudio(int channel, float *buffer, int numElements) {
  return mVideoReader->readAudio(channel, buffer, numElements);
}

void VideoTexture::seek(int frame) { mVideoReader->seek(frame); }

// -------------------------------------
