

/*	Allocore --
        Multimedia / virtual environment application class library

        Copyright (C) 2009. AlloSphere Research Group, Media Arts & Technology,
   UCSB. Copyright (c) 2015-2016 Donghao Ren Copyright (C) 2012. The Regents of
   the University of California. All rights reserved.

        Redistribution and use in source and binary forms, with or without
        modification, are permitted provided that the following conditions are
   met:

                Redistributions of source code must retain the above copyright
   notice, this list of conditions and the following disclaimer.

                Redistributions in binary form must reproduce the above
   copyright notice, this list of conditions and the following disclaimer in the
                documentation and/or other materials provided with the
   distribution.

                Neither the name of the University of California nor the names
   of its contributors may be used to endorse or promote products derived from
                this software without specific prior written permission.

        THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS
   IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
        IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
   PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
   CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
   EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
   PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
   OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
   WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
   OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
   ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.


        File description:


        File author(s):
        Donghao Ren 2015-2016
        Andrés Cabrera 2017 mantaraya36@gmail.com
*/

#include <atomic>
#include <condition_variable>
#include <iostream>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

extern "C" {
#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
#include <libavutil/pixfmt.h>
#include <libswscale/swscale.h>
}

// Hacks for older ffmpeg
#if LIBAVCODEC_VERSION_INT < AV_VERSION_INT(55, 28, 1)
#define AV_PIX_FMT_RGBA PIX_FMT_RGBA
#define av_frame_alloc avcodec_alloc_frame
#define av_frame_free avcodec_free_frame
#endif

#include "al/app/al_App.hpp"
#include "al/types/al_SingleRWRingBuffer.hpp"
//#include "allocore/system/al_Thread.hpp"

namespace al {

// TODO better handling of buffers and audio/video de-interleaving. Currently
// works OK with mp4 but fails with some mov files.
// TODO more informed choice of buffer sizes for mAudioBuffer and mPictureBuffer
// TODO add options for choosing video streams
// TODO add options to choose audio resampling
// TODO add a function to query if video file is done

class VideoFileReader {
public:
  typedef enum { SYNC_AUDIO, SYNC_VIDEO, SYNC_INTERNAL, SYNC_AUTO } SyncMode;

  typedef enum { PLAY_ONESHOT, PLAY_LOOPING } PlayMode;

  /**
   *
   * \brief A class that wraps libavcodec and libavformat (ffmpeg) for video
   * decoding
   * \param path to video file
   * \param syncMode determines the sync
   * "clock" for media
   * \param outputWidth if different to -1 the output's witdh
   * will be resized to this value
   * \param outputHeight if different to -1 the
   * output's height will be resized to this value
   *
   * This class can read the video file in three modes:
   * SYNC_AUDIO makes the audio request drive the decoding. Whenever
   * audio is read from the buffers using readAudio(), this triggers an update
   * of the current video frame when needed. Use this mode when you want to
   * playback audio and video in sync.
   *
   * SYNC_VIDEO makes the request for video frames drive the decoding. When
   * nextFrame() is called, the next frame is decoded. This allows getting
   * frames sequentially at an arbitrary rate different to the original
   * video's rate.
   *
   * SYNC_INTERNAL makes the computer's internal clock drive the decoding.
   * A separate thread watches the clock and updates the current frame
   * at the rate requested by the frame rate. This might mean that frames
   * might be missed or used twice as the process is asynchronous, but
   * guarantees that the duration and playback speed of the video is accurate
   * and consistent.
   *
   * SYNC_AUTO determines the mode from the contents of the movie file. If an
   * audio track is present in the file, SYNC_AUDIO is used. If no audio is
   * present SYNC_INTERNAL is used.
   */
  VideoFileReader(const char *path, SyncMode syncMode = SYNC_AUTO,
                  int outputWidth = -1, int outputHeight = -1);

  virtual ~VideoFileReader() { cleanup(); }

  void start();

  void stop();

  ///
  /// \brief width
  /// \return The width of the source video frame
  ///
  virtual int width() const { return mPictureCodecContext->width; }

  ///
  /// \brief height
  /// \return The height of the source video frame
  ///
  virtual int height() const { return mPictureCodecContext->height; }

  virtual double fps() { return mFps; }

  virtual double duration() { return mDuration; }

  int currentFrame() { return mCurrentVideoFrame.load(); }

  ///
  /// \brief nextFrame
  /// \return returns true if a new frame was decoded.
  /// Call this function only if the file has no audio track, to force
  /// reading of frames.
  virtual bool nextFrame();

  ///
  /// \brief readAudio
  /// \param channel
  /// \param buf
  /// \param numElements
  /// \return
  ///
  /// When syncing to audio you need to call read audio for at least channel 0,
  /// the free space in this buffer determines whether frames should be read.
  /// Even if you don't use this channel, make sure you call this function
  /// for it to flush its buffer.
  int readAudio(int channel, float *buf, size_t numElements);

  ///
  /// \brief Set pixel buffer where video frames are written
  /// \param buffer
  ///
  /// You need to call this only once, and the pixel buffer will be updated
  /// when required. If you call this function, the video will not be
  /// decoded to the internal buffer, so calling pixels() will return an
  /// empty buffer. You can reset usage to the internal buffer by passing
  /// nullptr as argument.
  virtual void setPixelBuffer(void *buffer);

  ///
  /// \brief returns a pointer to the current frame's pixel data
  ///
  /// Note that this is not inherently safe as the reader thread might be
  /// writing to the array. Make sure you call this synchronously
  ///
  /// \return pointer to frame pixel data
  ///
  virtual const void *pixels() const {
    if (assigned_buffer_) {
      return assigned_buffer_;
    } else {
      return mPictureBuffer[mPicBufRead];
    }
  }

  ///
  /// \brief Move the movie read location to specified frame
  ///
  virtual void seek(int64_t frame);

  void setPlayMode(PlayMode mode) { mPlayMode = mode; }

private:
  void intializeVideoStream();

  void cleanup();

  inline void readVideoFromPacket(VideoFileReader *obj, AVPacket &packet,
                                  AVFrame *frame) {
    int frame_finished;
    /*if (obj->mSkipFrames.load() > 0) {
        obj->mSkipFrames--;
    } else */
    {
      avcodec_decode_video2(obj->mPictureCodecContext, frame, &frame_finished,
                            &packet);
      if (frame_finished) {
        std::unique_lock<std::mutex> videoLock(obj->mVideoFrameMutex);
        sws_scale(obj->mScalingContext, (uint8_t const *const *)frame->data,
                  frame->linesize, 0, obj->mPictureCodecContext->height,
                  obj->frame_rgb_->data, obj->frame_rgb_->linesize);
        //                std::cout << "Video frame: " <<
        //                obj->mCurrentVideoFrame.load() << std::endl;
        obj->mCurrentVideoFrame++;

        // What should be done is frame is not finished at this point?
      }
    }
  }

  ///
  /// \brief static function for reader thread (for SYNC_AUDIO mode)
  ///
  static void readFunction(VideoFileReader *obj);

  ///
  /// \brief static function for picture only reader thread (for SYNC_VIDEO
  /// mode)
  ///
  static void readFunctionNoAudio(VideoFileReader *obj);

  void ffmpeg_initialize();

  AVFormatContext *mFormatContext;
  AVDictionary *mOptionsDict;
  SyncMode mSyncMode;
  PlayMode mPlayMode;

  // Video stream
  AVCodecContext *mPictureCodecContext;
  bool mCodecContextOpened; // True if video stream opened succesfully
  AVCodec *mPictureCodec;
  int mVideoStreamIndex; // Index to video stream
  struct SwsContext *mScalingContext;
  int64_t mTimeBase;
  double mFps;
  double mDuration;
  int mOutputWidth, mOutputHeight; // width and height of the output frame

  // Audio stream
  AVCodecContext *mAudioCodecContext;
  bool mAudioCodecContextOpened; // True if audio stream opened succesfully
  AVCodec *mAudioCodec;
  int mAudioStreamIndex; // Index to audio stream
  int mSampleRate;
  int mAudioChannels;

  // Picture decoding buffers
  AVPacket packet;
  AVFrame *frame_;
  AVFrame *frame_rgb_;
  uint8_t *assigned_buffer_ =
      nullptr;    // if nullptr, then use the picture ring buffer
  int num_bytes_; // Size of the video frame buffers

  std::atomic<uint64_t> mCurrentVideoFrame;
  std::atomic<int64_t> mSeek;
  std::atomic<int> mSkipFrames;

  std::mutex mVideoFrameMutex;
  std::mutex mLock;
  std::condition_variable mCondVar;
  std::thread *mReaderThread;
  std::atomic<bool> mRunning;

  SingleRWRingBuffer mAudioBuffer[2] = {{8192 * 8}, {8192 * 8}};
  std::vector<uint8_t *> mPictureBuffer;
  int mPicBufSize;
  int mPicBufWrite = 1, mPicBufRead = 0;
};

///
/// \brief The VideoTexture class provides a simple way to read video straight
/// into a texture
///
///
class VideoTexture : public Texture {
public:
  VideoTexture(std::string fileName = "test.avi",
               VideoFileReader::SyncMode syncMode = VideoFileReader::SYNC_AUTO,
               int textureWidth = -1, int textureHeight = -1);

  void start() { mVideoReader.start(); }

  void stop() { mVideoReader.stop(); }

  int width() { return mVideoReader.width(); }

  int height() { return mVideoReader.height(); }

  void readFrame();

  int readAudio(int channel, float *buffer, int numElements);

  void seek(int frame);

  int currentFrame() { return mVideoReader.currentFrame(); }

  void setPlayMode(VideoFileReader::PlayMode mode) {
    mVideoReader.setPlayMode(mode);
  }

private:
  VideoFileReader mVideoReader;
};

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
  if (mSyncMode == SYNC_AUDIO || mSyncMode == SYNC_AUTO) {
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
  if (mSyncMode == SYNC_AUDIO || mSyncMode == SYNC_AUTO) {
    mRunning.store(true);
    mReaderThread = new std::thread(readFunction, this);
    while (mAudioBuffer->readSpace() < 8192) {
      mCondVar.notify_one();
    }
  }

  // TODO if syncing to wall clock then spawn reader thread
  if (!mReaderThread && mCodecContextOpened &&
      (mSyncMode == SYNC_INTERNAL || mSyncMode == SYNC_AUTO)) {
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

  for (auto picBuffer : mPictureBuffer) {
    delete picBuffer;
  }
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
    picBuffer = new uint8_t[num_bytes_];
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
    if (obj->mAudioBuffer[0].writeSpace() < 8193) {
      continue;
    }
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
          if (!obj->assigned_buffer_) {
            std::cout << "Decode into index " << obj->mPicBufWrite << std::endl;
            avpicture_fill((AVPicture *)obj->frame_rgb_,
                           obj->mPictureBuffer[obj->mPicBufWrite],
                           AV_PIX_FMT_RGBA, obj->mPictureCodecContext->width,
                           obj->mPictureCodecContext->height);
            int newIndex = obj->mPicBufWrite + 1;
            if (newIndex == obj->mPicBufSize) {
              newIndex = 0;
            }
            obj->mPicBufWrite = newIndex;
          }
          obj->readVideoFromPacket(obj, packet, frame);
          if (!obj->assigned_buffer_) {
          }
          //                        obj->frames_buffer->write((const char
          //                        *)obj->frame_rgb_->data, obj->num_bytes_);
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

VideoTexture::VideoTexture(std::string fileName,
                           VideoFileReader::SyncMode syncMode, int textureWidth,
                           int textureHeight)
    : mVideoReader(fileName.c_str(), syncMode, textureWidth, textureHeight) {
  //  this->target(Texture::TEXTURE_2D);
  this->resize(mVideoReader.width(), mVideoReader.height());
  //  this->allocate();

  //    mVideoReader.setPixelBuffer(this->data());
}

void VideoTexture::readFrame() {
  //        if (frame >= 0 && frame != currentFrame()) {
  //            mVideoReader.seek(frame);
  //        }
  // FIXME need to make sure frame rate at which this is called matches video
  // frame rate
  if (mVideoReader.nextFrame()) {
    // FIXME we need to cpy the data to the texture
    //    memcpy(this->array().data.ptr, mVideoReader.pixels(),
    //           mVideoReader.width() * mVideoReader.height() * sizeof(uint8_t)
    //           * 4);
    //    this->dirty();
  }
}

int VideoTexture::readAudio(int channel, float *buffer, int numElements) {
  return mVideoReader.readAudio(channel, buffer, numElements);
}

void VideoTexture::seek(int frame) { mVideoReader.seek(frame); }

} // namespace al

// -------------------------------------
