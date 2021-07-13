#ifndef AL_VIDEOPLAYER_HPP
#define AL_VIDEOPLAYER_HPP

/*
Copyright (C) 2009, 2021. AlloSphere Research Group, Media Arts & Technology,
UCSB. Copyright (c) 2015-2016 Donghao Ren Copyright (C) 2012. The Regents of the
University of California. All rights reserved.


        File description:


    File author(s):
    Donghao Ren 2015-2016
              Andrés Cabrera 2017 mantaraya36@gmail.com
*/

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
  // TODO should we inherit from VideoFileReader
public:
  bool init(const char *path,
            VideoFileReader::SyncMode syncMode = VideoFileReader::SYNC_AUTO,
            int outputWidth = -1, int outputHeight = -1);

  void start() { mVideoReader->start(); }

  void stop() { mVideoReader->stop(); }

  int width() { return mVideoReader->width(); }

  int height() { return mVideoReader->height(); }

  void readFrame(uint64_t framenum = UINT64_MAX);

  int readAudio(int channel, float *buffer, int numElements);

  void seek(int frame);

  int currentFrame() { return mVideoReader->currentFrame(); }

  void setPlayMode(VideoFileReader::PlayMode mode) {
    mVideoReader->setPlayMode(mode);
  }

private:
  std::unique_ptr<VideoFileReader> mVideoReader;
};

} // namespace al

#endif // AL_VIDEOPLAYER_HPP
