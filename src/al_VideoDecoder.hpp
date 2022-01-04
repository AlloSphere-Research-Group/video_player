#ifndef AL_VIDEODECODER_HPP
#define AL_VIDEODECODER_HPP

extern "C" {
#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
#include <libavutil/imgutils.h>
#include <libswscale/swscale.h>
}

#include <atomic>
#include <condition_variable>
#include <iostream>
#include <mutex>
#include <thread>

#include "al/system/al_Time.hpp"
#include "al_MediaBuffer.hpp"

using namespace al;

static const int AUDIO_BUFFER_SIZE = 1;
static const int VIDEO_BUFFER_SIZE = 1;
static const double AV_SYNC_THRESHOLD = 0.01;
static const double AV_NOSYNC_THRESHOLD = 1.0;

enum MasterSync { AV_SYNC_AUDIO = 0, AV_SYNC_VIDEO = 1, AV_SYNC_EXTERNAL = 2 };

struct VideoState {
  // ** File I/O Context **
  AVFormatContext *format_ctx;

  // ** Video Stream **
  int video_st_idx;
  AVStream *video_st;
  AVCodecContext *video_ctx;
  struct SwsContext *sws_ctx;
  MediaBuffer *video_frames;

  // ** Audio Stream **
  bool audio_enabled;
  int audio_st_idx;
  AVStream *audio_st;
  AVCodecContext *audio_ctx;
  MediaBuffer *audio_frames;
  int audio_sample_size;
  int audio_channel_size;
  int audio_frame_size;

  // ** Sync **
  MasterSync master_sync{MasterSync::AV_SYNC_EXTERNAL};
  double video_clock;
  double audio_clock;
  double master_clock;
  double last_frame_pts;
  double last_frame_delay;

  // ** Seek **
  bool seek_requested;
  int seek_flags;
  int64_t seek_pos;

  // ** Global Quit Flag **
  int global_quit;
};

class VideoDecoder {
public:
  VideoDecoder()
      : video_buffer(VIDEO_BUFFER_SIZE), audio_buffer(AUDIO_BUFFER_SIZE) {
    init();
  }

  ~VideoDecoder() { stop(); }

  // initialize video state
  void init();

  // load video file
  bool load(const char *url);

  // start the threads
  void start();

  // get the next video/audio frame
  uint8_t *getVideoFrame(double external_clock = -1);
  uint8_t *getAudioFrame(double external_clock = -1);

  // seek position in video file
  void stream_seek(int64_t pos, int rel);

  // free memories
  void cleanup();

  // stop the video reader
  void stop();

  /// Utility functions
  // set which clock the playback will sync to
  void setMasterSync(MasterSync newSync) { video_state.master_sync = newSync; };

  // enable/disable audio playback
  void enableAudio(bool enable = true) { video_state.audio_enabled = enable; }

  // file has audio stream
  bool hasAudio() { return video_state.audio_st != nullptr; }

  // get audio parameters
  unsigned int audioSampleRate();
  unsigned int audioNumChannels();
  unsigned int audioSamplesPerChannel();

  // get video parameters
  int width();
  int height();
  double fps();

private:
  // open & initialize video/audio stream components
  bool stream_component_open(VideoState *vs, int stream_index);

  // thread functions for decoding and video
  static void decodeThreadFunction(VideoState *vs);

  // // attempt to guess proper timestamps for decoded video frames
  // int64_t guess_correct_pts(AVCodecContext *ctx, int64_t &reordered_pts,
  //                           int64_t &dts);

  // // update the pts
  // double synchronize_video(AVFrame *src_frame, double &pts);

  // // inserts decoded frame into picture queue
  // bool queue_picture(AVFrame *qFrame, double &pts);

  // // decode audio frame
  // // returns size of decoded audio data if successful, negative on fail
  // int audio_decode_frame();

  // // get the current audio reference clock
  // double get_audio_clock();

  VideoState video_state;

  MediaFrame video_output;
  MediaBuffer video_buffer;

  MediaFrame audio_output;
  MediaBuffer audio_buffer;

  // ** Threads **
  std::thread *decode_thread{nullptr};
};

#endif // AL_VIDEODECODER_HPP
