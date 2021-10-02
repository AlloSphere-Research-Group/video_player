#ifndef AL_VIDEOREADER_HPP
#define AL_VIDEOREADER_HPP

extern "C" {
#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
#include <libavutil/imgutils.h>
#include <libswscale/swscale.h>
}

#include <condition_variable>
#include <iostream>
#include <mutex>
#include <queue>
#include <thread>

#include "al/system/al_Time.hpp"
#include "al/types/al_SingleRWRingBuffer.hpp"

using namespace al;

static const int AUDIO_BUFFER_SIZE = 8192 * 10; // 24
static const int AUDIO_BUFFER_REFRESH_THRESHOLD = 8192;
static const int MAX_AUDIOQ_SIZE = (5 * 16 * 1024 * 4);
static const int MAX_VIDEOQ_SIZE = (5 * 256 * 1024); // * 16
static const int PICTQ_SIZE = 1;
static const double AV_SYNC_THRESHOLD = 0.01;
static const double AV_NOSYNC_THRESHOLD = 1.0;

enum MasterSync { AV_SYNC_AUDIO = 0, AV_SYNC_VIDEO = 1, AV_SYNC_EXTERNAL = 2 };

// queue structure to store AVPackets
typedef struct PacketQueue {
  std::queue<AVPacket *> queue;
  int dataSize;
  std::mutex mutex;
  std::condition_variable cond;
} PacketQueue;

// queue structure to store picture frames
typedef struct PictureQueue {
  std::vector<AVFrame *> queue;
  std::vector<double> pts;
  int write_index;
  int read_index;
  int size;
  std::mutex mutex;
  std::condition_variable cond;
} PictureQueue;

class VideoReader {
public:
  VideoReader();

  ~VideoReader() {} // cleanup here?

  // initialize video state
  void init();

  // load video file
  bool load(const char *url);

  // start the threads
  void start();

  // file has audio stream
  bool hasAudio() { return audio_st != nullptr; }

  // enable/disable audio playback
  void enableAudio(bool enable = true) { audio_enabled = enable; }

  // get audio parameters
  int audioSampleRate();
  int audioNumChannels();

  // set which clock the playback will sync to
  void setMasterSync(MasterSync newSync) { master_sync = newSync; };

  // get video parameters
  int width();
  int height();
  double fps();

  // packet queue status
  int audioq_dataSize() { return audioq.dataSize; };
  int videoq_dataSize() { return videoq.dataSize; };

  // get the next frame data
  uint8_t *getFrame(double &external_clock);
  // notify frame has been rendered
  void gotFrame();
  // TODO: use inherent frame number
  uint16_t getCurrentFrameNumber() { return currentFrame; }

  // get the audio channel buffer
  SingleRWRingBuffer *getAudioBuffer(int channel) {
    return &(audio_buffer[channel]);
  }

  void stream_seek(int64_t pos, int rel);

  // free memories
  void cleanup();

  // stop the video reader
  void stop();

private:
  // open & initialize video/audio stream components
  bool stream_component_open(int stream_index);

  // thread functions for decoding and video
  static void decodeThreadFunction(VideoReader *reader);
  static void videoThreadFunction(VideoReader *reader);
  static void audioThreadFunction(VideoReader *reader);

  // attempt to guess proper timestamps for decoded video frames
  int64_t guess_correct_pts(AVCodecContext *ctx, int64_t &reordered_pts,
                            int64_t &dts);

  // update the pts
  double synchronize_video(AVFrame *src_frame, double &pts);

  // inserts decoded frame into picture queue
  bool queue_picture(AVFrame *qFrame, double &pts);

  // decode audio frame
  // returns size of decoded audio data if successful, negative on fail
  int audio_decode_frame();

  // get the current audio reference clock
  double get_audio_clock();

  // functions to manage packet queue
  void packet_queue_init(PacketQueue *pktq);
  void packet_queue_put(PacketQueue *pktq, AVPacket *packet);
  // returns -1 if global quit flag is set, 0 if queue is empty, 1 if packet has
  // been extracted
  int packet_queue_get(PacketQueue *pktq, AVPacket *packet, int blocking);
  void packet_queue_flush(PacketQueue *pktq);

  // ** File I/O Context **
  AVFormatContext *pFormatCtx;

  // ** Audio Stream **
  int audio_st_idx;
  AVStream *audio_st;
  AVCodecContext *audio_ctx;
  PacketQueue audioq;
  std::vector<SingleRWRingBuffer> audio_buffer;
  AVFrame *audio_frame;
  AVPacket *audio_pkt;
  int audio_sample_size;
  int audio_data_rate;

  // ** Video Stream **
  int video_st_idx;
  AVStream *video_st;
  AVCodecContext *video_ctx;
  PacketQueue videoq;
  PictureQueue pictq;
  struct SwsContext *sws_ctx;

  // ** Sync **
  MasterSync master_sync{MasterSync::AV_SYNC_EXTERNAL};
  double video_clock;
  double audio_clock;
  double master_clock;
  double frame_last_pts;
  double frame_last_delay;

  // ** Seek **
  int seek_requested;
  int seek_flags;
  int64_t seek_pos;
  AVPacket *flush_pkt;
  int seek_diff_count;

  // ** Threads **
  std::thread *decode_thread;
  std::thread *video_thread;
  std::thread *audio_thread;

  // ** Global Quit Flag **
  int global_quit;

  // ** Video Player Parameters **
  bool audio_enabled{true};
  // TODO: use frame inherent data instead
  uint64_t currentFrame{0};
};

#endif // AL_VIDEOREADER_HPP
