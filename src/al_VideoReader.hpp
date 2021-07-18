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

static const int AUDIO_BUFFER_SIZE = 8192 * 8;
static const int AUDIO_BUFFER_REFRESH_THRESHOLD = 8192;
static const int MAX_AUDIOQ_SIZE = (5 * 16 * 1024);
static const int MAX_VIDEOQ_SIZE = (5 * 256 * 1024);
static const int PICTQ_SIZE = 3;

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

  // get video parameters
  int width();
  int height();
  double fps();

  // get the next frame data
  uint8_t *getFrame();
  // notify frame has been rendered
  void gotFrame();
  // TODO: use inherent frame number
  uint16_t getCurrentFrameNumber() { return currentFrame; }

  // fill audio buffer
  void readAudioBuffer();

  // get the audio channel buffer
  SingleRWRingBuffer *getAudioBuffer(int channel) {
    return &(audio_buffer[channel]);
  }

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

  // inserts decoded frame into picture queue
  bool queue_picture(AVFrame *qFrame);

  // decode audio frame
  // returns size of decoded audio data if successful, negative on fail
  int audio_decode_frame();

  // functions to manage packet queue
  void packet_queue_init(PacketQueue *pq);
  void packet_queue_put(PacketQueue *pq, AVPacket *packet);
  // returns -1 if global quit flag is set, 0 if queue is empty, 1 if packet has
  // been extracted
  int packet_queue_get(PacketQueue *pq, AVPacket *packet, int blocking);

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

  // ** Video Stream **
  int video_st_idx;
  AVStream *video_st;
  AVCodecContext *video_ctx;
  PacketQueue videoq;
  struct SwsContext *sws_ctx;
  PictureQueue pictq;

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
