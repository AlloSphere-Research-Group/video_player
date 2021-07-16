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

#include "al/types/al_SingleRWRingBuffer.hpp"

using namespace al;

// queue structure to store AVPackets
typedef struct PacketQueue {
  std::queue<AVPacket *> queue;
  int dataSize;
  std::mutex mutex;
  std::condition_variable cond;
} PacketQueue;

// queue structure to store video frames

class VideoReader {
public:
  VideoReader();

  ~VideoReader() {} // cleanup here?

  void init();

  bool load(const char *url);

  bool stream_component_open(int stream_index);

  int width() { return video_width; }

  int height() { return video_height; }

  double fps() { return video_fps; }

  int audioSampleRate() { return audio_sampleRate; }

  int audioNumChannels() { return audio_channels; }

  bool readFrame();

  uint8_t *getFrame(uint64_t frameNum = UINT64_MAX);
  uint16_t getCurrentFrameNumber() { return currentFrame; }

  void readAudioBuffer();

  SingleRWRingBuffer *getAudioBuffer(int i) { return &(audio_buffer[i]); }

  int audio_decode_frame();

  void packet_queue_init(PacketQueue *pq);

  void packet_queue_put(PacketQueue *pq, AVPacket *packet);

  int packet_queue_get(PacketQueue *pq, AVPacket *packet, int block);

  void close() { cleanup(); }

  void cleanup();

private:
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

  int audio_sampleRate;
  int audio_channels;

  // ** Video Stream **
  int video_st_idx;
  AVStream *video_st;
  AVCodecContext *video_ctx;
  PacketQueue videoq;
  struct SwsContext *sws_ctx;

  // add picture queue stuff

  int video_width;
  int video_height;
  double video_fps;

  // ** Threads **
  std::thread *decode_thread;
  std::thread *video_thread;

  // ** Input file name **
  char filename[1024] = {0};

  // ** Global Quit Flag **
  int quit;

  // todo: remove
  uint64_t currentFrame{0};
  AVFrame *pFrame;
  AVFrame *pFrameRGB;

  uint8_t *buffer;
  int numBytes;
  AVPacket *pPacket;
};

#endif // AL_VIDEOREADER_HPP
