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

class VideoReader {
public:
  VideoReader();

  ~VideoReader() {} // cleanup here?

  void init();

  bool load(const char *url);

  bool stream_component_open(int stream_index);

  int width() { return video_codec_width; }

  int height() { return video_codec_height; }

  double fps() { return r_fps; }

  int audioSampleRate() { return audio_sample_rate; }

  int audioNumChannels() { return audio_channels; }

  bool readFrame();

  uint8_t *getFrame();

  void readAudioBuffer();

  SingleRWRingBuffer *getAudioBuffer(int i) { return &(mAudioBuffer[i]); }

  int audio_decode_frame(AVCodecContext *aCodecCtx);

  void packet_queue_init(PacketQueue *pq);

  void packet_queue_put(PacketQueue *pq, AVPacket *packet);

  int packet_queue_get(PacketQueue *pq, AVPacket *packet, int block);

  void close() { cleanup(); }

  void cleanup();

private:
  AVFormatContext *pFormatCtx;

  int video_codec_width;
  int video_codec_height;
  double r_fps;
  int audio_sample_rate;
  int audio_channels;

  int videoStream;
  int audioStream;

  AVCodecContext *pCodecCtx;
  AVCodecContext *aCodecCtx;

  AVFrame *pFrame;
  AVFrame *pFrameRGB;
  AVFrame *aFrame;

  uint8_t *buffer;
  int numBytes;

  struct SwsContext *sws_ctx;
  AVPacket *pPacket;
  AVPacket *aPacket;

  PacketQueue videoq;
  PacketQueue audioq;

  SingleRWRingBuffer mAudioBuffer[2] = {{8192 * 8}, {8192 * 8}};

  int quit;

  std::thread *decode_thread;
};

#endif // AL_VIDEOREADER_HPP
