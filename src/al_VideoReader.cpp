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

  void init() {
    pFormatCtx = nullptr;
    videoStream = -1;
    audioStream = -1;
    pCodec = nullptr;
    aCodec = nullptr;
    pCodecCtx = nullptr;
    aCodecCtx = nullptr;
    pFrame = nullptr;
    pFrameRGB = nullptr;
    aFrame = nullptr;
    buffer = nullptr;
    numBytes = 0;
    sws_ctx = nullptr;
    pPacket = nullptr;
    aPacket = nullptr;
  }

  bool load(const char *url) {
    // open file
    if (avformat_open_input(&pFormatCtx, url, NULL, NULL) < 0) {
      std::cerr << "Could not open file: " << url << std::endl;
      quit();
    }

    // retrieve stream information
    if (avformat_find_stream_info(pFormatCtx, NULL) < 0) {
      std::cerr << "Could not find stream info: " << url << std::endl;
      quit();
    }

    // print info
    av_dump_format(pFormatCtx, 0, url, 0);

    // find video stream
    for (int i = 0; i < pFormatCtx->nb_streams; ++i) {
      if (pFormatCtx->streams[i]->codecpar->codec_type == AVMEDIA_TYPE_VIDEO &&
          videoStream < 0) {
        videoStream = i;
      }
      if (pFormatCtx->streams[i]->codecpar->codec_type == AVMEDIA_TYPE_AUDIO &&
          audioStream < 0) {
        audioStream = i;
      }
    }

    if (videoStream == -1) {
      std::cerr << "Could not find video stream" << std::endl;
      quit();
    }
    if (audioStream == -1) {
      std::cerr << "Could not find audio stream" << std::endl;
      quit();
    }

    // find decoder for audio stream
    aCodec = avcodec_find_decoder(
        pFormatCtx->streams[audioStream]->codecpar->codec_id);
    if (!aCodec) {
      std::cerr << "Unsupported audio codec" << std::endl;
    }

    // find decoder for video stream
    pCodec = avcodec_find_decoder(
        pFormatCtx->streams[videoStream]->codecpar->codec_id);
    if (!pCodec) {
      std::cerr << "Unsupported video codec" << std::endl;
      quit();
    }

    // retrieve audio codec context
    aCodecCtx = avcodec_alloc_context3(aCodec);

    if (avcodec_parameters_to_context(
            aCodecCtx, pFormatCtx->streams[audioStream]->codecpar) != 0) {
      std::cerr << "Could not copy audio codec context" << std::endl;
      quit();
    }

    // open audio codec
    if (avcodec_open2(aCodecCtx, aCodec, NULL) < 0) {
      std::cerr << "Could not open audio codec" << std::endl;
      quit();
    }

    // allocate a new frame, used to decode audio packets
    aFrame = av_frame_alloc();
    if (!aFrame) {
      std::cerr << "Could not allocate audio frame" << std::endl;
      quit();
    }

    // allocate audio packet
    aPacket = av_packet_alloc();
    if (!aPacket) {
      std::cerr << "Could not allocate audio packet" << std::endl;
      quit();
    }

    packet_queue_init(&audioq);

    // retrieve video codec context
    pCodecCtx = avcodec_alloc_context3(pCodec);

    if (avcodec_parameters_to_context(
            pCodecCtx, pFormatCtx->streams[videoStream]->codecpar) != 0) {
      std::cerr << "Could not copy video codec context" << std::endl;
      quit();
    }

    // open video codec
    if (avcodec_open2(pCodecCtx, pCodec, NULL) < 0) {
      std::cerr << "Could not open video codec" << std::endl;
      quit();
    }

    // allocate space for video frame
    pFrame = av_frame_alloc();
    if (!pFrame) {
      std::cerr << "Could not allocate frame" << std::endl;
      quit();
    }

    // allocate space for converted frame
    pFrameRGB = av_frame_alloc();
    if (!pFrameRGB) {
      std::cerr << "Could not allocate frame" << std::endl;
      quit();
    }

    // allocate space to convert raw data
    numBytes = av_image_get_buffer_size(AV_PIX_FMT_RGBA, pCodecCtx->width,
                                        pCodecCtx->height, 32);
    buffer = (uint8_t *)av_malloc(numBytes * sizeof(uint8_t));

    // Setup the parameters for pFrameRGB
    av_image_fill_arrays(pFrameRGB->data, pFrameRGB->linesize, buffer,
                         AV_PIX_FMT_RGBA, pCodecCtx->width, pCodecCtx->height,
                         32);

    // allocate video packet
    pPacket = av_packet_alloc();
    if (!pPacket) {
      std::cerr << "Could not allocate packet" << std::endl;
      quit();
    }

    // initialize SWS context for software scaling
    sws_ctx =
        sws_getContext(pCodecCtx->width, pCodecCtx->height, pCodecCtx->pix_fmt,
                       pCodecCtx->width, pCodecCtx->height, AV_PIX_FMT_RGBA,
                       SWS_FAST_BILINEAR, NULL, NULL, NULL);

    // read first frame
    readFrame();

    // set fps
    fps(av_q2d(pFormatCtx->streams[videoStream]->r_frame_rate));
  }

  bool readFrame() {
    while (true) {
      // free the packet that was allocated by av_read_frame
      av_packet_unref(pPacket);

      // read the next frame
      if (av_read_frame(pFormatCtx, pPacket) < 0) {
        mPlaying = false;
        // no more frames. end playback
        return false;
      }

      // is this from the video stream?
      if (pPacket->stream_index == videoStream) {
        // send next packet for decoding
        if (avcodec_send_packet(pCodecCtx, pPacket) < 0) {
          std::cerr << "Error sending packet for decoding" << std::endl;
          quit();
        }

        // successfully got packet
        return true;
      } else if (pPacket->stream_index == audioStream) {
        packet_queue_put(&audioq, pPacket);
      }
    }
  }

  void onAnimate(al_sec dt) override {
    if (mPlaying) {
      // receive frame
      int ret = avcodec_receive_frame(pCodecCtx, pFrame);

      if (ret == AVERROR(EAGAIN)) {
        if (!readFrame()) {
          // no more frames
          return;
        }

        // receive new frame
        ret = avcodec_receive_frame(pCodecCtx, pFrame);
      }

      if (ret == AVERROR(EAGAIN)) {
        std::cerr << "Error receiving frame" << std::endl;
        quit();
      } else if (ret == AVERROR_EOF) {
        mPlaying = false;
        // end of file
        return;
      } else if (ret < 0) {
        std::cerr << "Error while decoding" << std::endl;
        quit();
      }

      // Convert the image from its native format to RGB
      sws_scale(sws_ctx, (uint8_t const *const *)pFrame->data, pFrame->linesize,
                0, pCodecCtx->height, pFrameRGB->data, pFrameRGB->linesize);

      // ***** check pFrameRGB data structure
      tex.submit(buffer);
    }
  }

  void onDraw(Graphics &g) override {
    if (mPlaying) {
      g.clear();
      g.viewport(0, 0, fbWidth(), fbHeight());
      g.camera(Viewpoint::IDENTITY);
      tex.bind();
      g.texture();
      g.draw(mesh);
      tex.unbind();
    }
  }

  void onSound(AudioIOData &io) override {
    if (mPlaying) {
      // should be in separate thread
      // check readspace < 8192 and notify_one()
      //
      {
        while (mAudioBuffer[0].writeSpace() > 8192) {
          if (shouldQuit()) {
            return;
          }

          // get more audio data
          if (audio_decode_frame(aCodecCtx) < 0) {
            std::cerr << "audio_decode_frame() failed" << std::endl;
          }
        }
      }

      float buffer[8192];

      for (int i = 0; i < io.channelsOut(); ++i) {
        size_t bytesRead = mAudioBuffer[i].read(
            (char *)buffer, io.framesPerBuffer() * sizeof(float));

        // *** notify_one here too

        if (bytesRead > 0) {
          memcpy(io.outBuffer(i), buffer, bytesRead);
        }
      }
    }
  }

  int audio_decode_frame(AVCodecContext *aCodecCtx) {
    int data_size = 0;

    // check global quit flag
    if (shouldQuit()) {
      return 0;
    }

    while (aPacket->data) {
      int ret = avcodec_receive_frame(aCodecCtx, aFrame);
      if (ret == AVERROR(EAGAIN)) {
        break;
      } else if (ret == AVERROR_EOF) {
        return 0;
      } else if (ret < 0) {
        std::cerr << "Error receiving audio frame" << std::endl;
        quit();
      }

      int sample_size = av_get_bytes_per_sample(aCodecCtx->sample_fmt);
      if (sample_size < 0) {
        std::cerr << "Failed to calculate data size" << std::endl;
        quit();
      } else if (sample_size * aFrame->nb_samples > 8192) {
        std::cerr << "Audio overrun" << std::endl;
      }

      for (int i = 0; i < aFrame->nb_samples; ++i) {
        for (int ch = 0; ch < aCodecCtx->channels; ++ch) {
          data_size += mAudioBuffer[ch].write(
              (const char *)aFrame->data[ch] + sample_size * i, sample_size);
        }
      }

      if (mAudioBuffer[0].writeSpace() < 8192) {
        return data_size;
      }
    }

    if (aPacket->data) {
      // wipe the packet
      av_packet_unref(aPacket);
    }

    // get new audio packet from queue
    if (packet_queue_get(&audioq, aPacket, 1) < 0) {
      return 0; // global quit
    }

    if (avcodec_send_packet(aCodecCtx, aPacket) < 0) {
      std::cerr << "Error sending audio packet for decoding" << std::endl;
      return -1;
    }

    return data_size;
  }

  void packet_queue_init(PacketQueue *pq) { pq->dataSize = 0; }

  void packet_queue_put(PacketQueue *pq, AVPacket *packet) {
    std::unique_lock<std::mutex> lk(pq->mutex);

    pq->queue.push(av_packet_clone(packet));
    pq->dataSize += packet->size;

    // notify packet_queue_get
    pq->cond.notify_one();
  }

  int packet_queue_get(PacketQueue *pq, AVPacket *packet, int block) {
    int ret;

    std::unique_lock<std::mutex> lk(pq->mutex);

    while (true) {
      if (shouldQuit()) {
        ret = -1;
        break;
      }

      if (!pq->queue.empty()) {
        *packet = *(pq->queue.front());
        pq->dataSize -= packet->size;

        pq->queue.pop();

        // successful packet retrieval
        ret = 1;
        break;
      } else if (!block) {
        // avoid waiting if block = 0
        ret = 0;
        break;
      } else {
        // wait until packet is available
        pq->cond.wait(lk);
      }
    }

    return ret;
  }

  // bool onKeyDown(const Keyboard &k) override;

  void onExit() override { cleanup(); }

  void cleanup() {
    av_packet_unref(pPacket);
    av_packet_free(&pPacket);
    av_packet_unref(aPacket);
    av_packet_free(&aPacket);

    // Free the RGB image
    av_free(buffer);
    av_frame_free(&pFrameRGB);
    av_free(pFrameRGB);

    // Free the video frame
    av_frame_free(&pFrame);
    av_free(pFrame);

    // Free the audio frame
    av_frame_free(&aFrame);
    av_free(aFrame);

    // Close the codecs
    avcodec_close(pCodecCtx);
    avcodec_close(aCodecCtx);

    // Close the video file
    avformat_close_input(&pFormatCtx);
  }

private:
  AVFormatContext *pFormatCtx;

  int videoStream;
  int audioStream;

  const AVCodec *pCodec;
  const AVCodec *aCodec;

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

  std::thread *decode_thread;

  SingleRWRingBuffer mAudioBuffer[2] = {{8192 * 8}, {8192 * 8}};
};

#endif // AL_VIDEOREADER_HPP