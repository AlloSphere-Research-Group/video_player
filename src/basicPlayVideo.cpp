extern "C" {
#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
#include <libavutil/imgutils.h>
#include <libswscale/swscale.h>
}

#include "al/app/al_App.hpp"
#include <iostream>

using namespace al;

class VideoApp : public App {
public:
  VideoApp() {
    mPlaying = true;
    needFrame = true;
    // mSideBySide = false;
  }

  ~VideoApp() {}

  void onInit() override {
    pFormatCtx = nullptr;
    pCodecCtx = nullptr;
    videoStream = -1;
    pCodec = nullptr;
    pFrame = nullptr;
    pFrameRGB = nullptr;
    buffer = nullptr;
    numBytes = 0;
    sws_ctx = nullptr;
    pPacket = nullptr;
  }

  void onCreate() override {
    nav().pos(0, 0, 4); // change later

    // url of video file
    // const char *url =
    //     "/Users/cannedstar/code/video_player/data/renate-barcelona-driving.mp4";
    const char *url =
        "/Users/cannedstar/code/video_player/data/Iron_Man-Trailer_HD.mp4";
    // const char *url = "/Users/cannedstar/code/video_player/data/"
    //                   "3DH-Take1-Side-By-Side-4000x2000.mp4";
    // const char *url = "/Users/cannedstar/code/video_player/data/"
    //                   "unreal-village-omnistereo.mp4";

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
      if (pFormatCtx->streams[i]->codecpar->codec_type == AVMEDIA_TYPE_VIDEO) {
        videoStream = i;
        break;
      }
    }

    if (videoStream == -1) {
      std::cerr << "Could not find video stream" << std::endl;
      quit();
    }

    // find decoder for stream
    pCodec = avcodec_find_decoder(
        pFormatCtx->streams[videoStream]->codecpar->codec_id);

    if (!pCodec) {
      std::cerr << "Unsupported codec" << std::endl;
      quit();
    }

    // copy context
    pCodecCtx = avcodec_alloc_context3(pCodec);

    if (avcodec_parameters_to_context(
            pCodecCtx, pFormatCtx->streams[videoStream]->codecpar) < 0) {
      std::cerr << "Could not copy codec context" << std::endl;
      quit();
    }

    // open codec
    if (avcodec_open2(pCodecCtx, pCodec, NULL) < 0) {
      std::cerr << "Could not open codec" << std::endl;
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

    // allocate packet
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

    // generate texture
    tex.filter(Texture::LINEAR);
    tex.wrap(Texture::REPEAT, Texture::CLAMP_TO_EDGE, Texture::CLAMP_TO_EDGE);
    tex.create2D(pCodecCtx->width, pCodecCtx->height, Texture::RGBA8,
                 Texture::RGBA, Texture::UBYTE);

    // generate mesh
    mesh.primitive(Mesh::TRIANGLE_STRIP);
    mesh.vertex(-1, 1);
    mesh.vertex(-1, -1);
    mesh.vertex(1, 1);
    mesh.vertex(1, -1);

    // Add texture coordinates
    mesh.texCoord(0, 0);
    mesh.texCoord(0, 1);
    mesh.texCoord(1, 0);
    mesh.texCoord(1, 1);
    mesh.update();

    // read first frame
    readFrame();
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
    while (io()) {
      float out0 = 0;
      float out1 = 0;

      io.out(0) = out0;
      io.out(1) = out1;
    }
  }
  // bool onKeyDown(const Keyboard &k) override;

  void onExit() override { cleanup(); }

  void cleanup() {
    // Free the RGB image
    av_free(buffer);
    av_frame_free(&pFrameRGB);
    av_free(pFrameRGB);

    // Free the YUV frame
    av_frame_free(&pFrame);
    av_free(pFrame);

    // Close the codecs
    avcodec_close(pCodecCtx);

    // Close the video file
    avformat_close_input(&pFormatCtx);
  }

private:
  // VideoTexture mVideoTexture;
  // VAOMesh mQuadL, mQuadR;
  bool mPlaying;
  bool needFrame;
  Texture tex;
  VAOMesh mesh;

  AVFormatContext *pFormatCtx;
  AVCodecContext *pCodecCtx;
  int videoStream;

  const AVCodec *pCodec;

  AVFrame *pFrame;
  AVFrame *pFrameRGB;

  uint8_t *buffer;
  int numBytes;

  struct SwsContext *sws_ctx;
  AVPacket *pPacket;
};

int main(int argc, char *argv[]) {
  VideoApp app;
  app.dimensions(600, 400);
  app.title("Video Player");
  // app.fps(40);
  // app.audioDomain()->configure(44100, 128, 2, 1);  // rate, block, output,
  // input
  app.start();
}