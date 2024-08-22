extern "C" {
#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
#include <libavutil/imgutils.h>
#include <libswscale/swscale.h>
}

#include "al/app/al_App.hpp"
#include <iostream>

using namespace al;

const std::string yuv_vert = R"(
#version 330
uniform mat4 al_ModelViewMatrix;
uniform mat4 al_ProjectionMatrix;

layout (location = 0) in vec3 position;
layout (location = 2) in vec2 texcoord;

uniform float eye_sep;
uniform float foc_len;

out vec2 texcoord_;

vec4 stereo_displace(vec4 v, float e, float f) {
  // eye to vertex distance
  float l = sqrt((v.x - e) * (v.x - e) + v.y * v.y + v.z * v.z);
  // absolute z-direction distance
  float z = abs(v.z);
  // x coord of projection of vertex on focal plane when looked from eye
  float t = f * (v.x - e) / z;
  // x coord of displaced vertex to make displaced vertex be projected on focal plane
  // when looked from origin at the same point original vertex would be projected
  // when looked form eye
  v.x = z * (e + t) / f;
  // set distance from origin to displaced vertex same as eye to original vertex
  v.xyz = normalize(v.xyz);
  v.xyz *= l;
  return v;
}

void main() {
  if (eye_sep == 0) {
    gl_Position = al_ProjectionMatrix * al_ModelViewMatrix * vec4(position, 1.0);
  }
  else {
    gl_Position = al_ProjectionMatrix * stereo_displace(al_ModelViewMatrix * vec4(position, 1.0), eye_sep, foc_len);
  }

  texcoord_ = texcoord;
}
)";

const std::string yuv_frag = R"(
#version 330
uniform sampler2D texY;
uniform sampler2D texU;
uniform sampler2D texV;

in vec2 texcoord_;
out vec4 frag_color;

// can apply filters here
void main() {
  vec3 yuv;
  yuv.r = texture(texY, texcoord_).r - 0.0625;
  yuv.g = texture(texU, texcoord_).r - 0.5;
  yuv.b = texture(texV, texcoord_).r - 0.5;

  vec4 rgba;
  rgba.r = yuv.r + 1.596 * yuv.b;
  rgba.g = yuv.r - 0.813 * yuv.b - 0.391 * yuv.g;
  rgba.b = yuv.r + 2.018 * yuv.g;
  rgba.a = 1.0;
  frag_color = rgba;
}
)";

constexpr bool useRescale{false};

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
    //     "C:/Users/kenny/code/video_player/data/renate-barcelona-driving.mp4";
    // const char *url =
    //     "C:/Users/kenny/code/video_player/data/Iron_Man-Trailer_HD.mp4";
    // const char *url = "C:/Users/kenny/code/video_player/data/"
    //                   "3DH-Take1-Side-By-Side-4000x2000.mp4";
    // const char *url = "C:/Users/kenny/code/video_player/data/"
    //                   "unreal-village-omnistereo.mp4";
    const char *url =
        "C:/Users/kenny/code/video_player/data/LW_2K_ERP_noaudio.mp4";

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
    std::cout << "numBytes: " << numBytes << std::endl;
    buffer = (uint8_t *)av_malloc(numBytes * sizeof(uint8_t));

    int line_sizes[4];
    av_image_fill_linesizes(line_sizes, AV_PIX_FMT_YUV420P, pCodecCtx->width);

    numBytesY = line_sizes[0] * pCodecCtx->height;
    numBytesU = line_sizes[1] * pCodecCtx->height / 2;
    numBytesV = line_sizes[2] * pCodecCtx->height / 2;

    std::cout << "line_sizes[0]:" << line_sizes[0] << std::endl;
    std::cout << "line_sizes[1]:" << line_sizes[1] << std::endl;
    std::cout << "line_sizes[2]:" << line_sizes[2] << std::endl;

    std::cout << "numBytesY: " << numBytesY << std::endl;
    std::cout << "numBytesU: " << numBytesU << std::endl;
    std::cout << "numBytesV: " << numBytesV << std::endl;

    bufferY = (uint8_t *)av_malloc(numBytesY * sizeof(uint8_t));
    bufferU = (uint8_t *)av_malloc(numBytesU * sizeof(uint8_t));
    bufferV = (uint8_t *)av_malloc(numBytesV * sizeof(uint8_t));

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

    // TODO: test GL_UNSIGNED_INT_8_8_8_8_REV

    texY.filter(Texture::LINEAR);
    texY.wrap(Texture::REPEAT, Texture::CLAMP_TO_EDGE, Texture::CLAMP_TO_EDGE);
    texY.create2D(line_sizes[0], pCodecCtx->height, Texture::RED, Texture::RED,
                  Texture::UBYTE);
    texU.filter(Texture::LINEAR);
    texU.wrap(Texture::REPEAT, Texture::CLAMP_TO_EDGE, Texture::CLAMP_TO_EDGE);
    texU.create2D(line_sizes[1], pCodecCtx->height / 2, Texture::RED,
                  Texture::RED, Texture::UBYTE);
    texV.filter(Texture::LINEAR);
    texV.wrap(Texture::REPEAT, Texture::CLAMP_TO_EDGE, Texture::CLAMP_TO_EDGE);
    texV.create2D(line_sizes[2], pCodecCtx->height / 2, Texture::RED,
                  Texture::RED, Texture::UBYTE);

    yuv_shader.compile(yuv_vert, yuv_frag);

    yuv_shader.begin();
    yuv_shader.uniform("texY", 0);
    yuv_shader.uniform("texU", 1);
    yuv_shader.uniform("texV", 2);
    yuv_shader.end();

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
      frameCount++;
      if (frameCount == 100) {
        // mPlaying = false;
        decode_time *= 0.01;
        std::cout << "Decode time: " << decode_time << std::endl;
        frameCount = 0;
        decode_time = 0;
      }

      al_sec decode_start = al_system_time();
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
        return; // quit();
      } else if (ret == AVERROR_EOF) {
        mPlaying = false;
        // end of file
        return;
      } else if (ret < 0) {
        std::cerr << "Error while decoding" << std::endl;
        quit();
      }

      if (useRescale) {
        // Convert the image from its native format to RGB
        sws_scale(sws_ctx, (uint8_t const *const *)pFrame->data,
                  pFrame->linesize, 0, pCodecCtx->height, pFrameRGB->data,
                  pFrameRGB->linesize);
        tex.submit(buffer);
      } else {
        // memcpy(bufferY, pFrame->data[0], numBytesY);
        // memcpy(bufferU, pFrame->data[1], numBytesU);
        // memcpy(bufferV, pFrame->data[2], numBytesV);
        // texY.submit(bufferY);
        // texU.submit(bufferU);
        // texV.submit(bufferV);

        // std::cout << "pFrame->linesize: " << pFrame->linesize[0] <<
        // std::endl;

        texY.submit(pFrame->data[0]);
        texU.submit(pFrame->data[1]);
        texV.submit(pFrame->data[2]);
      }

      decode_time += al_system_time() - decode_start;
    }
  }

  void onDraw(Graphics &g) override {
    if (mPlaying) {
      if (useRescale) {
        g.clear();
        g.viewport(0, 0, fbWidth(), fbHeight());
        g.camera(Viewpoint::IDENTITY);
        tex.bind();
        g.texture();
        g.draw(mesh);
        tex.unbind();
      } else {
        g.shader(yuv_shader);
        g.clear();
        g.viewport(0, 0, fbWidth(), fbHeight());
        g.camera(Viewpoint::IDENTITY);
        texY.bind(0);
        texU.bind(1);
        texV.bind(2);
        g.draw(mesh);
        texY.unbind(0);
        texU.unbind(1);
        texV.unbind(2);
      }
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
    av_free(bufferY);
    av_free(bufferU);
    av_free(bufferV);
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
  Texture texY, texU, texV, tex;
  VAOMesh mesh;
  ShaderProgram yuv_shader;

  AVFormatContext *pFormatCtx;
  AVCodecContext *pCodecCtx;
  int videoStream;

  const AVCodec *pCodec;

  AVFrame *pFrame;
  AVFrame *pFrameRGB;

  uint8_t *bufferY, *bufferU, *bufferV, *buffer;
  int numBytesY, numBytesU, numBytesV, numBytes;

  int frameCount{0};
  double decode_time{0};

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