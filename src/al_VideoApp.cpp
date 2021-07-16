#include "al_VideoApp.hpp"

#include "al/graphics/al_Font.hpp"

using namespace al;

VideoApp::VideoApp() {
  mPlaying = false;

  // remove simulation domain to replace it with simulation domain that runs
  // post onDraw()
  mOpenGLGraphicsDomain->removeSubDomain(simulationDomain());
  mSimulationDomain =
      mOpenGLGraphicsDomain->newSubDomain<StateDistributionDomain<SharedState>>(
          false);
}

void VideoApp::onInit() {}

void VideoApp::onCreate() {
  // url of video file
  // const char *url =
  //     "/Users/cannedstar/code/video_player/data/renate-barcelona-driving.mp4";
  const char *url =
      "/Users/cannedstar/code/video_player/data/Iron_Man-Trailer_HD.mp4";
  // const char *url = "/Users/cannedstar/code/video_player/data/"
  //                   "3DH-Take1-Side-By-Side-4000x2000.mp4";
  // const char *url = "/Users/cannedstar/code/video_player/data/"
  //                   "unreal-village-omnistereo.mp4";
  // const char *url = "/Users/cannedstar/code/video_player/data/"
  //                   "LastWhispers_040719_ambix_360.mp4";

  // load video file
  audioDomain()->stop();
  if (!videoReader.load(url)) {
    std::cerr << "Error loading video file" << std::endl;
    quit();
  }

  // generate texture
  tex.filter(Texture::LINEAR);
  tex.wrap(Texture::REPEAT, Texture::CLAMP_TO_EDGE, Texture::CLAMP_TO_EDGE);
  tex.create2D(videoReader.width(), videoReader.height(), Texture::RGBA8,
               Texture::RGBA, Texture::UBYTE);

  // generate mesh
  quad.primitive(Mesh::TRIANGLE_STRIP);
  quad.vertex(-1, 1);
  quad.vertex(-1, -1);
  quad.vertex(1, 1);
  quad.vertex(1, -1);

  // Add texture coordinates
  quad.texCoord(0, 0);
  quad.texCoord(0, 1);
  quad.texCoord(1, 0);
  quad.texCoord(1, 1);

  // update vao mesh
  quad.update();

  // set fps
  if (isPrimary()) {
    fps(videoReader.fps());
  }

  //  mPlaying = true;

  // start audio
  audioDomain()->audioIO().framesPerSecond(videoReader.audioSampleRate());
  audioDomain()->audioIO().channelsOut(videoReader.audioNumChannels());
  audioDomain()->start();
}

void VideoApp::onAnimate(al_sec dt) {

  if (isPrimary()) {
    if (mPlaying) {
      auto *frame = videoReader.getFrame();
      if (frame) {
        tex.submit(frame);
      }
      state().frameNum = videoReader.getCurrentFrameNumber();
    }
  } else {
    //    auto *frame = videoReader.getFrame(state().frameNum);
    uint8_t *frame = nullptr;
    while (state().frameNum > videoReader.getCurrentFrameNumber()) {
      frame = videoReader.getFrame();
    }
    if (frame) {
      tex.submit(frame);
    }
  }
}

void VideoApp::onDraw(Graphics &g) {
  if (isPrimary()) {
    if (mPlaying) {
      g.clear();
      g.viewport(0, 0, fbWidth(), fbHeight());
      g.camera(Viewpoint::IDENTITY);
      tex.bind();
      g.texture();
      g.draw(quad);
      tex.unbind();
    }
  } else {
    // Renderer
    g.clear();
    g.pushMatrix();
    // Dummy rendering on quad while we map the sphere
    g.translate(0, 0, -4);
    g.scale(5);
    tex.bind();
    g.texture();
    g.draw(quad);
    tex.unbind();
    g.popMatrix();

    //    FontRenderer::render(g, std::to_string(state().frameNum).c_str(),
    //                         {-1, 1, -3});
  }
}

void VideoApp::onSound(AudioIOData &io) {
  if (mPlaying) {
    videoReader.readAudioBuffer();

    float audioBuffer[8192];
    //    if (isPrimary()) {

    for (int i = 0; i < io.channelsOut(); ++i) {
      SingleRWRingBuffer *audioRingBuffer = videoReader.getAudioBuffer(i);
      size_t bytesRead = audioRingBuffer->read(
          (char *)audioBuffer, io.framesPerBuffer() * sizeof(float));

      // *** notify_one here too

      if (bytesRead > 0) {
        memcpy(io.outBuffer(i), audioBuffer, bytesRead);
      }
    }
    //    } else {

    //    }
  }
}

bool VideoApp::onKeyDown(const Keyboard &k) {
  if (k.key() == ' ') {
    mPlaying = true;
  }
  return true;
}

// bool VideoApp::onKeyDown(const Keyboard &k) {}
