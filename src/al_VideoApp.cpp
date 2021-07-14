#include "al_VideoApp.hpp"

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

void VideoApp::onInit() override { videoReader.init(); }

void VideoApp::onCreate() override {
  // url of video file
  // const char *url =
  //     "/Users/cannedstar/code/video_player/data/renate-barcelona-driving.mp4";
  const char *url =
      "/Users/cannedstar/code/video_player/data/Iron_Man-Trailer_HD.mp4";
  // const char *url = "/Users/cannedstar/code/video_player/data/"
  //                   "3DH-Take1-Side-By-Side-4000x2000.mp4";
  // const char *url = "/Users/cannedstar/code/video_player/data/"
  //                   "unreal-village-omnistereo.mp4";

  // load video file
  videoReader.load(url);

  // configure audio IO
  audioDomain()->stop();
  audioDomain()->audioIO().framesPerSecond(videoReader.audioSampleRate());
  audioDomain()->audioIO().channelsOut(videoReader.audioNumChannels());

  // start audio
  audioDomain()->start();

  // generate texture
  tex.filter(Texture::LINEAR);
  tex.wrap(Texture::REPEAT, Texture::CLAMP_TO_EDGE, Texture::CLAMP_TO_EDGE);
  tex.create2D(videoReader.width(), videoReader.height(), Texture::RGBA8,
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

  // update vao mesh
  mesh.update();

  // read first frame
  readFrame();

  // set fps
  fps(videoReader.fps());
}

void VideoApp::onAnimate(al_sec dt) override {
  if (mPlaying) {
    tex.submit(videoReader.getFrame());
  }
}

void VideoApp::onDraw(Graphics &g) override {
  if (mPlaying) {
    g.clear();
    g.viewport(0, 0, fbWidth(), fbHeight());
    g.camera(Viewpoint::IDENTITY);
    tex.bind();
    g.texture();
    g.draw(quad);
    tex.unbind();
  }
}

void VideoApp::onSound(AudioIOData &io) override {
  if (mPlaying) {
    float audioBuffer[8192];

    for (int i = 0; i < io.channelsOut(); ++i) {
      SingleRWRingBuffer &audioRingBuffer = videoReader.getAudioBuffer(i);
      size_t bytesRead = audioRingBuffer.read(
          (char *)audioBuffer, io.framesPerBuffer() * sizeof(float));

      // *** notify_one here too

      if (bytesRead > 0) {
        memcpy(io.outBuffer(i), audioBuffer, bytesRead);
      }
    }
  }
}

// bool VideoApp::onKeyDown(const Keyboard &k) override;

void VideoApp::onExit() override { videoReader.quit(); }