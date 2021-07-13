
#include "al/system/al_Time.hpp"

#include "videoplayback.hpp"

using namespace al;

MyApp::MyApp() {
  mPlaying = false;

  // remove simulation domain to replace it with simulation domain that runs
  // post onDraw()
  mOpenGLGraphicsDomain->removeSubDomain(simulationDomain());
  mSimulationDomain =
      mOpenGLGraphicsDomain->newSubDomain<StateDistributionDomain<SharedState>>(
          false);
}

void MyApp::onCreate() {
  //	auto& s = shader();
  //    s.begin();
  //    s.uniform("texture", 1.0);
  //    s.uniform("lighting", 0.1);
  //    s.end();
  mVideoTexture.init(
      "C:/Users/Andres/Documents/Zoom/2021-04-28 19.26.43 Andres Cabrera "
      "Perez's Zoom Meeting 85001728190/zoom_0.mp4");
  // Set sampling rate and channel count from video
  audioDomain()->stop();
  audioDomain()->audioIO().framesPerSecond(mVideoTexture.audioSampleRate());
  audioDomain()->audioIO().channelsOut(mVideoTexture.audioNumChannels());
  audioDomain()->start();

  //  mVideoTexture.print();
  mVideoTexture.setPlayMode(VideoFileReader::PLAY_ONESHOT);

  if (mSideBySide) {
    // TODO fix side by side
    //    mQuadL.reset();
    //    mQuadL.primitive(Mesh::TRIANGLES);
    //    mQuadL.vertex(-1, -1, 0);
    //    mQuadL.texCoord(0, 1);
    //    mQuadL.vertex(1, -1, 0);
    //    mQuadL.texCoord(0.5, 1);
    //    mQuadL.vertex(1, 1, 0);
    //    mQuadL.texCoord(0.5, 0);
    //    mQuadL.vertex(-1, 1, 0);
    //    mQuadL.texCoord(0, 0);

    //    mQuadR.reset();
    //    mQuadR.primitive(Mesh::TRIANGLES);
    //    mQuadR.vertex(-1, -1, 0);
    //    mQuadR.texCoord(0.5, 1);
    //    mQuadR.vertex(1, -1, 0);
    //    mQuadR.texCoord(1, 1);
    //    mQuadR.vertex(1, 1, 0);
    //    mQuadR.texCoord(1, 0);
    //    mQuadR.vertex(-1, 1, 0);
    //    mQuadR.texCoord(0.5, 0);
  } else {

    mQuadL.reset();

    mQuadL.primitive(Mesh::TRIANGLE_STRIP);
    mQuadL.vertex(-1, 1);
    mQuadL.vertex(-1, -1);
    mQuadL.vertex(1, 1);
    mQuadL.vertex(1, -1);

    // Add texture coordinates
    mQuadL.texCoord(0, 0);
    mQuadL.texCoord(0, 1);
    mQuadL.texCoord(1, 0);
    mQuadL.texCoord(1, 1);
  }

  state().frameNum = 0;
}

void MyApp::onAnimate(al_sec dt) {
  if (isPrimary()) {
    //    std::cout << dt << std::endl;
    //	mTaker.get(mState);
    static int frame = 0;

    if (frame == 5) {
      //        osc::Send sender(9090, "localhost");
      //        sender.send("/play", 1.0f);

      mVideoTexture.start();
      mPlaying = true;
    }
    frame++;
    //    if (frame == 176) {
    //    }
    if (mPlaying) {
      mVideoTexture.readFrame();
      state().frameNum++;

      //	mMaker.set(state());
    }
    //    std::cout << (int) frame - mVideoTexture.currentFrame()<< std::endl;

  } else {
    mVideoTexture.readFrame(state().frameNum);
  }
}

void MyApp::onDraw(Graphics &g) {
  g.clear();
  if (isPrimary()) {
    g.texture();
    //    g.pushMatrix();
    //    g.translate(0, 0.1, -3);
    //    g.draw(mQuadL);
    //    g.popMatrix();
    mVideoTexture.bind();
    mVideoTexture.readFrame();
    if (mSideBySide) {
      g.pushMatrix();
      g.translate(1, -0.1, -2);
      g.draw(mQuadR);
      g.popMatrix();
    } else {
      g.pushMatrix();

      g.translate(0, 0, -4);
      g.scale(mVideoTexture.width() / (float)mVideoTexture.height(), 1, 1);
      g.draw(mQuadL);
      g.popMatrix();
    }
    mVideoTexture.unbind();
    //    std::cout << state().frameNum - mVideoTexture.currentFrame() <<
    //    std::endl;
  }
}

void MyApp::onSound(AudioIOData &io) {
  if (isPrimary()) {
    if (!mVideoTexture.initialized()) {
      return;
    }
    float buffer[8192];
    int numFileChnls = 2;
    for (int i = 0; i < numFileChnls; i++) {
      size_t elementsRead =
          mVideoTexture.readAudio(i, buffer, io.framesPerBuffer());
      if (elementsRead > 0) {
        memcpy(io.outBuffer(i), buffer, elementsRead * sizeof(float));
      }
      //        if (elementsRead < io.framesPerBuffer()) {
      //            std::cout << "buffer underrun! read " << elementsRead <<
      //            std::endl;
      //        }
    }
  }
}

void MyApp::onMessage(osc::Message &m) {}

bool MyApp::onKeyDown(const Keyboard &k) {
  if (k.key() == 32) {
    mPlaying = true;
    osc::Send sender(9090, "localhost");
    sender.send("/play", 1.0f);
  } else if (k.keyAsNumber() == 0) {
    std::cout << "play again" << std::endl;
    mVideoTexture.seek(0);
    mVideoTexture.start();
  }
  return true;
}
