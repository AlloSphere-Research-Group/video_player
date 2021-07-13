#ifndef VIDEOPLAYBACK_HPP
#define VIDEOPLAYBACK_HPP

#include "al/app/al_DistributedApp.hpp"

#include "al_VideoPlayer.hpp"

typedef struct {
  int frameNum;
} SharedState;

namespace al {

class MyApp : public DistributedAppWithState<SharedState> {
public:
  MyApp() { mPlaying = false; }

  virtual ~MyApp() {}

  virtual void onCreate() override;
  virtual void onDraw(Graphics &gl) override;
  virtual void onAnimate(al_sec dt) override;
  virtual void onSound(AudioIOData &io) override;
  virtual void onMessage(osc::Message &m) override;
  virtual bool onKeyDown(const Keyboard &k) override;

private:
  //	VideoTexturer mVideoTexture {"/data/andres/PlieguesALTh264.mov"};
  //    VideoTexturer mVideoTexture {"/data/andres/Pliegues1024.mp4"};
  //    VideoTexture mVideoTexture {"/home/andres/Probably-3mins-final.mp4"};
  VideoTexture mVideoTexture{"/home/andres/Documents/07 Proyectos y "
                             "trabajos/06 Musica/geminiflux/CCT1.mp4"};
  //    VideoTexturer mVideoTexture
  //    {"/media/andres/LaCie_Mac_Only/_AlloArchive/AlloSphere
  //    Archive_032714/1-New-UK-KucheraMorin.mov", true}; VideoTexturer
  //    mVideoTexture {"/media/andres/LACIE SHARE/brain-test.mp4",
  //    VideoFileReader::SYNC_AUDIO}; VideoTexturer mVideoTexture
  //    {"/home/andres/Videos/Webcam/2014-12-08-210226.webm"};
  bool mSideBySide = true;
  Mesh mQuadL, mQuadR;
  bool mPlaying;
};

} // namespace al

#endif // VIDEOPLAYBACK_HPP
