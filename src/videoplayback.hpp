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
  virtual void onAnimate(al_sec dt) override;
  virtual void onDraw(Graphics &gl) override;
  virtual void onSound(AudioIOData &io) override;
  virtual void onMessage(osc::Message &m) override;
  virtual bool onKeyDown(const Keyboard &k) override;

private:
  VideoTexture mVideoTexture;
  bool mSideBySide = false;
  Mesh mQuadL, mQuadR;
  bool mPlaying;
};

} // namespace al

#endif // VIDEOPLAYBACK_HPP
