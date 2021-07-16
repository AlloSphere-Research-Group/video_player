#ifndef AL_VIDEOAPP_HPP
#define AL_VIDEOAPP_HPP

#include "al/app/al_DistributedApp.hpp"
#include "al_VideoReader.hpp"

typedef struct {
  int frameNum;
} SharedState;

namespace al {

class VideoApp : public DistributedAppWithState<SharedState> {
public:
  VideoApp();

  virtual ~VideoApp() {}

  virtual void onInit() override;
  virtual void onCreate() override;
  virtual void onAnimate(al_sec dt) override;
  virtual void onDraw(Graphics &gl) override;
  virtual void onSound(AudioIOData &io) override;
  // virtual bool onKeyDown(const Keyboard &k) override;
  virtual void onExit() override { videoReader.stop(); }

private:
  Texture tex;
  VAOMesh quad;

  VideoReader videoReader;

  bool mPlaying{false};
};

} // namespace al

#endif // AL_VIDEOAPP_HPP
