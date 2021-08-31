#ifndef AL_VIDEOAPP_HPP
#define AL_VIDEOAPP_HPP

#include "al/app/al_DistributedApp.hpp"
#include "al/sound/al_Ambisonics.hpp"
#include "al_VideoReader.hpp"

typedef struct {
  int frameNum = 0;
  bool diagnostics{false};
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
  virtual bool onKeyDown(const Keyboard &k) override;
  virtual void onExit() override { videoReader.stop(); }

  int addSphereWithEquirectTex(Mesh &m, double radius, int bands);
  void configureAudio();

  void setVideoFile(std::string videoFileUrl) {
    mVideoFileToLoad = videoFileUrl;
  };

private:
  Texture tex;
  VAOMesh quad, sphere;
  bool mEquirectangular{false};

  ShaderProgram pano_shader;
  float mExposure;
  bool mUniformChanged{false};

  VideoReader videoReader;
  bool frameFinished{false};

  AmbiDecode ambisonics{3, 1, 2, 2};
  bool decodeAmbisonics{false};

  bool mPlaying{false};

  std::string mVideoFileToLoad;
};

} // namespace al

#endif // AL_VIDEOAPP_HPP
