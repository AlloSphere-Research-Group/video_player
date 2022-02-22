#ifndef AL_VIDEOAPP_HPP
#define AL_VIDEOAPP_HPP

#include "MTCReader.hpp"
#include "al/app/al_DistributedApp.hpp"
#include "al_ext/video/al_VideoDecoder.hpp"

typedef struct SharedState {
  double global_clock;
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
  virtual void onExit() override;

  int addSphereWithEquirectTex(Mesh &m, double radius, int bands);

  void configureAudio();

  void setVideoFile(std::string videoFileUrl) {
    videoFileToLoad = dataRoot + videoFileUrl;
  };

  double previousClock{0};

private:
  Texture tex;
  VAOMesh quad, sphere;
  bool equirectangular{false};

  ShaderProgram pano_shader;
  float exposure;
  bool uniformChanged{false};

  VideoDecoder videoDecoder;
  std::string videoFileToLoad;

  MTCReader mtcReader;

  bool playing{true};
  bool showHUD{false};

  ParameterBool syncToMTC{"syncToMTC"};
  ParameterBool renderVideo{"renderVideo", "", 1.0};
};

} // namespace al

#endif // AL_VIDEOAPP_HPP
