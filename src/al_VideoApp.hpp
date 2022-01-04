#ifndef AL_VIDEOAPP_HPP
#define AL_VIDEOAPP_HPP

#include "MTCReader.hpp"
#include "al/app/al_DistributedApp.hpp"
#include "al/sound/al_Ambisonics.hpp"
#include "al_VideoDecoder.hpp"

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
    mVideoFileToLoad = dataRoot + videoFileUrl;
  };

  double wallTime{0};

private:
  Texture tex;
  VAOMesh quad, sphere;
  bool mEquirectangular{false};

  ShaderProgram pano_shader;
  float mExposure;
  bool mUniformChanged{false};

  VideoDecoder videoDecoder;
  std::string mVideoFileToLoad;

  AmbiDecode ambisonics{3, 1, 2, 2};
  bool decodeAmbisonics{false};

  MTCReader mtcReader;

  bool mPlaying{true};
  bool mShowDiagnostic{false};

  ParameterBool syncToMTC{"syncToMTC"};
};

} // namespace al

#endif // AL_VIDEOAPP_HPP
