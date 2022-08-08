#ifndef AL_VIDEOAPP_HPP
#define AL_VIDEOAPP_HPP

#include "MTCReader.hpp"
#include "al/app/al_DistributedApp.hpp"
#include "al_ext/video/al_VideoDecoder.hpp"

typedef struct SharedState {
  double global_clock;
} SharedState;

struct MappedAudioFile {
  std::unique_ptr<SoundFileBuffered> soundfile;
  std::vector<size_t> outChannelMap;
  std::string fileInfoText;
  std::string fileName;
  float gain;
  bool mute{false};
};

namespace al {

class VideoApp : public DistributedAppWithState<SharedState> {
public:
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

  bool loadAudioFile(std::string name, std::vector<size_t> channelMap,
                     float gain, bool loop);
  double wallTime{0};

  void setVideoFile(std::string videoFileUrl) {
    videoFileToLoad = dataRoot + videoFileUrl;
  };

  void setWindowed(Pose pose, Vec3f scale);

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

  std::vector<MappedAudioFile> soundfiles;
  uint64_t samplesPlayed{0};

  ParameterBool syncToMTC{"syncToMTC"};
  ParameterBool renderVideo{"renderVideo", "", 1.0};

  ParameterBool windowed{"windowed", "", 0.0};
  ParameterPose renderPose{"renderPose", "", Pose(Vec3d(0, 0, -4))};
  ParameterVec3 renderScale{"renderScale", "", Vec3f(1, 1, 1)};
};

} // namespace al

#endif // AL_VIDEOAPP_HPP
