#include <iostream>

#include "al_VideoApp.hpp"

using namespace al;

bool loadSession(VideoApp &app, std::string sessionFile) {
  std::cout << "Using video session: " << sessionFile << std::endl;
  TomlLoader appConfig(sessionFile);

  if (appConfig.hasKey<std::string>("videoFile")) {
    app.setVideoFile(appConfig.gets("videoFile"));
  }

  Pose p;
  Vec3f s;
  if (appConfig.root->get_array_of<double>("position")) {
    auto xyz = appConfig.getVector<double>("position");
    if (xyz.size() == 3) {
      p.pos(xyz[0], xyz[1], xyz[2]);
    } else {
      std::cerr << "ERROR: position in session file has wrong size."
                << std::endl;
    }
  }

  if (appConfig.root->get_array_of<double>("quaternion")) {
    auto quat = appConfig.getVector<double>("quaternion");
    if (quat.size() == 4) {
      p.quat(Quatd(quat[0], quat[1], quat[2], quat[3]));
    } else {
      std::cerr << "ERROR: position, quaternion or scale in session file has "
                   "wrong size."
                << std::endl;
    }
  }
  if (appConfig.root->get_array_of<double>("scale")) {
    auto scale = appConfig.getVector<double>("scale");
    if (scale.size() == 3) {
      s = Vec3f(scale[0], scale[1], scale[2]);
    } else {
      std::cerr << "ERROR: scale in session file has wrong size." << std::endl;
    }
  }
  if (appConfig.hasKey<bool>("window")) {
    app.setWindowed(p, s);
  }

  if (appConfig.hasKey<double>("globalGain")) {
    assert(app.audioDomain()->parameters()[0]->getName() == "gain");
    app.audioDomain()->parameters()[0]->fromFloat(appConfig.getd("globalGain"));
  }

  // Load audio for simulator only
  if (!sphere::isRendererMachine()) {
    if (appConfig.hasKey<double>("audioDelay")) {
      app.setAudioDelay(appConfig.getd("audioDelay"));
    }
    auto nodesTable = appConfig.root->get_table_array("audioFile");
    std::vector<std::string> filesToLoad;
    if (nodesTable) {
      for (const auto &table : *nodesTable) {
        std::string name = *table->get_as<std::string>("name");
        auto outChannelsToml = *table->get_array_of<int64_t>("outChannels");
        std::vector<size_t> outChannels;
        float gain = 1.0f;
        bool loop = false;
        if (table->contains("gain")) {
          gain = *table->get_as<double>("gain");
        }
        if (table->contains("loop")) {
          loop = *table->get_as<bool>("loop");
        }
        for (auto channel : outChannelsToml) {
          outChannels.push_back(channel);
        }
        // Load requested file into app. If any file fails, abort.
        if (!app.loadAudioFile(name, outChannels, gain, loop)) {
          return false;
        }
        std::cout << "Audio: " << name << std::endl;
      }
    } else {
      return false;
    }
  }
  return true;
}

int main(int argc, char *argv[]) {
  VideoApp app;
  app.title("Video Player");
  app.dimensions(600, 400);

  // Set data root
  if (sphere::isSimulatorMachine()) {
    app.dataRoot = File::conformDirectory("/Volumes/Data/media/LastWhispers");
  } else if (sphere::isRendererMachine()) {
    app.dataRoot = File::conformDirectory("/data/media/LastWhispers");
  } else {
    // Change this to your local data root path
    app.dataRoot =
        al::File::conformDirectory("c:/Users/Andres/Documents/Mangroves");
    //    app.dataRoot = "/Users/cannedstar/code/video_player/data/";
  }

  // Accept a video file or a 'session' file as command line argument
  std::string videoUrl;
  std::string sessionFile =
      al::File::conformDirectory(app.dataRoot) + "VideoPlayer.toml";
  if (argc > 1) {
    std::string fileName = argv[1];
    if (fileName.substr(fileName.size() - 5) == ".toml") {
      sessionFile = fileName;
      app.dataRoot = al::File::directory(fileName);
      std::cout << "Setting data root to " << app.dataRoot << std::endl;
    } else {
      videoUrl = fileName;
      app.setVideoFile(videoUrl);
    }
  }

  if (videoUrl.size() == 0) {
    if (!loadSession(app, sessionFile)) {
      std::cerr << " ERROR loading session: " << sessionFile << std::endl;
      return -1;
    }
  }

  auto dev = AudioDevice::defaultOutput();
  if (!app.isPrimary() && app.omniRendering) {
    // Disable stereo
    app.omniRendering->stereo(false);
  }

  app.start();
  return 0;
}
