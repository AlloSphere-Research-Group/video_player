#include <iostream>

#include "al_VideoApp.hpp"

using namespace al;

int main(int argc, char *argv[]) {
  VideoApp app;
  app.title("Video Player");
  app.dimensions(600, 400);

  // Set data root
  if (sphere::isSimulatorMachine()) {
    app.dataRoot = File::conformDirectory("/Volumes/Data/media/Last Whispers");
  } else if (sphere::isRendererMachine()) {
    app.dataRoot = File::conformDirectory("/data/media/");
  } else {
    // Change this to your local data root path
    app.dataRoot = "c:/Users/Andres/Documents/Last Whispers/Media/";
    //    app.dataRoot = "/Users/cannedstar/code/video_player/data/";
  }

  std::string videoUrl;
  if (argc > 1) {
    videoUrl = argv[1];
    app.setVideoFile(videoUrl);
  } else {
    TomlLoader appConfig("Video Player.toml");

    if (appConfig.hasKey<std::string>("videoFile")) {
      app.setVideoFile(appConfig.gets("videoFile"));
    }

    if (appConfig.hasKey<double>("globalGain")) {
      assert(app.audioDomain()->parameters()[0]->getName() == "gain");
      app.audioDomain()->parameters()[0]->fromFloat(
          appConfig.getd("globalGain"));
    }    
    if (appConfig.hasKey<double>("audioDelay")) {
      app.setAudioDelay(appConfig.getd("audioDelay"));
    }

    // Load audio for simulator only
    if (!sphere::isRendererMachine()) {
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
std::cout << name << "   " << outChannels[0] << " "  << gain << std::endl;
          // Load requested file into app. If any file fails, abort.
          if (!app.loadAudioFile(name, outChannels, gain, loop)) {
            return -1;
          }
        }
      } else {
        return -1;
      }
    }
  }

  auto dev = AudioDevice::defaultOutput();
  app.audioDomain()->configure(dev, 48000, 512, 2, 0);

  app.start();
  return 0;
}
