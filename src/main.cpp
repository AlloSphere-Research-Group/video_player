#include <iostream>
#include "cpptoml.h"

#include "al_VideoApp.hpp"

using namespace al;

// Helper function to list all keys in the TOML file for debugging
void listAllKeys(const std::shared_ptr<cpptoml::table> &root, const std::string &prefix = "") {
  if (!root) return;
  
  for (const auto &pair : *root) {
    std::string key = prefix.empty() ? pair.first : prefix + "." + pair.first;
    std::cout << "  Key: " << key;
    
    if (pair.second->is_value()) {
      std::cout << " (value)";
      // Try to get different value types using the table's get_as method
      try {
        auto boolVal = root->get_as<bool>(pair.first);
        if (boolVal) {
          std::cout << " = " << (*boolVal ? "true" : "false");
        } else {
          auto intVal = root->get_as<int64_t>(pair.first);
          if (intVal) {
            std::cout << " (int64) = " << *intVal;
          } else {
            auto doubleVal = root->get_as<double>(pair.first);
            if (doubleVal) {
              std::cout << " (double) = " << *doubleVal;
            } else {
              auto strVal = root->get_as<std::string>(pair.first);
              if (strVal) {
                std::cout << " (string) = \"" << *strVal << "\"";
              }
            }
          }
        }
      } catch (...) {
        std::cout << " (value, could not extract)";
      }
    } else if (pair.second->is_array()) {
      std::cout << " (array)";
    } else if (pair.second->is_table()) {
      std::cout << " (table)";
    } else if (pair.second->is_table_array()) {
      std::cout << " (table_array)";
    }
    std::cout << std::endl;
    
    // Recursively list keys in nested tables
    if (pair.second->is_table()) {
      listAllKeys(pair.second->as_table(), key);
    }
  }
}

bool loadSession(VideoApp &app, std::string sessionFile) {
  std::cout << "=== Loading session file ===" << std::endl;
  std::cout << "Session file path: " << sessionFile << std::endl;
  
  // Check if file exists before trying to load
  if (!al::File::exists(sessionFile)) {
    std::cerr << "ERROR: Session file does not exist: " << sessionFile << std::endl;
    return false;
  }
  std::cout << "Session file exists: YES" << std::endl;
  
  TomlLoader appConfig;
  try {
    appConfig.setFile(sessionFile);
    std::cout << "Session file parsed successfully" << std::endl;
  } catch (const std::exception &e) {
    std::cerr << "ERROR: Failed to parse session file: " << e.what() << std::endl;
    return false;
  }
  
  if (!appConfig.root) {
    std::cerr << "ERROR: TOML root is null after parsing" << std::endl;
    return false;
  }
  std::cout << "TOML root is valid: YES" << std::endl;
  std::cout << "All keys found in config file:" << std::endl;
  listAllKeys(appConfig.root);

  // Load video file
  if (appConfig.hasKey<std::string>("videoFile")) {
    std::string videoFile = appConfig.gets("videoFile");
    std::cout << "Found 'videoFile': " << videoFile << std::endl;
    app.setVideoFile(videoFile);
  } else {
    std::cout << "Key 'videoFile' not found in config" << std::endl;
  }

  Pose p;
  Vec3f s;
  
  // Load position
  if (appConfig.root->get_array_of<double>("position")) {
    auto xyz = appConfig.getVector<double>("position");
    std::cout << "Found 'position' array with " << xyz.size() << " elements" << std::endl;
    if (xyz.size() == 3) {
      p.pos(xyz[0], xyz[1], xyz[2]);
      std::cout << "  Position set to: (" << xyz[0] << ", " << xyz[1] << ", " << xyz[2] << ")" << std::endl;
    } else {
      std::cerr << "ERROR: position in session file has wrong size (expected 3, got " << xyz.size() << ")." << std::endl;
    }
  } else {
    std::cout << "Key 'position' not found or not an array" << std::endl;
  }

  // Load quaternion
  if (appConfig.root->get_array_of<double>("quaternion")) {
    auto quat = appConfig.getVector<double>("quaternion");
    std::cout << "Found 'quaternion' array with " << quat.size() << " elements" << std::endl;
    if (quat.size() == 4) {
      p.quat(Quatd(quat[0], quat[1], quat[2], quat[3]));
      std::cout << "  Quaternion set to: (" << quat[0] << ", " << quat[1] << ", " << quat[2] << ", " << quat[3] << ")" << std::endl;
    } else {
      std::cerr << "ERROR: quaternion in session file has wrong size (expected 4, got " << quat.size() << ")." << std::endl;
    }
  } else {
    std::cout << "Key 'quaternion' not found or not an array" << std::endl;
  }
  
  // Load scale
  if (appConfig.root->get_array_of<double>("scale")) {
    auto scale = appConfig.getVector<double>("scale");
    std::cout << "Found 'scale' array with " << scale.size() << " elements" << std::endl;
    if (scale.size() == 3) {
      s = Vec3f(scale[0], scale[1], scale[2]);
      std::cout << "  Scale set to: (" << scale[0] << ", " << scale[1] << ", " << scale[2] << ")" << std::endl;
    } else {
      std::cerr << "ERROR: scale in session file has wrong size (expected 3, got " << scale.size() << ")." << std::endl;
    }
  } else {
    std::cout << "Key 'scale' not found or not an array" << std::endl;
  }
  // Load windowed flag
  if (appConfig.hasKey<bool>("windowed")) {
    auto w = appConfig.root->get_as<bool>("windowed");
    std::cout << "Found 'windowed': " << (w && *w ? "true" : "false") << std::endl;
    if(w && *w) {
      app.setWindowed(p, s);
      std::cout << "  Windowed mode enabled" << std::endl;
    }
  } else {
    std::cout << "Key 'windowed' not found" << std::endl;
  }
  
  // Load stereo flag
  if (appConfig.hasKey<bool>("stereo")) {
    auto s = appConfig.root->get_as<bool>("stereo");
    std::cout << "Found 'stereo': " << (s && *s ? "true" : "false") << std::endl;
    if(s) {
      app.stereo = *s;
      std::cout << "  Stereo set to: " << (*s ? "true" : "false") << std::endl;
    }
  } else {
    std::cout << "Key 'stereo' not found" << std::endl;
  }
  
  // Load fullscreen flag
  if (appConfig.hasKey<bool>("fullscreen")) {
    auto fs = appConfig.root->get_as<bool>("fullscreen");
    std::cout << "Found 'fullscreen': " << (fs && *fs ? "true" : "false") << std::endl;
    if(fs && *fs) {
      app.fullscreen.set(1.0);
      std::cout << "  Fullscreen mode enabled" << std::endl;
    }
  } else {
    std::cout << "Key 'fullscreen' not found" << std::endl;
  }

  // Load global gain
  if (appConfig.hasKey<double>("globalGain")) {
    double gain = appConfig.getd("globalGain");
    std::cout << "Found 'globalGain': " << gain << std::endl;
    assert(app.audioDomain()->parameters()[0]->getName() == "gain");
    app.audioDomain()->parameters()[0]->fromFloat(gain);
    std::cout << "  Global gain set to: " << gain << std::endl;
  } else {
    std::cout << "Key 'globalGain' not found" << std::endl;
  }

  // Load audio for simulator only
  if (!sphere::isRendererMachine()) {
    std::cout << "Loading audio configuration (not renderer machine)" << std::endl;
    
    if (appConfig.hasKey<double>("audioDelay")) {
      double delay = appConfig.getd("audioDelay");
      std::cout << "Found 'audioDelay': " << delay << std::endl;
      app.setAudioDelay(delay);
    } else {
      std::cout << "Key 'audioDelay' not found" << std::endl;
    }
    
    auto nodesTable = appConfig.root->get_table_array("audioFile");
    std::vector<std::string> filesToLoad;
    if (nodesTable) {
      auto &tables = nodesTable->get();
      std::cout << "Found 'audioFile' table array with " << tables.size() << " entries" << std::endl;
      for (size_t i = 0; i < tables.size(); ++i) {
        const auto &table = tables[i];
        std::cout << "  Processing audioFile entry " << (i+1) << std::endl;
        
        if (!table->contains("name")) {
          std::cerr << "ERROR: audioFile entry " << (i+1) << " missing 'name' field" << std::endl;
          return false;
        }
        std::string name = *table->get_as<std::string>("name");
        std::cout << "    name: " << name << std::endl;
        
        if (!table->contains("outChannels")) {
          std::cerr << "ERROR: audioFile entry " << (i+1) << " missing 'outChannels' field" << std::endl;
          return false;
        }
        auto outChannelsToml = *table->get_array_of<int64_t>("outChannels");
        std::vector<size_t> outChannels;
        for (auto channel : outChannelsToml) {
          outChannels.push_back(channel);
        }
        std::cout << "    outChannels: [";
        for (size_t j = 0; j < outChannels.size(); ++j) {
          std::cout << outChannels[j];
          if (j < outChannels.size() - 1) std::cout << ", ";
        }
        std::cout << "]" << std::endl;
        
        float gain = 1.0f;
        if (table->contains("gain")) {
          gain = *table->get_as<double>("gain");
          std::cout << "    gain: " << gain << std::endl;
        }
        
        bool loop = false;
        if (table->contains("loop")) {
          loop = *table->get_as<bool>("loop");
          std::cout << "    loop: " << (loop ? "true" : "false") << std::endl;
        }
        
        // Load requested file into app. If any file fails, abort.
        std::cout << "    Loading audio file..." << std::endl;
        if (!app.loadAudioFile(name, outChannels, gain, loop)) {
          std::cerr << "ERROR: Failed to load audio file: " << name << std::endl;
          return false;
        }
        std::cout << "    Audio file loaded successfully: " << name << std::endl;
      }
    } else {
      std::cout << "No 'audioFile' table array found in config file - no audio configured" << std::endl;
    }
  } else {
    std::cout << "Skipping audio configuration (renderer machine)" << std::endl;
  }
  
  std::cout << "=== Session file loaded successfully ===" << std::endl;
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
        al::File::conformDirectory("C:/Users/Andres/Documents/Mangroves");
    //    app.dataRoot = "/Users/cannedstar/code/video_player/data/";
  }

  // Accept a video file or a 'session' file as command line argument
  std::string videoUrl;
  std::string sessionFile =
      al::File::conformDirectory(app.dataRoot) + "VideoPlayer.toml";
  
  std::cout << "=== Command line arguments ===" << std::endl;
  std::cout << "argc: " << argc << std::endl;
  for (int i = 0; i < argc; ++i) {
    std::cout << "  argv[" << i << "]: " << argv[i] << std::endl;
  }
  std::cout << "Data root: " << app.dataRoot << std::endl;
  std::cout << "Default session file: " << sessionFile << std::endl;
  
  if (argc > 1) {
    std::string fileName = argv[1];
    std::cout << "Processing argument: " << fileName << std::endl;
    if (fileName.substr(fileName.size() - 5) == ".toml") {
      sessionFile = fileName;
      app.dataRoot = al::File::directory(fileName);
      std::cout << "Detected TOML file, setting session file to: " << sessionFile << std::endl;
      std::cout << "Setting data root to: " << app.dataRoot << std::endl;
    } else {
      videoUrl = fileName;
      std::cout << "Detected video file, setting video URL to: " << videoUrl << std::endl;
      app.setVideoFile(videoUrl);
    }
  }

  if (videoUrl.size() == 0) {
    std::cout << "No video URL specified, loading session file..." << std::endl;
    if (!loadSession(app, sessionFile)) {
      std::cerr << "ERROR loading session: " << sessionFile << std::endl;
      return -1;
    }
  } else {
    std::cout << "Video URL specified, skipping session file load" << std::endl;
  }

  auto dev = AudioDevice::defaultOutput();
  if (!app.isPrimary() && app.omniRendering) {
    // Disable stereo
    // app.omniRendering->stereo(false);
    // app.displayMode(Window::DEFAULT_BUF);
  }

  app.start();
  return 0;
}
