#include <iostream>

#include "al_VideoApp.hpp"

using namespace al;

int main(int argc, char *argv[]) {
  VideoApp app;
  app.title("Video Player");
  app.dimensions(600, 400);

  std::string videoUrl;
  if (argc > 1) {
    videoUrl = argv[1];
  } else {
    // url of video file
    // videoUrl = "renate-barcelona-driving.mp4";
    // videoUrl = "3DH-Take1-Side-By-Side-4000x2000.mp4";
    // videoUrl = "unreal-village-omnistereo.mp4";
    // videoUrl = "LastWhispers_040719_ambix_360.mp4";
    // videoUrl = "LW_KT_Edit_1205_360-convert.mp4";
    // videoUrl = "Lw Kt Edit 0103 Good 75Mbps 8K 360-4k-30fps-noaudio.m4v";
    // videoUrl = "Iron_Man-Trailer_HD.mp4";
    videoUrl = "LW_2K_ERP_noaudio.mp4";

    if (sphere::isSimulatorMachine()) {
      app.dataRoot += "/Volumes/Data/media/";
    } else if (sphere::isRendererMachine()) {
      app.dataRoot += "/data/media/";
    } else {
      // CHange this to your local data root path
      // app.dataRoot = "c:/Users/Andres/Downloads/";
      app.dataRoot = "/Users/cannedstar/code/video_player/data/";
    }
  }

  app.setVideoFile(videoUrl);
  app.start();
  return 0;
}
