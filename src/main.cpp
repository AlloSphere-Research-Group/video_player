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
    // videoUrl = "/data/media/pano_videos/renate-barcelona-driving.mp4";
    // videoUrl = "/data/media/pano_videos/Iron_Man-Trailer_HD.mp4";
    // videoUrl =
    // "/data/media/pano_videos/3DH-Take1-Side-By-Side-4000x2000.mp4";
    // videoUrl = "/data/media/pano_videos/unreal-village-omnistereo.mp4";
    // videoUrl =
    //  "/data/media/pano_videos/LastWhispers_040719_ambix_360.mp4";

    // videoUrl = "c:/Users/Andres/Downloads/"
    //            "LastWhispers_040719_ambix_360.mp4";
    videoUrl = "/Users/cannedstar/code/video_player/data/"
               "LastWhispers_040719_ambix_360.mp4";
    // videoUrl =
    //     "/Users/cannedstar/code/video_player/data/Iron_Man-Trailer_HD.mp4";
    // videoUrl = "/Users/cannedstar/code/video_player/data/test.mov";
  }

  app.setVideoFile(videoUrl);
  app.start();
  return 0;
}
