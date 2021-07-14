#include <iostream>

#include "al_VideoApp.hpp"

using namespace al;

int main() {
  VideoApp app;
  app.title("Video Player");
  app.dimensions(600, 400);
  app.start();
  return 0;
}