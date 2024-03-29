#include <iostream>

#include "al/app/al_App.hpp"
#include "al/graphics/al_Shapes.hpp"

#include "videoplayback.hpp"

using namespace al;

int main() {
  MyApp app;
  app.fps(25);
  app.dimensions(600, 400);
  app.start();
  return 0;
}
