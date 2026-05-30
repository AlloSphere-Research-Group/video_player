#!/bin/bash

(
  mkdir -p build
  cd build
  mkdir -p release
  cd release
  cmake -DCMAKE_BUILD_TYPE=Release -DRTAUDIO_API_JACK=OFF -DRTMIDI_API_JACK=OFF -Wno-deprecated -DBUILD_EXAMPLES=0 ../..
)

# Configure debug build
# (
#   mkdir -p build
#   cd build
#   mkdir -p debug
#   cd debug
#   cmake -DCMAKE_BUILD_TYPE=Debug -Wno-deprecated -DBUILD_EXAMPLES=0 ../..
# )

