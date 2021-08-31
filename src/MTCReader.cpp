#include "MTCReader.hpp"

MTCReader::MTCReader() {
  mMtcReceiver.bindTo(midiIn);
  midiIn.ignoreTypes(false, false, false);
  std::vector<std::string> fps = {"24", "25", "29.97", "30"};

  TCframes.setElements(fps);
}

float MTCReader::fps() { return mFrameValues[TCframes.get()]; }

uint32_t MTCReader::frameNum() {
  return mMtcReceiver.hour * 60 * 60 * fps() +
         mMtcReceiver.minute * 60 * fps() + mMtcReceiver.second * fps() +
         mMtcReceiver.frame;
}
