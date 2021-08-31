#ifndef MTCREADER_HPP
#define MTCREADER_HPP

#include "al/app/al_App.hpp"
#include "al/io/al_File.hpp"
#include "al/io/al_Imgui.hpp"
#include "al/io/al_Toml.hpp"
#include "al/sound/al_SpeakerAdjustment.hpp"
#include "al/sphere/al_AlloSphereSpeakerLayout.hpp"
#include "al/sphere/al_SphereUtils.hpp"
#include "al/ui/al_FileSelector.hpp"
#include "al/ui/al_ParameterGUI.hpp"
#include "al_ext/soundfile/al_SoundfileBuffered.hpp"

// From: https://github.com/hideakitai/MTCParser
// Under MIT license
//    Copyright (c) 2018 Hideaki Tai

// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.

//   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
#include "MTCParser.h"
#include <mutex>

using namespace al;

class MTCReceiver : public MIDIMessageHandler {
public:
  MTCParser mtc;
  uint8_t hour{0};
  uint8_t minute{0};
  uint8_t second{0};
  uint8_t frame{0};

  std::mutex mLock;
  virtual ~MTCReceiver() {}

  /// Called when a MIDI message is received
  virtual void onMIDIMessage(const MIDIMessage &m) {
    if (m.type() == MIDIByte::SYSTEM_MSG && m.status() == MIDIByte::TIME_CODE) {
      assert(m.dataSize() <= UINT8_MAX);
      mtc.feed(m.bytes, (uint8_t)m.dataSize());
      if (mtc.available()) {
        mLock.lock();
        hour = mtc.hour();
        minute = mtc.minute();
        second = mtc.second();
        frame = mtc.frame();
        mLock.unlock();
        mtc.pop();
      }
    }
  };
};

class MTCReader {
public:
  ParameterMenu TCframes{"TC_fps"};
  RtMidiIn midiIn;
  // TODO implement frame offset
  Parameter frameOffset{"frame_offset", "", 0, -25, 25};

  MTCReader();

  float fps();
  uint32_t frameNum();
  void getMTC(uint8_t &hour, uint8_t &minute, uint8_t &second, uint8_t &frame) {
    mMtcReceiver.mLock.lock();
    hour = mMtcReceiver.hour;
    minute = mMtcReceiver.minute;
    second = mMtcReceiver.second;
    frame = mMtcReceiver.frame;
    mMtcReceiver.mLock.unlock();
  }

protected:
  MTCReceiver mMtcReceiver;
  std::vector<float> mFrameValues = {24, 25, 30, 30};
};

#endif // MTCREADER_HPP
