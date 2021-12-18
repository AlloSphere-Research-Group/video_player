#ifndef AL_MEDIABUFFER_HPP
#define AL_MEDIABUFFER_HPP

// https://github.com/Golim4r/OpenGL-Video-Player

#include <atomic>
#include <condition_variable>
#include <mutex>
#include <vector>

struct MediaFrame {
  MediaFrame() {}
  MediaFrame(uint8_t *data_ptr, size_t data_size, double data_pts)
      : data(data_ptr, data_ptr + data_size), pts(data_pts) {}

  void clear() {
    data.clear();
    pts = 0;
  }

  std::vector<uint8_t> data;
  double pts;
};

class MediaBuffer {
public:
  MediaBuffer(int numElements)
      : frames(numElements), valid(numElements), readPos(0), writePos(0) {}

  bool put(MediaFrame &&mFrame) {
    if (valid[writePos]) {
      std::cerr << "Buffer pos already occupied: " << writePos << std::endl;
      return false;
    }

    std::cout << "Writing at buffer: " << writePos << std::endl;

    // possible unnecessary memory copy
    frames[writePos] = std::move(mFrame);
    valid[writePos] = true;
    writePos = ++writePos % frames.size();

    return true;
  }

  bool get(MediaFrame &mFrame) {
    if (!valid[readPos]) {
      std::cerr << "Buffer pos empty: " << readPos << std::endl;
      return false;
    }

    std::cout << "Reading at buffer: " << readPos << std::endl;

    // possible unnecessary memory copy
    mFrame = std::move(frames[readPos]);
    valid[readPos] = false;
    readPos = ++readPos % frames.size();

    return true;
  }

  void flush() {
    for (int i = 0; i < frames.size(); ++i) {
      valid[i] = false;
      frames[i].clear();
    }

    readPos = 0;
    writePos = 0;
  }

  int getReadPos() { return readPos.load(); }

  int getWritePos() { return writePos.load(); }

private:
  std::vector<MediaFrame> frames;
  std::vector<std::atomic<bool>> valid;

  std::atomic<int> readPos;
  std::atomic<int> writePos;

  // std::mutex mutex;
  // std::condition_variable cond;
};

#endif