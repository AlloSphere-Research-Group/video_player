#include "al_VideoApp.hpp"

#include "al/sphere/al_AlloSphereSpeakerLayout.hpp"
#include "al/sphere/al_SphereUtils.hpp"

using namespace al;

const std::string pano_vert = R"(
#version 330
uniform mat4 al_ModelViewMatrix;
uniform mat4 al_ProjectionMatrix;

layout (location = 0) in vec3 position;
layout (location = 2) in vec2 texcoord;

uniform float eye_sep;
uniform float foc_len;

out vec2 texcoord_;

vec4 stereo_displace(vec4 v, float e, float f) {
  // eye to vertex distance
  float l = sqrt((v.x - e) * (v.x - e) + v.y * v.y + v.z * v.z);
  // absolute z-direction distance
  float z = abs(v.z);
  // x coord of projection of vertex on focal plane when looked from eye
  float t = f * (v.x - e) / z;
  // x coord of displaced vertex to make displaced vertex be projected on focal plane
  // when looked from origin at the same point original vertex would be projected
  // when looked form eye
  v.x = z * (e + t) / f;
  // set distance from origin to displaced vertex same as eye to original vertex
  v.xyz = normalize(v.xyz);
  v.xyz *= l;
  return v;
}

void main() {
  if (eye_sep == 0) {
    gl_Position = al_ProjectionMatrix * al_ModelViewMatrix * vec4(position, 1.0);
  }
  else {
    gl_Position = al_ProjectionMatrix * stereo_displace(al_ModelViewMatrix * vec4(position, 1.0), eye_sep, foc_len);
  }

  texcoord_ = texcoord;
}
)";

const std::string pano_frag = R"(
#version 330
uniform sampler2D tex0;
uniform float exposure;

in vec2 texcoord_;
out vec4 frag_color;

mat4 exposureMat (float value) {
  return mat4(value, 0, 0, 0,
              0, value, 0, 0,
              0, 0, value, 0,
              0, 0, 0, 1);
}

// can apply filters here
void main() {
  frag_color = exposureMat(exposure) * texture(tex0, texcoord_);

  // frag_color = texture(tex0, texcoord_);
}
)";

VideoApp::VideoApp() {
  // remove simulation domain to replace it with simulation domain that runs
  // post onDraw()
  mOpenGLGraphicsDomain->removeSubDomain(simulationDomain());
  mSimulationDomain =
      mOpenGLGraphicsDomain->newSubDomain<StateDistributionDomain<SharedState>>(
          false);
}

void VideoApp::onInit() {
  if (hasCapability(CAP_OMNIRENDERING)) {
    omniRendering->drawOmni = false;
  }

  mPlaying = false;
  mExposure = 1.0f;
  audioIO().gain(1.0); // 0.4
}

void VideoApp::onCreate() {
  pano_shader.compile(pano_vert, pano_frag);

  pano_shader.begin();
  pano_shader.uniform("tex0", 0);
  pano_shader.uniform("exposure", mExposure);
  pano_shader.end();

  // url of video file
  // const char *url =
  //     "/Users/cannedstar/code/video_player/data/renate-barcelona-driving.mp4";
  // const char *url =
  //     "/Users/cannedstar/code/video_player/data/Iron_Man-Trailer_HD.mp4";
  // const char *url = "/Users/cannedstar/code/video_player/data/"
  //                   "3DH-Take1-Side-By-Side-4000x2000.mp4";
  // const char *url = "/Users/cannedstar/code/video_player/data/"
  //                   "unreal-village-omnistereo.mp4";
  const char *url = "/Users/cannedstar/code/video_player/data/"
                    "LastWhispers_040719_ambix_360.mp4";

  // TODO: not sure if this can be extracted from metadata
  mEquirectangular = true;

  // load video file
  audioDomain()->stop();
  if (!videoReader.load(url)) {
    std::cerr << "Error loading video file" << std::endl;
    quit();
  }

  if (!isPrimary()) {
    videoReader.enableAudio(false);
  }

  videoReader.start();

  // generate texture
  tex.filter(Texture::LINEAR);
  tex.wrap(Texture::REPEAT, Texture::CLAMP_TO_EDGE, Texture::CLAMP_TO_EDGE);
  tex.create2D(videoReader.width(), videoReader.height(), Texture::RGBA8,
               Texture::RGBA, Texture::UBYTE);

  // generate mesh
  quad.primitive(Mesh::TRIANGLE_STRIP);
  quad.vertex(-1, 1);
  quad.vertex(-1, -1);
  quad.vertex(1, 1);
  quad.vertex(1, -1);

  // Add texture coordinates
  quad.texCoord(0, 0);
  quad.texCoord(0, 1);
  quad.texCoord(1, 0);
  quad.texCoord(1, 1);

  // update vao mesh
  quad.update();

  // addSphereWithTexcoords(sphere, 5, 20);
  addSphereWithEquirectTex(sphere, 10, 50);
  sphere.update();

  // set fps
  if (isPrimary()) {
    fps(videoReader.fps());
  }

  mPlaying = true;

  configureAudio();

  // start audio
  audioDomain()->start();
}

void VideoApp::onAnimate(al_sec dt) {
  nav().pos().set(0);

  frameFinished = false;
  double av_delay = 0;

  if (isPrimary()) {
    // state().quat = nav().quat();

    if (mPlaying) {
      while (!frameFinished) {
        uint8_t *frame = videoReader.getFrame(av_delay);

        if (av_delay > 0) { // video needs to be delayed
          return;
        } else if (av_delay == 0) { // video is in sync with audio
          if (frame) {
            // returns immediately if nullptr is submitted
            tex.submit(frame);
            state().frameNum = videoReader.getCurrentFrameNumber();
            frameFinished = true;
          }
          return;
        }

        // video needs to catch up with audio
        if (frame) {
          videoReader.gotFrame();
        }
      }
    }
  } else {
    // nav().quat().set(state().quat);

    uint8_t *frame = nullptr;

    if (videoReader.getCurrentFrameNumber() > state().frameNum) {
      return;
    }

    while (state().frameNum > videoReader.getCurrentFrameNumber()) {
      frame = videoReader.getFrame(av_delay);
      if (frame) {
        videoReader.gotFrame();
      }
    }

    frame = videoReader.getFrame(av_delay);
    if (frame) {
      frameFinished = true;
    }

    tex.submit(frame);
  }
}

void VideoApp::onDraw(Graphics &g) {
  g.clear();

  g.shader(pano_shader);

  if (mUniformChanged) {
    g.shader().uniform("exposure", mExposure);
  }

  tex.bind();

  if (!mEquirectangular) {
    if (!hasCapability(CAP_OMNIRENDERING)) {
      g.viewport(0, 0, fbWidth(), fbHeight());
      g.camera(Viewpoint::IDENTITY);
      g.draw(quad);
    } else if (!omniRendering->drawOmni) {
      g.draw(sphere);
    }
  } else {
    g.draw(sphere);
  }

  if (frameFinished) {
    // need to be called to advance picture queue
    videoReader.gotFrame();
  }

  tex.unbind();
}

void VideoApp::onSound(AudioIOData &io) {
  if (isPrimary()) {
    if (mPlaying && videoReader.hasAudio()) {
      // ambisonics
      if (decodeAmbisonics) {
        float audioBuffer[4][8192];
        float *audioBufferScan[4];
        assert(io.framesPerBuffer() <= 8192);
        // Read video file audio buffers
        size_t channelBytesRead = 0;

        for (int inChan = 0; inChan < 4; inChan++) {
          SingleRWRingBuffer *audioRingBuffer =
              videoReader.getAudioBuffer(inChan);
          size_t bytesRead =
              audioRingBuffer->read((char *)audioBuffer[inChan],
                                    io.framesPerBuffer() * sizeof(float));

          // map from FuMa to ACN (should be determined from file metadata
          int ambiMap[] = {0, 2, 3, 1};
          audioBufferScan[inChan] = audioBuffer[ambiMap[inChan]];
          if (inChan > 0 && channelBytesRead != bytesRead) {
            std::cerr << "ERROR audio buffer size mismatch" << std::endl;
            channelBytesRead = std::min(channelBytesRead, bytesRead);
          } else {
            channelBytesRead = bytesRead;
          }
        }
        float *outbufs[64];
        assert(io.channelsOut() <= 64);
        for (int i = 0; i < io.channelsOut(); ++i) {
          outbufs[i] = io.outBuffer(i);
        }
        ambisonics.decode((float **)outbufs, (const float **)audioBufferScan,
                          channelBytesRead);
      } else { // no ambisonics
        float audioBuffer[8192];

        for (int i = 0; i < videoReader.audioNumChannels(); ++i) {
          SingleRWRingBuffer *audioRingBuffer = videoReader.getAudioBuffer(i);
          size_t bytesRead = audioRingBuffer->read(
              (char *)audioBuffer, io.framesPerBuffer() * sizeof(float));

          if (bytesRead > 0) {
            memcpy(io.outBuffer(i), audioBuffer, bytesRead);
          }
        }
      }

      // update current audio reference clock
      videoReader.updateAudioRef();
    }
  }
}

bool VideoApp::onKeyDown(const Keyboard &k) {
  if (k.key() == ' ') {
    mPlaying = true;
  } else if (k.key() == 'o') {
    if (hasCapability(CAP_OMNIRENDERING)) {
      omniRendering->drawOmni = !omniRendering->drawOmni;
    }
  } else if (k.key() == 'p') {
    mEquirectangular = !mEquirectangular;
  }
  return true;
}

int VideoApp::addSphereWithEquirectTex(Mesh &m, double radius, int bands) {
  m.primitive(Mesh::TRIANGLES);

  double &r = radius;

  for (int lat = 0; lat <= bands; ++lat) {
    double theta = lat * M_PI / bands;
    double sinTheta = sin(theta);
    double cosTheta = cos(theta);

    for (int lon = 0; lon <= bands; ++lon) {
      double phi = lon * M_2PI / bands;
      double sinPhi = sin(phi);
      double cosPhi = cos(phi);

      double x = sinPhi * sinTheta;
      double y = cosTheta;
      double z = cosPhi * sinTheta;

      // inversed longitude when looked from inside
      double u = 1.0 - (double)lon / bands;
      double v = (double)lat / bands;

      m.vertex(r * x, r * y, r * z);
      m.texCoord(u, v);
      // inversed normal
      m.normal(-x, -y, -z);
    }
  }

  for (int lat = 0; lat < bands; ++lat) {
    for (int lon = 0; lon < bands; ++lon) {
      int first = lat * (bands + 1) + lon;
      int second = first + bands + 1;

      m.index(first);
      m.index(first + 1);
      m.index(second);

      m.index(second);
      m.index(first + 1);
      m.index(second + 1);
    }
  }

  return m.vertices().size();
}

void VideoApp::configureAudio() {
  auto sl = AlloSphereSpeakerLayoutExtraThin();
  if (al::sphere::isSimulatorMachine()) {
    ambisonics.setSpeakers(sl);
  } else {
    ambisonics.setSpeakers(sl);
  }

  if (videoReader.hasAudio()) {
    audioDomain()->audioIO().framesPerSecond(videoReader.audioSampleRate());
    // audioDomain()->audioIO().channelsOut(videoReader.audioNumChannels());
    audioDomain()->audioIO().channelsOut(60);
    if (videoReader.audioNumChannels() == 4) {
      // TODO Determine this from metadata
      decodeAmbisonics = true;
    }
  }
}
