#include "al_VideoApp.hpp"

#include "al/graphics/al_Font.hpp"

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
  mPlaying = false;

  // remove simulation domain to replace it with simulation domain that runs
  // post onDraw()
  mOpenGLGraphicsDomain->removeSubDomain(simulationDomain());
  mSimulationDomain =
      mOpenGLGraphicsDomain->newSubDomain<StateDistributionDomain<SharedState>>(
          false);
}

void VideoApp::onInit() {
  omniRendering->drawOmni = false;

  mExposure = 1.0f;
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
  const char *url =
      "/Users/cannedstar/code/video_player/data/Iron_Man-Trailer_HD.mp4";
  // const char *url = "/Users/cannedstar/code/video_player/data/"
  //                   "3DH-Take1-Side-By-Side-4000x2000.mp4";
  // const char *url = "/Users/cannedstar/code/video_player/data/"
  //                   "unreal-village-omnistereo.mp4";
  // const char *url = "/Users/cannedstar/code/video_player/data/"
  //                   "LastWhispers_040719_ambix_360.mp4";

  // mEquirectangular = true;

  // load video file
  audioDomain()->stop();
  if (!videoReader.load(url)) {
    std::cerr << "Error loading video file" << std::endl;
    quit();
  }

  if (!hasCapability(CAP_AUDIO_IO)) {
    videoReader.disableAudio();
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
  fps(videoReader.fps());

  mPlaying = true;

  // start audio
  if (videoReader.hasAudio()) {
    audioDomain()->audioIO().framesPerSecond(videoReader.audioSampleRate());
    audioDomain()->audioIO().channelsOut(videoReader.audioNumChannels());
  }

  audioDomain()->start();
}

void VideoApp::onAnimate(al_sec dt) {
  nav().pos().set(0);

  if (mPlaying) {
    tex.submit(videoReader.getFrame());
    state().frameNum = videoReader.getCurrentFrameNumber();
    videoReader.gotFrame();
  }
}

void VideoApp::onDraw(Graphics &g) {
  if (isPrimary()) {
    if (mPlaying) {
      g.clear();

      g.shader(pano_shader);

      if (mUniformChanged) {
        g.shader().uniform("exposure", mExposure);
      }

      tex.bind();

      if (!mEquirectangular && !omniRendering->drawOmni) {
        g.viewport(0, 0, fbWidth(), fbHeight());
        g.camera(Viewpoint::IDENTITY);
        g.draw(quad);
      } else {
        g.draw(sphere);
      }

      tex.unbind();
    }
  } else {
    // Renderer
    g.clear();
    FontRenderer::render(g, std::to_string(state().frameNum).c_str(),
                         {-1, 1, -3});
  }
}

void VideoApp::onSound(AudioIOData &io) {
  if (mPlaying && videoReader.hasAudio()) {
    videoReader.readAudioBuffer();

    float audioBuffer[8192];

    for (int i = 0; i < io.channelsOut(); ++i) {
      SingleRWRingBuffer *audioRingBuffer = videoReader.getAudioBuffer(i);
      size_t bytesRead = audioRingBuffer->read(
          (char *)audioBuffer, io.framesPerBuffer() * sizeof(float));

      // *** notify_one here too

      if (bytesRead > 0) {
        memcpy(io.outBuffer(i), audioBuffer, bytesRead);
      }
    }
  }
}

bool VideoApp::onKeyDown(const Keyboard &k) {
  if (k.key() == 'o') {
    omniRendering->drawOmni = !omniRendering->drawOmni;
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
