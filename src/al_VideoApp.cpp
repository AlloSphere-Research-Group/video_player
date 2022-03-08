#include "al_VideoApp.hpp"

#include "al/graphics/al_Font.hpp"
#include "al/sphere/al_AlloSphereSpeakerLayout.hpp"
#include "al/sphere/al_SphereUtils.hpp"
#include "al_ext/statedistribution/al_CuttleboneStateSimulationDomain.hpp"

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
}

void VideoApp::onInit() {
  exposure = 1.0f;
  audioIO().gain(1.0); // 0.4
  CuttleboneStateSimulationDomain<SharedState>::enableCuttlebone(this);
}

void VideoApp::onCreate() {
  // compile & initialize shader
  pano_shader.compile(pano_vert, pano_frag);

  pano_shader.begin();
  pano_shader.uniform("tex0", 0);
  pano_shader.uniform("exposure", exposure);
  pano_shader.end();

  // load video file
  audioDomain()->stop();
  // TODO: temporarily disabled audio
  // if (!isPrimary()) {
  videoDecoder.enableAudio(false);
  // }

  // if (isPrimary()) {
  // videoDecoder.setMasterSync(MasterSync::AV_SYNC_AUDIO);
  // }

  // TODO: check audio startup
  if (!videoDecoder.load(videoFileToLoad.c_str())) {
    std::cerr << "Error loading video file" << std::endl;
    quit();
  }

  // // TODO: this can probably be extracted from video file metadata
  equirectangular = true;

  videoDecoder.start();

  // generate texture
  tex.filter(Texture::LINEAR);
  tex.wrap(Texture::REPEAT, Texture::CLAMP_TO_EDGE, Texture::CLAMP_TO_EDGE);
  tex.create2D(videoDecoder.width(), videoDecoder.height(), Texture::RGBA8,
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

  // TODO: review high fps option with manual timing control
  // set fps
  if (isPrimary()) {
    fps(videoDecoder.fps());
    mtcReader.TCframes.setCurrent("30");
    state().global_clock = 0;
  }

  configureAudio();

  // start audio
  audioDomain()->start();

  // start GUI
  if (hasCapability(Capability::CAP_2DGUI)) {
    imguiInit();
  }
}

void VideoApp::onAnimate(al_sec dt) {
  nav().pos().set(0);

  if (isPrimary()) {
    uint8_t hour, minute, second, frame;

    if (syncToMTC.get() == 1.0) {
      // TODO: review drift + handle offset (won't start at 0)
      mtcReader.getMTC(hour, minute, second, frame);

      state().global_clock = ((int32_t)hour * 360) + ((int32_t)minute * 60) +
                             second + (frame / mtcReader.fps()) +
                             mtcReader.frameOffset;
    } else if (playing) {
      state().global_clock += dt;
    }

    if (hasCapability(Capability::CAP_2DGUI)) {
      imguiBeginFrame();

      ImGui::Begin("MIDI Time Code");

      ParameterGUI::draw(&renderVideo);
      ParameterGUI::draw(&syncToMTC);
      ParameterGUI::drawMIDIIn(&mtcReader.midiIn);
      ParameterGUI::draw(&mtcReader.TCframes);
      ParameterGUI::draw(&mtcReader.frameOffset);

      ImGui::Text("%02i:%02i:%02i:%02i", hour, minute, second,
                  (int)(frame / mtcReader.fps() * 100));

      ImGui::End();
      imguiEndFrame();
    }
  }

  if ((renderVideo.get() == 1.0) && playing) {
    uint8_t *frame = videoDecoder.getVideoFrame(state().global_clock);

    if (frame) {
      tex.submit(frame);
      videoDecoder.gotVideoFrame();
    }
  }
}

void VideoApp::onDraw(Graphics &g) {
  g.clear();

  if (isPrimary()) {
    if (renderVideo.get() == 1.0) {
      g.viewport(0, 0, fbWidth(), fbHeight());
      g.pushCamera(Viewpoint::IDENTITY);
      tex.bind();
      g.texture();
      g.draw(quad);
      tex.unbind();
      g.popCamera();
    }
  } else {
    // Renderer
    g.shader(pano_shader);

    // TODO: add exposure control
    if (uniformChanged) {
      g.shader().uniform("exposure", exposure);
      uniformChanged = false;
    }

    tex.bind();
    g.draw(sphere);
    tex.unbind();
  }

  if (showHUD) {
    FontRenderer::render(
        g, ("Clock: " + std::to_string(state().global_clock)).c_str(),
        {-0.7, 0.45, -2}, 0.05);

    FontRenderer::render(
        g,
        ("FPS: " + std::to_string(1.0 / simulationDomain()->timeDelta()))
            .c_str(),
        {-0.7, 0.4, -2}, 0.05);

    // FontRenderer::render(
    //     g, ("Last PTS: " + std::to_string(videoDecoder.getPTS())).c_str(),
    //     {-0.7, 0.4, -2}, 0.05);

    // FontRenderer::render(
    //     g,
    //     ("audioq: " + std::to_string(videoDecoder.audioq_size()) + "/" +
    //      std::to_string(MAX_AUDIOQ_SIZE))
    //         .c_str(),
    //     {-0.7, 0.35, -2}, 0.05);

    // FontRenderer::render(
    //     g,
    //     ("videoq: " + std::to_string(videoDecoder.videoq_size()) + "/" +
    //      std::to_string(MAX_VIDEOQ_SIZE))
    //         .c_str(),
    //     {-0.7, 0.3, -2}, 0.05);

    // FontRenderer::render(
    //     g,
    //     ("pictq: " + std::to_string(videoDecoder.pictq_size()) + "/" +
    //      std::to_string(PICTQ_SIZE))
    //         .c_str(),
    //     {-0.7, 0.25, -2}, 0.05);

    if (hasCapability(Capability::CAP_2DGUI)) {
      imguiDraw();
    }
  }
}

void VideoApp::onSound(AudioIOData &io) {
  if (isPrimary()) {
    if (playing && videoDecoder.hasAudio()) {
      uint8_t *audioBuffer = videoDecoder.getAudioFrame(state().global_clock);

      // check if gotAudioFrame needed to be called before returning
      if (!audioBuffer)
        return;

      int channelSize = io.framesPerBuffer() * sizeof(float);
      int frameSize = channelSize * videoDecoder.audioNumChannels();

      memcpy(io.outBuffer(0), audioBuffer, frameSize);
      // for (int i = 0; i < videoDecoder.audioNumChannels(); ++i) {
      //   memcpy(io.outBuffer(i), audioBuffer + i * channelSize,
      //   channelSize);
      // }

      videoDecoder.gotAudioFrame();
    }
  }
}

bool VideoApp::onKeyDown(const Keyboard &k) {
  if (k.key() == ' ') {
    playing = !playing;
  } else if (k.key() == 'o') {
    if (hasCapability(CAP_OMNIRENDERING)) {
      omniRendering->drawOmni = !omniRendering->drawOmni;
    }
  } else if (k.key() == 'p') {
    equirectangular = !equirectangular;
  } else if (k.key() == Keyboard::TAB) {
    showHUD = !showHUD;
  } else if (k.key() == '[') {
    if (isPrimary()) {
      // TODO: implement get_master_clock
      double pos = state().global_clock;
      double diff = -10.0;
      pos += diff;
      if (pos < 0) {
        pos = 0;
      }
      state().global_clock = pos;
    }
  } else if (k.key() == ']') {
    if (isPrimary()) {
      // TODO: implement get_master_clock
      // TODO: get end pos
      double pos = state().global_clock;
      pos += 10.0;
      state().global_clock += 10.0;
    }
  }
  return true;
}

void VideoApp::onExit() {
  if (hasCapability(Capability::CAP_2DGUI)) {
    imguiShutdown();
  }
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
  if (videoDecoder.hasAudio()) {
    audioDomain()->audioIO().framesPerSecond(videoDecoder.audioSampleRate());
    audioDomain()->audioIO().framesPerBuffer(
        videoDecoder.audioSamplesPerChannel());
    audioDomain()->audioIO().channelsOut(videoDecoder.audioNumChannels());
  }
}
