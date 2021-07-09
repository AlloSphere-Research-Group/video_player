
#include "alloutil/al_OmniApp.hpp"
#include "allocore/system/al_Time.hpp"

#include "al_VideoPlayer.cpp"

#include "Cuttlebone/Cuttlebone.hpp"

#include "common.h"

using namespace al;
class MyApp: public OmniApp {
public:
	MyApp(std::string name = "omniapp", bool slave = false)
	{
		mPlaying = false;
		mTaker.start();
	}

    virtual ~MyApp()
	{
		mTaker.stop();
	}

	virtual bool onCreate() override;
    virtual void onDraw(Graphics& gl) override;
	virtual void onAnimate(al_sec dt) override;

	SharedState &state() {return mState;}
private:
	Texture mVideoTexture;

	Mesh mQuad;
	SharedState mState;
	bool mPlaying;

	cuttlebone::Taker<SharedState> mTaker;
};

bool MyApp::onCreate()
{
	OmniApp::onCreate();
	auto& s = shader();
    s.begin();
    // the shader will look for texture0 uniform's data
    // at binding point '0' >> we will bind the texture at '0'
//    s.uniform("texture0", 0);
    // how much we mix the texture
    s.uniform("texture", 1.0);
    s.uniform("lighting", 0.1);
    s.end();

	mQuad.reset();
	addSphereWithTexcoords(mQuad, 1, 128);

//	mQuad.generateNormals();

	state().frameNum = 0;
	Window::fps(30);
	omni().mode(OmniStereo::MONO);
	mVideoTexture.target(Texture::TEXTURE_2D);
//		mVideoTexture.type(Graphics::BYTES_4);
	mVideoTexture.resize(VIDEO_WIDTH, VIDEO_HEIGHT);
	mVideoTexture.allocate();
	mVideoTexture.filter(Texture::NEAREST);
}

void MyApp::onDraw(Graphics &g)
{
	g.pushMatrix();
	g.rotate(-90, 0, 1, 0);
	//	g.scale(0.3);
	mVideoTexture.bind();
	g.draw(mQuad);
	mVideoTexture.unbind();
	g.popMatrix();
}

void MyApp::onAnimate(al_sec dt)
{
	mTaker.get(state());
	memcpy(mVideoTexture.array().data.ptr, state().pixels, VIDEO_WIDTH*VIDEO_HEIGHT * sizeof(u_int32_t));
	mVideoTexture.dirty();
//	mVideoTexture.readFrame(state().frameNum);
}

int main(int argc, char *argv[])
{
	MyApp().start();
    return 0;
}
