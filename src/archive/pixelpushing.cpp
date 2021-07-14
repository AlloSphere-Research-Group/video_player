
#include "alloutil/al_OmniApp.hpp"
#include "allocore/system/al_Time.hpp"

#include "al_VideoPlayer.cpp"

#include "Cuttlebone/Cuttlebone.hpp"

#include "common.h"


using namespace al;
class MyApp: public App {
public:
	MyApp(std::string name = "omniapp", bool slave = false) :
        mMaker("127.0.0.1")
	{
		mPlaying = false;
		mMaker.start();
        initWindow(Window::Dim(VIDEO_WIDTH, VIDEO_HEIGHT), "Pixelpusher", 30);
	}

    virtual ~MyApp()
	{
		mMaker.stop();
	}

	virtual void onCreate(const ViewpointWindow& win) override;
    virtual void onDraw(Graphics& gl) override;
	virtual void onAnimate(al_sec dt) override;

	virtual void onKeyDown(const Keyboard& k) override;


	SharedState &state() {return mState;}
private:
//	VideoPlayer mVideoTexture {"/data/andres/PlieguesALTh264.mov"};
//	VideoFileReader mVideoTexture {"/data/andres/Pliegues1024.mp4"};
// VideoTexture mVideoTexture {"/home/andres/Probably-3mins-final.mp4", VideoFileReader::SYNC_VIDEO, TEXTURE_WIDTH};
    VideoTexture mVideoTexture {"/Users/kurt/Downloads/leader_512kb.mp4", VideoFileReader::SYNC_VIDEO};

	Mesh mQuad;
	SharedState mState;
	bool mPlaying;

	cuttlebone::Maker<SharedState> mMaker;
};

void MyApp::onCreate(const ViewpointWindow &win)
{
	mQuad.reset();
	addSphereWithTexcoords(mQuad, 1, 128);

//	mQuad.generateNormals();

	state().frameNum = 0;
//	Window::fps(30);

	mVideoTexture.print();
}

void MyApp::onDraw(Graphics &g)
{
	if(mPlaying) {
//		mVideoTexture.readFrame(state().frameNum);
		//	mVideoTexture.readFrame();
		g.pushMatrix();
		g.rotate(-90, 0, 1, 0);
//			g. translate(0, 0, -4);
//			g.scale(0.3);
		mVideoTexture.bind();
		g.draw(mQuad);
		mVideoTexture.unbind();
		g.popMatrix();
		// state().frameNum++;
	}
	memcpy(state().pixels, mVideoTexture.array().data.ptr, VIDEO_WIDTH*VIDEO_HEIGHT * sizeof(u_int32_t));
	mMaker.set(state());
}

void MyApp::onAnimate(al_sec dt)
{
    static int frame = 0;
//	mTaker.get(mState);
    // if (frame++ == 150) {
    //     osc::Send sender(9090, "localhost");
    //     sender.send("/play", 1.0f);
    //     mPlaying = true;
    //     mVideoTexture.start();
    // }
//    if (frame == 176) {
//    }
    if (mPlaying) {
			mVideoTexture.readFrame();
			state().frameNum++;
    }
    // std::cout << dt << std::endl;
}

void MyApp::onKeyDown(const Keyboard &k)
{
	if (k.key() == 32) {
		osc::Send sender(9090, "localhost");
		sender.send("/play", 1.0f);
		mVideoTexture.start();
		mPlaying = true;
	}
}

int main(int argc, char *argv[])
{
	MyApp().start();
    return 0;
}
