
#include "alloutil/al_OmniApp.hpp"
#include "allocore/system/al_Time.hpp"

#include "al_VideoPlayer.cpp"

#include "Cuttlebone/Cuttlebone.hpp"


typedef struct {
	int frameNum;
} SharedState;

//#define BUILD_RENDERER

using namespace al;

class MyApp: public OmniApp {
public:
	MyApp(std::string name = "omniapp", bool slave = false)

#ifdef BUILD_RENDERER
#else
	    :
	mMaker("192.168.10.0")
#endif
	{
		mPlaying = false;
#ifdef BUILD_RENDERER
		mTaker.start();
#else
//		mMaker.start();
#endif

	}

    virtual ~MyApp()
	{
#ifdef BUILD_RENDERER
		mTaker.stop();
#else
		mMaker.stop();
#endif

	}

    virtual bool onCreate() override;
    virtual void onDraw(Graphics& gl) override;
	virtual void onAnimate(al_sec dt) override;
#ifdef BUILD_RENDERER
#else
    virtual void onSound(AudioIOData &io) override;
    virtual void onMessage(osc::Message& m) override;
	virtual bool onKeyDown(const Keyboard& k) override;
#endif

	SharedState &state() {return mState;}
private:
//	VideoTexturer mVideoTexture {"/data/andres/PlieguesALTh264.mov"};
//    VideoTexturer mVideoTexture {"/data/andres/Pliegues1024.mp4"};
//    VideoTexture mVideoTexture {"/home/andres/Probably-3mins-final.mp4"};
    VideoTexture mVideoTexture {"/home/andres/Documents/07 Proyectos y trabajos/06 Musica/geminiflux/CCT1.mp4"};
//    VideoTexturer mVideoTexture {"/media/andres/LaCie_Mac_Only/_AlloArchive/AlloSphere Archive_032714/1-New-UK-KucheraMorin.mov", true};
//    VideoTexturer mVideoTexture {"/media/andres/LACIE SHARE/brain-test.mp4", VideoFileReader::SYNC_AUDIO};
//    VideoTexturer mVideoTexture {"/home/andres/Videos/Webcam/2014-12-08-210226.webm"};
    bool mSideBySide = true;
	Mesh mQuadL, mQuadR;
	SharedState mState;
	bool mPlaying;

#ifdef BUILD_RENDERER
	cuttlebone::Taker<SharedState> mTaker;
#else
	cuttlebone::Maker<SharedState> mMaker;
#endif
};

bool MyApp::onCreate()
{
	OmniApp::onCreate();
	auto& s = shader();
    s.begin();
    s.uniform("texture", 1.0);
    s.uniform("lighting", 0.1);
    s.end();
    if (mSideBySide) {
        mQuadL.reset();
        mQuadL.primitive(Graphics::QUADS);
        mQuadL.vertex(-1, -1, 0);
        mQuadL.texCoord(0, 1);
        mQuadL.vertex( 1, -1, 0);
        mQuadL.texCoord(0.5, 1);
        mQuadL.vertex( 1,  1, 0);
        mQuadL.texCoord(0.5, 0);
        mQuadL.vertex(-1,  1, 0);
        mQuadL.texCoord(0, 0);

        mQuadR.reset();
        mQuadR.primitive(Graphics::QUADS);
        mQuadR.vertex(-1, -1, 0);
        mQuadR.texCoord(0.5, 1);
        mQuadR.vertex( 1, -1, 0);
        mQuadR.texCoord(1, 1);
        mQuadR.vertex( 1,  1, 0);
        mQuadR.texCoord(1, 0);
        mQuadR.vertex(-1,  1, 0);
        mQuadR.texCoord(0.5, 0);
    } else {
        mQuadL.reset();
        mQuadL.primitive(Graphics::QUADS);
        mQuadL.vertex(-1, -1, 0);
        mQuadL.texCoord(0, 1);
        mQuadL.vertex( 1, -1, 0);
        mQuadL.texCoord(1, 1);
        mQuadL.vertex( 1,  1, 0);
        mQuadL.texCoord(1, 0);
        mQuadL.vertex(-1,  1, 0);
        mQuadL.texCoord(0, 0);
    }

	mVideoTexture.print();
    mVideoTexture.setPlayMode(VideoFileReader::PLAY_ONESHOT);

	state().frameNum = 0;
	Window::fps(30);
#ifdef BUILD_RENDERER
	omni().mode(OmniStereo::MONO);
#else
	omniEnable(false);
    initAudio(44100);
#endif
}

void MyApp::onAnimate(al_sec dt)
{
#ifdef BUILD_RENDERER
	mTaker.get(mState);
	mVideoTexture.readFrame(state().frameNum);
#else
//    std::cout << dt << std::endl;
    //	mTaker.get(mState);

    if (frame == 5)
    {
//        osc::Send sender(9090, "localhost");
//        sender.send("/play", 1.0f);

        mVideoTexture.start();
        mPlaying = true;
    }
//    if (frame == 176) {
    //    }
    if (mPlaying) {
        mVideoTexture.readFrame();
        state().frameNum++;

        //	mMaker.set(state());
    }
//    std::cout << (int) frame - mVideoTexture.currentFrame()<< std::endl;
#endif
}

void MyApp::onDraw(Graphics &g)
{
#ifdef BUILD_RENDERER
	//	mVideoTexture.readFrame();
	g.pushMatrix();
	g.rotate(-90, 0, 1, 0);
	//	g.scale(0.3);
	mVideoTexture.bind();
	g.draw(mQuad);
	mVideoTexture.unbind();
	g.popMatrix();
#else
    mVideoTexture.bind();
    g.pushMatrix();
    g.translate(-1, 0.1, -2);
    g.draw(mQuadL);
    g.popMatrix();
    if (mSideBySide) {
        g.pushMatrix();
        g.translate(1, -0.1, -2);
        g.draw(mQuadR);
        g.popMatrix();
	}
	mVideoTexture.unbind();
//    std::cout << state().frameNum - mVideoTexture.currentFrame() << std::endl;
#endif
}

#ifdef BUILD_RENDERER

#else
void MyApp::onSound(AudioIOData &io)
{
    float buffer[8192];
    int numFileChnls = 2;
    for (int i = 0; i < numFileChnls; i++) {
        size_t elementsRead = mVideoTexture.readAudio(i, buffer, io.framesPerBuffer());
        if (elementsRead > 0) {
            memcpy(io.outBuffer(i), buffer, elementsRead * sizeof(float));
        }
//        if (elementsRead < io.framesPerBuffer()) {
//            std::cout << "buffer underrun! read " << elementsRead <<  std::endl;
//        }
    }
}

void MyApp::onMessage(osc::Message &m)
{

}

bool MyApp::onKeyDown(const Keyboard &k)
{
	if (k.key() == 32) {
		mPlaying = true;
		osc::Send sender(9090, "localhost");
		sender.send("/play", 1.0f);
	} else if (k.keyAsNumber() == 0) {
        std::cout << "play again" << std::endl;
        mVideoTexture.seek(0);
        mVideoTexture.start();
    }
	return true;
}
#endif

int main(int argc, char *argv[])
{
	MyApp().start();
    return 0;
}
