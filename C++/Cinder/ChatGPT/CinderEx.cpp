#include "cinder/app/App.h"
#include "cinder/app/RendererGl.h"
#include "cinder/gl/gl.h"

using namespace ci;
using namespace ci::app;
using namespace std;

class MobiusTrainApp : public App {
public:
    void setup() override;
    void update() override;
    void draw() override;
    vec3 mobiusPoint(float t, float v);
    void drawCubesOnMobius();
    
private:
    CameraPersp mCam;
    float time = 0.0f;
    static const int NUM_CUBES = 50;
    static constexpr float SPEED = 0.002f;
    static constexpr float MOBIUS_WIDTH = 1.0f;
    static constexpr float MOBIUS_RADIUS = 3.0f;
};

void MobiusTrainApp::setup() {
    mCam.lookAt(vec3(0, 0, 10), vec3(0, 0, 0));
}

void MobiusTrainApp::update() {
    time += SPEED;
}

void MobiusTrainApp::draw() {
    gl::clear(Color(0, 0, 0));
    gl::enableDepthRead();
    gl::enableDepthWrite();
    
    gl::setMatrices(mCam);
    drawCubesOnMobius();
}

void MobiusTrainApp::drawCubesOnMobius() {
    for (int i = 0; i < NUM_CUBES; ++i) {
        float t = fmod(time + (float)i / NUM_CUBES, 1.0f) * 2.0f * M_PI;
        vec3 pos = mobiusPoint(t, 0);
        
        gl::pushModelMatrix();
        gl::translate(pos);
        gl::color(Color(1, 0, 0));
        gl::drawCube(vec3(0), vec3(0.2f));
        gl::popModelMatrix();
    }
}

vec3 MobiusTrainApp::mobiusPoint(float t, float v) {
    float a = MOBIUS_RADIUS + v * cos(t / 2.0f);
    float x = a * cos(t);
    float y = a * sin(t);
    float z = v * sin(t / 2.0f);
    return vec3(x, y, z);
}

CINDER_APP(MobiusTrainApp, RendererGl)
