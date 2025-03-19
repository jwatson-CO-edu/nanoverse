#include "cinder/app/App.h"
#include "cinder/app/RendererGl.h"
#include "cinder/gl/gl.h"
#include "cinder/Camera.h"
#include "cinder/CameraUi.h"
#include "cinder/params/Params.h"

using namespace ci;
using namespace ci::app;
using namespace std;

class MobiusTrainApp : public App {
public:
    void setup() override;
    void update() override;
    void draw() override;
    void mouseDown(MouseEvent event) override;
    void mouseDrag(MouseEvent event) override;
    void keyDown(KeyEvent event) override;

private:
    // Returns a 3D point on the Mobius strip at parameter values u and v
    vec3 getMobiusPoint(float u, float v);
    
    // Creates a cube mesh centered at the origin with specified dimensions
    gl::BatchRef createCube(const vec3& size);
    
    // Animation properties
    float mAnimationTime = 0.0f;
    float mAnimationSpeed = 0.5f;
    
    // Mobius strip properties
    float mMobiusRadius = 3.0f;
    float mMobiusWidth = 1.0f;
    int mNumCubes = 20;
    
    // Camera
    CameraPersp mCamera;
    CameraUi mCameraUi;
    
    // Cube properties
    vec3 mCubeSize = vec3(0.3f, 0.3f, 0.3f);
    gl::BatchRef mCubeBatch;
    
    // Parameters GUI
    params::InterfaceGlRef mParams;
};

void MobiusTrainApp::setup() {
    // Set up camera
    mCamera.setPerspective(45.0f, getWindowAspectRatio(), 0.1f, 100.0f);
    mCamera.lookAt(vec3(0, 0, 10), vec3(0));
    mCameraUi = CameraUi(&mCamera, getWindow());
    
    // Create cube mesh
    auto cubeShader = gl::getStockShader(gl::ShaderDef().color());
    mCubeBatch = createCube(mCubeSize);
    mCubeBatch->getGlslProg()->uniform("uColor", ColorA(1.0f, 0.0f, 0.0f, 1.0f));
    
    // Set up GUI
    mParams = params::InterfaceGl::create("Mobius Train Controls", vec2(200, 200));
    mParams->addParam("Animation Speed", &mAnimationSpeed).min(0.1f).max(2.0f).step(0.1f);
    mParams->addParam("Mobius Radius", &mMobiusRadius).min(1.0f).max(10.0f).step(0.5f);
    mParams->addParam("Mobius Width", &mMobiusWidth).min(0.5f).max(3.0f).step(0.1f);
    mParams->addParam("Number of Cubes", &mNumCubes).min(5).max(50).step(1);
    
    // Enable depth testing
    gl::enableDepthRead();
    gl::enableDepthWrite();
}

void MobiusTrainApp::update() {
    // Update animation time
    mAnimationTime += mAnimationSpeed * getFrameRate() / 60.0f;
}

void MobiusTrainApp::draw() {
    // Clear the window
    gl::clear(Color(0.1f, 0.1f, 0.1f));
    
    // Set up the camera
    gl::setMatrices(mCamera);
    
    // Draw the Mobius strip as a wireframe
    gl::color(Color(0.5f, 0.5f, 0.5f));
    gl::lineWidth(1.0f);
    gl::begin(GL_LINES);
    const int stepsU = 100;
    const int stepsV = 10;
    for (int u = 0; u < stepsU; u++) {
        for (int v = 0; v < stepsV; v++) {
            float u1 = u / (float)stepsU * 2.0f * M_PI;
            float u2 = (u + 1) / (float)stepsU * 2.0f * M_PI;
            float v1 = (v / (float)stepsV - 0.5f) * mMobiusWidth;
            float v2 = ((v + 1) / (float)stepsV - 0.5f) * mMobiusWidth;
            
            gl::vertex(getMobiusPoint(u1, v1));
            gl::vertex(getMobiusPoint(u1, v2));
            
            gl::vertex(getMobiusPoint(u1, v1));
            gl::vertex(getMobiusPoint(u2, v1));
        }
    }
    gl::end();
    
    // Draw the train of cubes
    gl::color(Color(1.0f, 0.0f, 0.0f));
    for (int i = 0; i < mNumCubes; i++) {
        float offset = mAnimationTime * 0.1f + i * (2.0f * M_PI / mNumCubes);
        float u = fmod(offset, 2.0f * M_PI);
        float v = 0.0f; // Center of the strip
        
        // Calculate position on the Mobius strip
        vec3 position = getMobiusPoint(u, v);
        
        // Calculate orientation
        // Tangent vector along the strip (in u direction)
        float delta = 0.01f;
        vec3 tangent = getMobiusPoint(u + delta, v) - getMobiusPoint(u - delta, v);
        tangent = normalize(tangent);
        
        // Normal vector (in v direction, but needs to be perpendicular to tangent)
        vec3 normal = getMobiusPoint(u, v + delta) - getMobiusPoint(u, v - delta);
        normal = normalize(normal);
        
        // Binormal vector (perpendicular to both tangent and normal)
        vec3 binormal = cross(tangent, normal);
        binormal = normalize(binormal);
        
        // Re-compute normal to ensure orthogonality
        normal = cross(binormal, tangent);
        
        // Create rotation matrix from these vectors
        mat4 rotation;
        rotation[0] = vec4(tangent, 0);
        rotation[1] = vec4(normal, 0);
        rotation[2] = vec4(binormal, 0);
        rotation[3] = vec4(0, 0, 0, 1);
        
        // Create model matrix combining position and orientation
        mat4 model = glm::translate(mat4(1.0f), position) * rotation;
        
        // Draw the cube
        gl::pushModelMatrix();
        gl::multModelMatrix(model);
        gl::color(ColorA(1.0f, 0.0f, 0.0f, 1.0f));
        mCubeBatch->draw();
        gl::popModelMatrix();
    }
    
    // Draw GUI
    mParams->draw();
}

vec3 MobiusTrainApp::getMobiusPoint(float u, float v) {
    // Parametric equation for a Mobius strip
    float x = (mMobiusRadius + v * cos(u / 2)) * cos(u);
    float y = (mMobiusRadius + v * cos(u / 2)) * sin(u);
    float z = v * sin(u / 2);
    
    return vec3(x, y, z);
}

gl::BatchRef MobiusTrainApp::createCube(const vec3& size) {
    // Create a cube mesh centered at the origin
    auto cube = gl::VboMesh::create(gl::Cube(vec3(0), size));
    return gl::Batch::create(cube, gl::getStockShader(gl::ShaderDef().color()));
}

void MobiusTrainApp::mouseDown(MouseEvent event) {
    mCameraUi.mouseDown(event);
}

void MobiusTrainApp::mouseDrag(MouseEvent event) {
    mCameraUi.mouseDrag(event);
}

void MobiusTrainApp::keyDown(KeyEvent event) {
    if (event.getChar() == 'f') {
        setFullScreen(!isFullScreen());
    }
    else if (event.getChar() == 'r') {
        mCamera.lookAt(vec3(0, 0, 10), vec3(0));
    }
}

// CINDER_APP macro connects the app class to the rendering subsystem
CINDER_APP(MobiusTrainApp, RendererGl)