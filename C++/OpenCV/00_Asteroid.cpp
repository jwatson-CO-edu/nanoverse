// g++ 00_Asteroid.cpp SfM.cpp image_proc.cpp helpers.cpp  `pkg-config --cflags --libs opencv4` -std=c++17 -lopencv_xfeatures2d -I /usr/local/include/opencv4/ -o build-asteroid.out

// #include <matio.h>
#include <matioCpp/matioCpp.h>

#include "SfM.hpp"


const string _IMAGE_DIR  = "images/";
const string _IMAGE_EXT  = "png";
const float  fov_deg     = 40.0f; // [deg]
const float  cam_res     = 3000.0f; // [pixels]
const float  foc_length  = cam_res/(2*Tanf(fov_deg/2)); // [pixels]
const Mat    cam_intr    = camera_instrinsics( foc_length, foc_length, cam_res/2,cam_res/2 );

matioCpp::File rot_cam2astFile( "zb_MATLAB/rot_cam2ast.mat" );
matioCpp::File baselinesFile( "zb_MATLAB/baselines.mat" );

matioCpp::CellArray rot_cam2ast = rot_cam2astFile.read("rot_cam2ast").asCellArray();
matioCpp::CellArray baselinese  = baselinesFile.read("baselines").asCellArray();


// const mat_t* rot_cam2ast = Mat_Open( "rot_cam2ast.mat", MAT_ACC_RDWR );
// const mat_t* baselines   = Mat_Open( "baselines.mat"  , MAT_ACC_RDWR );
// Mat_SizeOf(  )


int main( int argc, char* argv[] ){

    cout << "About to calculate KAZE keypoints ..." << endl;

    vector<NodePtr> nodes = images_to_nodes( _IMAGE_DIR, _IMAGE_EXT );

    cout << "Calculated KAZE keypoints for the following images ..." << endl;
    for( NodePtr node : nodes ){
        cout << node->imPth << endl;
    }

    return 0;
}