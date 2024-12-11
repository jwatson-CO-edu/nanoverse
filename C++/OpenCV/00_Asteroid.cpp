// g++ 00_Asteroid.cpp SfM.cpp image_proc.cpp helpers.cpp  `pkg-config --cflags --libs opencv4` -std=c++17 -lopencv_xfeatures2d -I /usr/local/include/opencv4/ -o build-asteroid.out

// #include <matio.h>
#include <matioCpp/matioCpp.h>

#include "SfM.hpp"

matioCpp::File /*--------------------*/ rot_cam2astFile( "zb_MATLAB/rot_cam2ast.mat" ); // 3, 3, 37
matioCpp::MultiDimensionalArray<double> rot_cam2ast = rot_cam2astFile.read("rot_cam2ast").asMultiDimensionalArray<double>();

matioCpp::File /*--------------------*/ baselinesFile( "zb_MATLAB/baselines.mat" ); // 3, 36
matioCpp::MultiDimensionalArray<double> baselines = baselinesFile.read("baselines").asMultiDimensionalArray<double>();

const string _IMAGE_DIR  = "images/";
const string _IMAGE_EXT  = "png";
const float  fov_deg     = 40.0f; // [deg]
const float  cam_res     = 3000.0f; // [pixels]
const float  foc_length  = cam_res/(2*Tanf(fov_deg/2)); // [pixels]
const Mat    cam_intr    = camera_instrinsics( foc_length, foc_length, cam_res/2,cam_res/2 );
const ubyte  n_poses     = baselinesFile.read("baselines").dimensions()[1];


int main( int argc, char* argv[] ){

    
    

    cout << rot_cam2astFile.read("rot_cam2ast").name() << ", " << endl
         << rot_cam2astFile.read("rot_cam2ast").dimensions()[0] << ", " 
         << rot_cam2astFile.read("rot_cam2ast").dimensions()[1] << ", " 
         << rot_cam2astFile.read("rot_cam2ast").dimensions()[2] << endl; 
         
    cout << baselinesFile.read("baselines").name() << ", " << endl
         << baselinesFile.read("baselines").dimensions()[0] << ", " 
         << baselinesFile.read("baselines").dimensions()[1] << endl; 

    cout << "About to calculate KAZE keypoints ..." << endl;

    vector<NodePtr> nodes = images_to_nodes( _IMAGE_DIR, _IMAGE_EXT );

    cout << "Calculated KAZE keypoints for the following images ..." << endl;
    for( NodePtr node : nodes ){
        cout << node->imPth << ", " << node->kypts.size() << " keypoints" << endl;
    }

    return 0;
}