// g++ 00_Asteroid.cpp SfM.cpp image_proc.cpp helpers.cpp  `pkg-config --cflags --libs opencv4` -std=c++17 -lopencv_xfeatures2d -I /usr/local/include/opencv4/ -o build-asteroid.out

#include <matioCpp/matioCpp.h>
using matioCpp::Element;

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

// FIXME: MATLAB IS COLUMN-MAJOR

Mat copy_slice_from_MatIO_3d( matioCpp::MultiDimensionalArray<double>& matioMat, uint rows, uint cols, uint depth, uint sliceDex ){
    Mat     rtnMat = Mat::zeros( rows, cols, CV_32F );
    double* arr    = matioMat.data(); // FIXME, START HERE: SEGMENTATION FAULT, GET THE SIZE OF THE ARRAY
    Element<double> elem;

    

    for( uint i = 0; i < rows; ++i ){
        for( uint j = 0; j < cols; ++j ){
            /* build-asteroid.out: /usr/local/include/matioCpp/impl/MultiDimensionalArray.tpp:333: 
            matioCpp::MultiDimensionalArray<T>::element_type& matioCpp::MultiDimensionalArray<T>::operator[](matioCpp::MultiDimensionalArray<T>::index_type) 
            [with T = double; matioCpp::MultiDimensionalArray<T>::reference = double&; matioCpp::MultiDimensionalArray<T>::index_type = long unsigned int]: 
            Assertion `el < numberOfElements() && "[matioCpp::MultiDimensionalArray::operator[]] 
            The required element is out of bounds."' failed.
            Aborted (core dumped)
            */
            rtnMat.at<float>( j, i ) = (float) arr[ i*cols*depth + j*depth + sliceDex ];
        }
    }
    return rtnMat;
}

int main( int argc, char* argv[] ){

    Mat /*---*/ rot_prev;
    Mat /*---*/ rot_curr;
    Mat /*---*/ rel_pos = Mat::zeros( 3, n_poses-1, CV_32F );
    vector<Mat> rel_rot_cam;
    for( ubyte i = 0; i < (n_poses-1); ++i ){  rel_rot_cam.push_back(  Mat::zeros(3, 3, CV_32F)  );  }
    
    

    for( ubyte i = 0; i < n_poses; ++i ){
        rot_prev = copy_slice_from_MatIO_3d( rot_cam2ast, 3, 3, n_poses, i   );
        rot_curr = copy_slice_from_MatIO_3d( rot_cam2ast, 3, 3, n_poses, i-1 );
    }

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