// g++ 00_Asteroid.cpp SfM.cpp image_proc.cpp helpers.cpp  `pkg-config --cflags --libs opencv4` -std=c++17 -lopencv_xfeatures2d -I /usr/local/include/opencv4/ -o build-asteroid.out


////////// INIT ////////////////////////////////////////////////////////////////////////////////////

///// Imports /////////////////////////////////////////////////////////////

#include <matioCpp/matioCpp.h>
using matioCpp::Element;

#include "SfM.hpp"
using cv::transpose;


///// Globals /////////////////////////////////////////////////////////////

// Load camera-pose data
matioCpp::File /*--------------------*/ rot_cam2astFile( "zb_MATLAB/rot_cam2ast.mat" ); // 3, 3, 37
matioCpp::MultiDimensionalArray<double> rot_cam2ast = rot_cam2astFile.read( "rot_cam2ast" ).asMultiDimensionalArray<double>();
matioCpp::File /*--------------------*/ baselinesFile( "zb_MATLAB/baselines.mat" ); // 3, 36
matioCpp::MultiDimensionalArray<double> baselines = baselinesFile.read( "baselines" ).asMultiDimensionalArray<double>();
matioCpp::File /*--------------------*/ cam_pos_trueFile( "zb_MATLAB/cam_pos_true.mat" ); 
matioCpp::MultiDimensionalArray<double> cam_pos_true = baselinesFile.read( "cam_pos_true" ).asMultiDimensionalArray<double>();

// Images
const string _IMAGE_DIR = "images/";
const string _IMAGE_EXT = "png";
// Define camera intrinsic parameters
const float /*-------*/ fov_deg    = 40.0f; // [deg]
const float /*-------*/ cam_res    = 3000.0f; // [pixels]
const float /*-------*/ foc_length = cam_res/(2*Tanf(fov_deg/2)); // [pixels]
const CameraInstrinsics cam_intr{ foc_length, foc_length, cam_res/2,cam_res/2, cam_res, cam_res };
// Reconstruction
const ubyte n_poses = baselinesFile.read( "baselines" ).dimensions()[1];



////////// HELPER FUNCTIONS ////////////////////////////////////////////////////////////////////////


Mat copy_slice_from_MatIO_3d( matioCpp::MultiDimensionalArray<double>& matioMat, 
                              uint cols, uint rows, uint depth, uint sliceDex ){
    // Get [:,:,`sliceDex`] from a 3D `MatIO_Cpp` array
    Mat     rtnMat = Mat::zeros( rows, cols, CV_32F );
    double* arr    = matioMat.data(); 
    for( uint i = 0; i < rows; ++i ){
        for( uint j = 0; j < cols; ++j ){
            rtnMat.at<float>( i, j ) = (float) arr[ matioMat.rawIndexFromIndices( {j,i,sliceDex} ) ];
        }
    }
    return rtnMat;
}


Mat copy_row_from_MatIO_2d( matioCpp::MultiDimensionalArray<double>& matioMat, 
                            uint cols, uint rows, uint rowDex ){
    // Get [`rowDex`,:] from a 2D `MatIO_Cpp` array
    Mat     rtnMat = Mat::zeros( 1, cols, CV_32F );
    double* arr    = matioMat.data(); 
    if( rowDex < rows ){
        for( uint j = 0; j < cols; ++j ){
            rtnMat.at<float>( 0, j ) = (float) arr[ matioMat.rawIndexFromIndices( {j,rowDex} ) ];
        }
    }
    return rtnMat;
}



////////// MAIN ////////////////////////////////////////////////////////////////////////////////////


int main( int argc, char* argv[] ){

    ///// Compute relative pose between consecutive camera views /////
    Mat /*---*/ rot_prev;
    Mat /*---*/ rot_prevT;
    Mat /*---*/ rot_curr;
    Mat /*---*/ rel_pos /*-----*/ = Mat::zeros( n_poses-1, 3, CV_32F );
    Mat /*---*/ pos_true_camframe = Mat::zeros( n_poses  , 3, CV_32F );
    Mat /*---*/ baseRow_i;
    Mat /*---*/ trueRow_i;
    vector<Mat> rel_rot_cam;
    for( ubyte i = 0; i < (n_poses-1); ++i ){  rel_rot_cam.push_back(  Mat::zeros(3, 3, CV_32F)  );  }

    for( ubyte i = 0; i < (n_poses-1); ++i ){

        // Compute relative rotation
        rot_prev = copy_slice_from_MatIO_3d( rot_cam2ast, 3, 3, n_poses, i   );
        rot_curr = copy_slice_from_MatIO_3d( rot_cam2ast, 3, 3, n_poses, i+1 );
        transpose( rot_prev, rot_prevT );
        rel_rot_cam[i] = rot_prevT * rot_curr;

        // Compute relative position
        baseRow_i = copy_row_from_MatIO_2d( baselines, 3, 36, i );
        baseRow_i.copyTo( rel_pos.row(i) );
    
    }

    // Load true camera positions in the asteroid-fixed frame
    // Note: (only for plotting purposes, not used for shape reconstruction!)
    Mat rot_cam2ast_1  = copy_slice_from_MatIO_3d( rot_cam2ast, 3, 3, n_poses, 1 );
    Mat rot_cam2ast_1T;
    transpose( rot_cam2ast_1, rot_cam2ast_1T );
    Mat cam_pos_true_1 = copy_row_from_MatIO_2d( cam_pos_true, 3, n_poses, 1 );
    Mat cam_pos_true_i;
    Mat pos_true_camframe_i;

    for( ubyte i = 1; i < n_poses; ++i ){
        cam_pos_true_i = copy_row_from_MatIO_2d( cam_pos_true, 3, n_poses, i );
        pos_true_camframe_i = rot_cam2ast_1T * (cam_pos_true_i - cam_pos_true_1);
        pos_true_camframe_i.copyTo( pos_true_camframe.row(i) );
    }

    // cout << rot_cam2astFile.read("rot_cam2ast").name() << ", " << endl
    //      << rot_cam2astFile.read("rot_cam2ast").dimensions()[0] << ", " 
    //      << rot_cam2astFile.read("rot_cam2ast").dimensions()[1] << ", " 
    //      << rot_cam2astFile.read("rot_cam2ast").dimensions()[2] << endl; 
         
    // cout << baselinesFile.read("baselines").name() << ", " << endl
    //      << baselinesFile.read("baselines").dimensions()[0] << ", " 
    //      << baselinesFile.read("baselines").dimensions()[1] << endl; 

    // Initialize pose graph objects
    // Read images
    cout << "About to calculate KAZE keypoints ..." << endl;
    vector<NodePtr> nodes = images_to_nodes( _IMAGE_DIR, _IMAGE_EXT );

    // cout << "Calculated KAZE keypoints for the following images ..." << endl;
    // for( NodePtr node : nodes ){
    //     cout << node->imPth << ", " << node->kypts.size() << " keypoints" << endl;
    // }

    return 0;
}