// g++ 00_Asteroid.cpp SfM.cpp image_proc.cpp helpers.cpp  `pkg-config --cflags --libs opencv4` -std=c++17 -lopencv_xfeatures2d -I /usr/local/include/opencv4/ -o build-asteroid.out


////////// INIT ////////////////////////////////////////////////////////////////////////////////////

///// Imports /////////////////////////////////////////////////////////////

#include <matioCpp/matioCpp.h>
using matioCpp::Element;

#include "SfM.hpp"
using cv::transpose;


///// Globals /////////////////////////////////////////////////////////////

// Load camera-pose data
// matioCpp::File /*--------------------*/ rot_cam2astFile(  ); // 3, 3, 37
// matioCpp::MultiDimensionalArray<double> rot_cam2ast = rot_cam2astFile.read( "rot_cam2ast" ).asMultiDimensionalArray<double>();
// matioCpp::File /*--------------------*/ baselinesFile( "zb_MATLAB/baselines.mat" ); // 3, 36
// matioCpp::MultiDimensionalArray<double> baselines = baselinesFile.read( "baselines" ).asMultiDimensionalArray<double>();


// matioCpp::File /*--------------------*/ cam_pos_trueFile( "zb_MATLAB/cam_pos_true.mat" ); 
// // matioCpp::MultiDimensionalArray<double> cam_pos_true = baselinesFile.read( "cam_pos_true" ).asMultiDimensionalArray<double>();
// matioCpp::Vector<double> cam_pos_true = baselinesFile.read( "cam_pos_true" ).asVector<double>();






// Images
const string _IMAGE_DIR = "images/";
const string _IMAGE_EXT = "png";
// Define camera intrinsic parameters
const float /*-------*/ fov_deg    = 40.0f; // [deg]
const float /*-------*/ cam_res    = 3000.0f; // [pixels]
const float /*-------*/ foc_length = cam_res/(2*Tanf(fov_deg/2)); // [pixels]
const CameraInstrinsics cam_intr{ foc_length, foc_length, cam_res/2,cam_res/2, cam_res, cam_res };
// Reconstruction
// const ubyte n_poses = baselinesFile.read( "baselines" ).dimensions()[1];



////////// HELPER FUNCTIONS ////////////////////////////////////////////////////////////////////////

Mat MatIO_3d_to_cv_Mat( string filName, string varName ){
    matioCpp::File /*--------------------*/ matFile3d( filName );
    matioCpp::MultiDimensionalArray<double> matVar3d = matFile3d.read( varName ).asMultiDimensionalArray<double>();
    double* /*---------------------------*/ arr /**/ = matVar3d.data();
    int /*-------------------------------*/ rows     = matFile3d.read( varName ).dimensions()[0];
    int /*-------------------------------*/ cols     = matFile3d.read( varName ).dimensions()[1];
    int /*-------------------------------*/ slcs     = matFile3d.read( varName ).dimensions()[2];
    int /*-------------------------------*/ sizes[]  = {slcs, cols, rows};
    Mat /*-------------------------------*/ rtnMat   = Mat(3, sizes, CV_32F, cv::Scalar(0));
    for( size_t i = 0; i < rows; ++i ){
        for( size_t j = 0; j < cols; ++j ){
            for( size_t k = 0; k < slcs; ++k ){
                rtnMat.at<float>( k, j, i ) = (float) arr[ matVar3d.rawIndexFromIndices( {i, j, k} ) ];
            }
        }
    }
    return rtnMat;
}


Mat MatIO_2d_to_cv_Mat( string filName, string varName ){
    matioCpp::File /*--------------------*/ matFile2d( filName );
    matioCpp::MultiDimensionalArray<double> matVar2d = matFile2d.read( varName ).asMultiDimensionalArray<double>();
    double* /*---------------------------*/ arr /**/ = matVar2d.data();
    int /*-------------------------------*/ rows     = matFile2d.read( varName ).dimensions()[0];
    int /*-------------------------------*/ cols     = matFile2d.read( varName ).dimensions()[1];
    int /*-------------------------------*/ sizes[]  = {cols, rows};
    Mat /*-------------------------------*/ rtnMat   = Mat(2, sizes, CV_32F, cv::Scalar(0));
    for( size_t i = 0; i < rows; ++i ){
        for( size_t j = 0; j < cols; ++j ){
            rtnMat.at<float>( j, i ) = (float) arr[ matVar2d.rawIndexFromIndices( {i, j} ) ];
        }
    }
    return rtnMat;
}


Mat get_triples_from_MatIO_var_dbbl( string fPath ){
    // Get triples from a car with an inaccessible type
    mat_t*    matfp /*--*/ = Mat_Open( fPath.c_str(), MAT_ACC_RDONLY );
    matvar_t* variable     = Mat_VarReadNext( matfp );
    double*   varData /**/ = (double*) variable->data;
    size_t    num_elements = variable->nbytes / Mat_SizeOfClass( variable->class_type ); 
    size_t    num_rows     = num_elements / 3;
    size_t    row /*----*/ = 0;
    Mat /*-*/ rtnMat;
    if( num_elements%3 != 0 ){
        cerr << "Var elems were NOT a multiple of 3!" << endl;
        rtnMat = Mat::zeros( 0, 3, CV_32F );
    }else{
        rtnMat = Mat::zeros( num_rows, 3, CV_32F );
        for( size_t i = 0; i < num_elements; i += 3 ){
            for( ubyte j = 0; j < 3; ++j ){
                rtnMat.at<float>( row, j ) = varData[i+j];
            }
            ++row;
        }
    }
    return rtnMat;
}


////////// MAIN ////////////////////////////////////////////////////////////////////////////////////


int main( int argc, char* argv[] ){

    ///// Fetch provided data /////
    Mat rot_cam2ast  = MatIO_3d_to_cv_Mat( "zb_MATLAB/rot_cam2ast.mat", "rot_cam2ast" );
    Mat baselines    = MatIO_2d_to_cv_Mat( "zb_MATLAB/baselines.mat"  , "baselines" );
    Mat cam_pos_true = get_triples_from_MatIO_var_dbbl( "zb_MATLAB/cam_pos_true.mat" );

    // cout << rot_cam2ast  << endl << endl;
    // cout << baselines    << endl << endl;
    // cout << cam_pos_true << endl << endl;

    // FIXME, START HERE: PRINT OUT THE DIMS AND CONTENTS OF EACH Mat IN ORDER TO UNDERSTAND THE PROBLEM

    ///// Compute relative pose between consecutive camera views /////
    Mat /*---*/ rot_prev;
    Mat /*---*/ rot_prevT;
    Mat /*---*/ rot_curr;
    // Mat /*---*/ rel_pos /*-----*/ = Mat::zeros( n_poses-1, 3, CV_32F );
    // Mat /*---*/ pos_true_camframe = Mat::zeros( n_poses  , 3, CV_32F );
    
    Mat /*---*/ baseRow_i;
    Mat /*---*/ trueRow_i;
    vector<Mat> rel_rot_cam;
    // for( ubyte i = 0; i < (n_poses-1); ++i ){  rel_rot_cam.push_back(  Mat::zeros(3, 3, CV_32F)  );  }

    // for( ubyte i = 0; i < (n_poses-1); ++i ){

    //     // Compute relative rotation
    //     // rot_prev = copy_slice_from_MatIO_3d( rot_cam2ast, 3, 3, n_poses, i   );
    //     // rot_curr = copy_slice_from_MatIO_3d( rot_cam2ast, 3, 3, n_poses, i+1 );
    //     transpose( rot_prev, rot_prevT );
    //     rel_rot_cam[i] = rot_prevT * rot_curr;

    //     // Compute relative position
    //     // baseRow_i = copy_row_from_MatIO_2d( baselines, 3, 36, i );
    //     // baseRow_i.copyTo( rel_pos.row(i) );
    
    // }


    // Mat rot_cam2ast_1  = copy_slice_from_MatIO_3d( rot_cam2ast, 3, 3, n_poses, 1 );
    Mat rot_cam2ast_1T;
    // transpose( rot_cam2ast_1, rot_cam2ast_1T );
    Mat cam_pos_true_1;
    transpose( cam_pos_true.row(0), cam_pos_true_1 );
    Mat cam_pos_true_i;
    Mat pos_true_camframe_i;


    // for( ubyte i = 1; i < n_poses; ++i ){
    //     // cam_pos_true_i = cam_pos_true.row(i);
    //     transpose( cam_pos_true.row(i), cam_pos_true_i );
    //     pos_true_camframe_i = rot_cam2ast_1T * (cam_pos_true_i - cam_pos_true_1);
    //     pos_true_camframe_i.copyTo( pos_true_camframe.row(i) );
    // }
    
    // Initialize pose graph objects
    // Read images
    // cout << "About to calculate KAZE keypoints ..." << endl;
    // vector<NodePtr> nodes = images_to_nodes( _IMAGE_DIR, _IMAGE_EXT );

    // cout << "Calculated KAZE keypoints for the following images ..." << endl;
    // for( NodePtr node : nodes ){
    //     cout << node->imPth << ", " << node->kypts.size() << " keypoints" << endl;
    // }

    return 0;
}


////////// SPARE PARTS /////////////////////////////////////////////////////////////////////////////

// cout << rot_cam2astFile.read("rot_cam2ast").name() << ", " << endl
    //      << rot_cam2astFile.read("rot_cam2ast").dimensions()[0] << ", " 
    //      << rot_cam2astFile.read("rot_cam2ast").dimensions()[1] << ", " 
    //      << rot_cam2astFile.read("rot_cam2ast").dimensions()[2] << endl; 
         
    // cout << baselinesFile.read("baselines").name() << ", " << endl
    //      << baselinesFile.read("baselines").dimensions()[0] << ", " 
    //      << baselinesFile.read("baselines").dimensions()[1] << endl; 



    
    

    // while (variable != NULL) {
    //     std::cout << "Variable name: " << variable->name << std::endl;

    //     size_t num_elements = variable->nbytes / Mat_SizeOfClass(variable->class_type); 

    //     // Print the size
    //     std::cout << "Size: " << num_elements << std::endl; 

    //     std::cout << "Variable type: ";

    //     switch (variable->class_type) {
    //         case MAT_C_DOUBLE:
    //             std::cout << "double" << std::endl;
    //             break;
    //         case MAT_C_SINGLE:
    //             std::cout << "single" << std::endl;
    //             break;
    //         case MAT_C_INT32:
    //             std::cout << "int32" << std::endl;
    //             break;
    //         case MAT_C_UINT8:
    //             std::cout << "uint8" << std::endl;
    //             break;
    //         case MAT_C_CHAR:
    //             std::cout << "char" << std::endl;
    //             break;
    //         case MAT_C_STRUCT:
    //             std::cout << "struct" << std::endl;
    //             break;
    //         case MAT_C_CELL:
    //             std::cout << "cell" << std::endl;
    //             break;
    //         // Add more cases as needed
    //         default:
    //             std::cout << "unknown" << std::endl;
    //             break;
    //     }

    //     Mat_VarFree(variable);
    //     variable = Mat_VarReadNext(matfp);
    // }

    // Mat_Close(matfp);