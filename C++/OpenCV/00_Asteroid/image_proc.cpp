////////// INIT ////////////////////////////////////////////////////////////////////////////////////

///// Includes /////
#include "SfM.hpp"

////////// IMAGE PROCESSING ////////////////////////////////////////////////////////////////////////

///// Image Loading ///////////////////////////////////////////////////////

void fetch_images_at_path( string path, vector<string>& fNames, vector<Mat>& images, 
                           uint limit, string ext ){
    // Load all the images found at a path
    fNames.clear();
    images.clear();
    fNames = list_files_at_path_w_ext( path, ext, true );
    uint /*-----*/ Nimg   = fNames.size();
    Mat /*------*/ img;  
    uint /*-----*/ i = 0;
    cout << "Found: " << Nimg << " files ... " << endl;
    for( string fName : fNames ){
        if( (limit > 0) && (i > (limit-1)) ) break;
        cout << "Found: " << fName << " ... " << flush;
        img = imread( fName, IMREAD_GRAYSCALE );
        images.push_back( img );
        cout << "Loaded: " << fName << ", Bytes: " << (img.total() * img.elemSize()) << endl;
        i++;
    }
    cout << endl << "Got " << images.size() << " images!" << endl;
}

///// KAZE ////////////////////////////////////////////////////////////////

KAZE::KAZE(){  akaze = AKAZE::create();  }

void KAZE::get_KAZE_keypoints( const Mat& img, vector<KeyPoint>& kptsOut, Mat& descOut ){
    akaze->detectAndCompute( img, cv::noArray(), kptsOut, descOut, false );
}



////////// CAMERA MODEL ////////////////////////////////////////////////////////////////////////////

CameraInstrinsics::CameraInstrinsics( float f_x, float f_y, float x_0, float y_0, float r_x, float r_y  ){
    // Return the idealized intrinsic matrix
    K = Mat::eye( 3, 3, CV_32F );
    K.at<float>(0, 0) = f_x; 
    K.at<float>(1, 1) = f_y; 
    K.at<float>(0, 2) = x_0; 
    K.at<float>(1, 2) = y_0; 
    F = f_x == f_y ? f_x : nan("");
    R = r_x == r_y ? r_x : nan("");
}