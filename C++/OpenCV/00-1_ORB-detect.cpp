// cls && g++ 00-1_ORB-detect.cpp `pkg-config --cflags --libs opencv4` -std=c++17 -lopencv_xfeatures2d -I /usr/local/include/opencv4/
// 

// https://docs.opencv.org/3.4/d7/d66/tutorial_feature_detection.html

#include <iostream>
using std::cout, std::endl;
#include <opencv2/opencv.hpp>
#include "opencv2/core.hpp"
#include <opencv2/core/utility.hpp>
using cv::Mat, cv::String;
using cv::CommandLineParser;
#include <opencv2/imgcodecs.hpp>
using cv::imread, cv::IMREAD_GRAYSCALE, cv::IMREAD_COLOR;
#include "opencv2/highgui.hpp"
using namespace cv::samples;
using cv::waitKey;
#include "opencv2/features2d.hpp"
using cv::Ptr, cv::KeyPoint, cv::FeatureDetector;
#include "opencv2/xfeatures2d.hpp"
using cv::ORB;


int main( int argc, char* argv[] ){

    // -- Step -1: Init
    CommandLineParser parser( argc, argv, "{@input1||}{@input2||}" );
    Mat /*---------*/ src1 = imread( findFile( parser.get<String>( "@input1" ) ), IMREAD_GRAYSCALE );
    Mat /*---------*/ src2 = imread( findFile( parser.get<String>( "@input2" ) ), IMREAD_GRAYSCALE );

    // -- Step 0: Check image non-empty
    if ( src1.empty() || src2.empty() ){
        cout << "Could not open or find the images!\n" << endl;
        cout << parser.get<String>( "@input1" ) << endl;
        cout << parser.get<String>( "@input2" ) << endl;
        cout << "Usage: " << argv[0] << " <Input image>" << endl;
        return -1;
    }

    // -- Step 1: Detect the keypoints using SURF Detector
    // int /*-------------*/ minHessian = 400;
    // Ptr<SURF> /*-------*/ detector   = SURF::create( minHessian );
    Ptr<FeatureDetector>  detector = ORB::create( 1000000 ); //,cv::ORB::FAST_SCORE);
    std::vector<KeyPoint> keypoints1;
    std::vector<KeyPoint> keypoints2;
    detector->detect( src1, keypoints1 );
    detector->detect( src2, keypoints2 );

    // -- Step 2: Draw keypoints
    Mat img_keypoints1;
    Mat img_keypoints2;
    drawKeypoints( src1, keypoints1, img_keypoints1 );
    drawKeypoints( src2, keypoints2, img_keypoints2 );

    // -- Step 3: Show detected (drawn) keypoints
    imshow("SURF Keypoints 1", img_keypoints1 );
    waitKey();
    imshow("SURF Keypoints 2", img_keypoints2 );
    waitKey();
    
    return 0;
}
