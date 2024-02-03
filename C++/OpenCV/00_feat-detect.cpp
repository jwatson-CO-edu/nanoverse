// cls && g++ 00_feat-detect.cpp `pkg-config --cflags --libs opencv4` -std=c++17 -lopencv_xfeatures2d -I /usr/local/include/opencv4/

// cls && g++ `pkg-config --cflags --libs opencv4` -std=c++17 00_feat-detect.cpp -I /usr/local/include/opencv4/

// cls && g++ 00_feat-detect.cpp -std=c++17 -I /usr/local/include/opencv4/ -lopencv_core -lopencv_highgui -lopencv_features2d -lopencv_xfeatures2d -lopencv_imgcodecs -o load-img.out
// cls && g++ 00_feat-detect.cpp -std=c++17 -I /usr/local/include/opencv4/ -o load_images.out `pkg-config --cflags --libs opencv4`

// https://docs.opencv.org/3.4/d7/d66/tutorial_feature_detection.html

#include <iostream>
using std::cout, std::endl;
#include <opencv2/opencv.hpp>
#include "opencv2/core.hpp"
#include <opencv2/core/utility.hpp>
using cv::Mat, cv::String;
#include <opencv2/imgcodecs.hpp>

// #include "opencv2/core/core.hpp"
#include "opencv2/highgui.hpp"
using namespace cv::samples;
using cv::waitKey;
#include "opencv2/features2d.hpp"
using cv::Ptr, cv::KeyPoint;
#include "opencv2/xfeatures2d.hpp"
// using namespace cv::xfeatures2d;
using cv::xfeatures2d::SURF;
// using cv::xfeatures2d::SURF::create;

using cv::CommandLineParser;
using cv::imread, cv::IMREAD_GRAYSCALE;
// using namespace cv;
// #include <opencv2/opencv.hpp>


int main( int argc, char* argv[] )
{
    CommandLineParser parser( argc, argv, "{@input | box.png | input image}" );
    Mat src = imread( findFile( parser.get<String>( "@input" ) ), IMREAD_GRAYSCALE );
    if ( src.empty() )
    {
        cout << "Could not open or find the image!\n" << endl;
        cout << "Usage: " << argv[0] << " <Input image>" << endl;
        return -1;
    }
    // -- Step 1: Detect the keypoints using SURF Detector
    int minHessian = 400;
    Ptr<SURF> detector = SURF::create( minHessian );
    std::vector<KeyPoint> keypoints;
    detector->detect( src, keypoints );
    //-- Draw keypoints
    Mat img_keypoints;
    drawKeypoints( src, keypoints, img_keypoints );
    //-- Show detected (drawn) keypoints
    imshow("SURF Keypoints", img_keypoints );
    waitKey();
    return 0;
}
