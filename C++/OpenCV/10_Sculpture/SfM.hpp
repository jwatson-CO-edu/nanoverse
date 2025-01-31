////////// INIT ////////////////////////////////////////////////////////////////////////////////////

///// Includes /////

/// Standard ///
#include <vector>
using std::vector;
#include <string>
using std::string, std::to_string, std::stof;
#include <memory>
using std::shared_ptr;

/// Special ///
#include <opencv2/opencv.hpp>
#include "opencv2/core.hpp"
#include <opencv2/core/utility.hpp>
using cv::Mat, cv::String, cv::Vec;
#include <opencv2/imgcodecs.hpp>
using cv::imread, cv::IMREAD_COLOR, cv::IMREAD_GRAYSCALE;
#include "opencv2/features2d.hpp"
using cv::Ptr, cv::KeyPoint, cv::Point2f, cv::FeatureDetector;



////////////////////////////////////////////////////////////////////////////////////////////////////
////////// Structure.cpp ///////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////

class ImgNode{ public:
    // Represents an image and all of the information inferred from that image
    
    /// Members ///
    string /*-----*/ imgPth; // Image path
    Mat /*--------*/ image; //- Image data
    vector<KeyPoint> keyPts; // Keypoints for image
    nodePtr /*----*/ prev; // - Parent `ImgNode`
    nodePtr /*----*/ next; // - Successor `ImgNode`s
    Mat /*--------*/ xform; //- Transform relative to `prev` node
    
};
typedef shared_ptr<ImgNode> nodePtr;