////////// INIT ////////////////////////////////////////////////////////////////////////////////////

///// Includes /////

/// Standard ///
#include <iostream>
using std::cout, std::cerr, std::endl, std::flush;
#include <vector>
using std::vector;
#include <string>
using std::string, std::to_string, std::stof;
#include <memory>
using std::shared_ptr;
#include <filesystem>
using std::filesystem::directory_iterator;

/// Special ///
#include <opencv2/opencv.hpp>
#include "opencv2/core.hpp"
#include <opencv2/core/utility.hpp>
using cv::Mat, cv::String, cv::Vec;
#include <opencv2/imgcodecs.hpp>
using cv::imread, cv::IMREAD_COLOR, cv::IMREAD_GRAYSCALE;
#include "opencv2/features2d.hpp"
using cv::Ptr, cv::KeyPoint, cv::Point2f, cv::FeatureDetector, cv::Feature2D, cv::AKAZE;


////////////////////////////////////////////////////////////////////////////////////////////////////
////////// Image.cpp ///////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////

////////// KAZE ////////////////////////////////////////////////////////////////////////////////////

class KAZE{ public:
    Ptr<Feature2D> akaze;

    KAZE();

    void get_KAZE_keypoints( const Mat& img, vector<KeyPoint>& kptsOut, Mat& descOut );
};


////////// STRING PROCESSING ///////////////////////////////////////////////////////////////////////
string to_upper( string input ); // Return a version of the string that is upper case
// Return a vector of strings found in `input` separated by whitespace
vector<string> split_string_on_char( string input, char ch ); 


////////// FILE OPERATIONS /////////////////////////////////////////////////////////////////////////
bool file_has_ext( string path, string ext ); // Return true if a `path` has `ext`
vector<string> list_files_at_path( string path, bool sortAlpha ); // List all the files found at a path
// List all paths under `path` with the given `ext`
vector<string> list_files_at_path_w_ext( string path, string ext, bool sortAlpha );
// Load all the images found at a path
void fetch_images_at_path( string path, vector<string>& fNames, vector<Mat>& images, 
                           uint limit = 0, string ext = "jpg" );
    
    

////////////////////////////////////////////////////////////////////////////////////////////////////
////////// Structure.cpp ///////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////

class ImgNode;
typedef shared_ptr<ImgNode> NodePtr;

class ImgNode{ public:
    // Represents an image and all of the information inferred from that image
    // NOTE: This class assumes that the pictures are taken in a sequence, Doubly-Linked List
    
    /// Members ///
    string /*-----*/ imgPth; // Image path
    Mat /*--------*/ image; //- Image data
    vector<KeyPoint> keyPts; // Keypoints for image
    NodePtr /*----*/ prev; // - Parent `ImgNode`
    NodePtr /*----*/ next; // - Successor `ImgNode`s
    Mat /*--------*/ xform; //- Transform relative to `prev` node

    ImgNode( string path, const Mat& sourceImg );
    
};


vector<NodePtr> images_to_nodes( string path, string ext ); // Populate a vector of nodes with paths and images