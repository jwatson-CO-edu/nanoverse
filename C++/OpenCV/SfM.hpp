/*
WARNING: DO NOT WORRY ABOUT SERIALIZATION!
*/

////////// INIT ////////////////////////////////////////////////////////////////////////////////////

///// Includes ////////////////////////////////////////////////////////////

/// Standard ///
#include <cmath>
using std::nan;
#include <iostream>
using std::cout, std::endl, std::flush;
#include <vector>
using std::vector;
#include <queue>
using std::queue;
#include <string>
using std::string, std::to_string;
#include <memory>
using std::shared_ptr;
#include <filesystem>
using std::filesystem::directory_iterator;


/// Special ///
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/core/utility.hpp>
using cv::Mat, cv::String, cv::Vec, cv::Vec3f, cv::norm;
#include <opencv2/imgcodecs.hpp>
using cv::imread, cv::IMREAD_COLOR, cv::IMREAD_GRAYSCALE;
#include "opencv2/features2d.hpp"
using cv::Ptr, cv::KeyPoint, cv::Point2f, cv::FeatureDetector, cv::Feature2D, cv::AKAZE;


///// Aliases /////////////////////////////////////////////////////////////
typedef char /*----*/ byte;
typedef unsigned char ubyte;

////////////////////////////////////////////////////////////////////////////////////////////////////
////////// helpers.cpp /////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////


////////// STANDARD CONTAINERS /////////////////////////////////////////////////////////////////////

template<typename T>
bool p_vec_has_item( const vector<T>& vec, const T& item ){
    for( const T& elem : vec ) if( elem == item )  return true;
    return false;
}

template<typename T>
T get_last( vector<T>& vec ){  
    // Get the last element of a vector, if it exists, otherwise throw an index error
    size_t N = vec.size();
    if( N > 0 )
        return vec[ N-1 ];
    else
        throw std::out_of_range{ "get_last: Vector was EMPTY!" };
}

template<typename T>
T get_last( const vector<T>& vec ){  
    // Get the last element of a vector, if it exists, otherwise throw an index error
    size_t N = vec.size();
    if( N > 0 )
        return vec[ N-1 ];
    else
        throw std::out_of_range{ "get_last: Vector was EMPTY!" };
}


////////// STRING PROCESSING ///////////////////////////////////////////////////////////////////////

string /*---*/ to_upper( string input );
vector<string> split_string_on_char( string input, char ch );


////////// PATH AND FILE OPERATIONS ////////////////////////////////////////////////////////////////

bool /*-----*/ file_has_ext( string path, string ext );
vector<string> list_files_at_path( string path, bool sortAlpha = true );
vector<string> list_files_at_path_w_ext( string path, string ext, bool sortAlpha = true );


////////// TRIGONOMETRY ////////////////////////////////////////////////////////////////////////////

// Cosine and Sine in degrees
// Author: Willem A. (Vlakkies) Schreüder  
double Cos( double x );
double Sin( double x );
double Tan( double x );
float  Cosf( float x );
float  Sinf( float x );
float  Tanf( float x );
float  Atan2f( float y, float x );



////////////////////////////////////////////////////////////////////////////////////////////////////
////////// image_proc.cpp //////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////

////////// IMAGE PROCESSING ////////////////////////////////////////////////////////////////////////

void fetch_images_at_path( string path, vector<string>& fNames, vector<Mat>& images, 
                           uint limit = 0, string ext = "jpg" );

class KAZE{ public:
    Ptr<Feature2D> akaze;

    KAZE();

    void get_KAZE_keypoints( const Mat& img, vector<KeyPoint>& kptsOut, Mat& descOut );
};

class CameraInstrinsics{ public:
    Mat   K; // Intrinsic Matrix
    float F; // Focal Length
    float R; // Resolution

    CameraInstrinsics( float f_x, float f_y, float x_0, float y_0, float r_x, float r_y );
};



////////////////////////////////////////////////////////////////////////////////////////////////////
////////// SfM.cpp /////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////

////////// SCENE GRAPH /////////////////////////////////////////////////////////////////////////////

///// SG_Node /////////////////////////////////////////////////////////////

class SG_Node;
typedef shared_ptr<SG_Node> NodePtr;


class SG_Node{ public:
    // Scene Graph Node for Structure from Motion. Assumes one image per pose.

    /// Members ///
    string /*-----------*/ imPth; // Image path
    Mat /*--------------*/ image; // Image data
    vector<KeyPoint> /*-*/ kypts; // Keypoints in the (greyscale) image
    Mat /*--------------*/ kpNfo; // Keypoint Info (Not used?)
    Mat /*--------------*/ xform; // Estimated camera pose
    vector<NodePtr> /*--*/ nhbrs; // Graph neighbors
    vector<vector<size_t>> match; // Keypoint correspondences to each neighbor
    ubyte /*------------*/ visit; // Was this node expanded?

    /// Constructors ///
    SG_Node( const string fName, const Mat& img ); // Constructor from an image
};

vector<NodePtr> images_to_nodes( string path, string ext = "jpg" );


////////// TRIANGULATION ///////////////////////////////////////////////////////////////////////////