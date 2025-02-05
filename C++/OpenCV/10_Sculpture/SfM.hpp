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
#include <sys/stat.h>
#include <fstream>
using std::ifstream, std::ofstream;

/// Special ///
#include <opencv2/opencv.hpp>
#include "opencv2/core.hpp"
#include <opencv2/core/utility.hpp>
using cv::Mat, cv::String, cv::Vec, cv::Ptr, cv::Range, cv::Size, cv::imshow, cv::waitKey;
#include <opencv2/imgcodecs.hpp>
using cv::imread, cv::IMREAD_COLOR, cv::IMREAD_GRAYSCALE;
#include "opencv2/features2d.hpp"
using cv::Ptr, cv::KeyPoint, cv::Point2f, cv::Point2i, cv::Point3f, cv::DMatch, 
      cv::FeatureDetector, cv::Feature2D, cv::AKAZE, cv::DescriptorMatcher;
using std::invalid_argument; // Who included `<stdexcept>`?


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
bool file_exists( const string& fName ); // Return true if the file exists , otherwise return false
vector<string> read_lines( string path ); // Return all the lines of text file as a string vector
string get_line_arg( string line ); // Get everything after the last ':' in a string

////////// UTILITY FUNCTIONS ///////////////////////////////////////////////////////////////////////

// Deserialize an OpenCV `CV_32F` matrix stored row-major in a comma-separated list in a string
Mat deserialize_2d_Mat_f( string input, int Mrows, int Ncols, char sep = ',' );

    

////////////////////////////////////////////////////////////////////////////////////////////////////
////////// Structure.cpp ///////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////

////////// RELATIVE CAMERA POSE ////////////////////////////////////////////////////////////////////

Mat load_cam_calibration( string cPath ); // Fetch the K matrix stored as plain text

class CamData{ public:
    Mat     Kintrinsic;
    Point2i imgSize;
    float   horzFOV_deg; // 65.0deg // https://www.camerafv5.com/devices/manufacturers/motorola/moto_g_power_sofia_0/
    float   vertFOV_deg; // 51.1deg // https://www.camerafv5.com/devices/manufacturers/motorola/moto_g_power_sofia_0/

    CamData( string kPath, string imgDir, float horzFOV_ = 65.0f, float vertFOV_ = 51.1f, string ext = "jpg" );
};


struct TwoViewResult{
    // Contains info that can be computed by two (consecutive) views
    // Source: https://claude.ai/chat/b55bb623-d26f-42fe-9ece-c7fd3477e0ac
    
    /// Stage 1 ///
    Mat R;  // Rotation matrix
    Mat t;  // Translation vector
    vector<Point2f> matched_points1;
    vector<Point2f> matched_points2;
    vector<DMatch>  good_matches;
    bool /*------*/ success;

    /// Stage 2 ///
    vector<Point3f> PCD;
};


class TwoViewCalculator{ public:
    // Uses the keypoints of two images to estimate a relative camera pose between them
    // Source: https://claude.ai/chat/b55bb623-d26f-42fe-9ece-c7fd3477e0ac

    // Camera intrinsic parameters - these would typically come from calibration

    // Matcher object
    Ptr<DescriptorMatcher> matcher;
    
    // Parameters for feature detection and matching
    float ratioThresh;
    float ransacThresh;
    float confidence;

    TwoViewCalculator( float ratioThresh_ = 0.7f, float ransacThresh_ = 2.5f, float confidence_ = 0.99f );

    TwoViewResult estimate_pose( const CamData& camInfo,
                              const vector<KeyPoint>& keypoints1, const Mat& descriptors1, 
                              const vector<KeyPoint>& keypoints2, const Mat& descriptors2 );

    vector<Point3f> generate_point_cloud( const vector<Point2f>& keypoints1, const vector<Point2f>& keypoints2,
                                          const Mat& R, const Mat& t, const CamData& camInfo );

    static Mat visualize_matches( const Mat& img1, const vector<KeyPoint>& keypoints1,
                                  const Mat& img2, const vector<KeyPoint>& keypoints2,
                                  const TwoViewResult& result );
};



////////// POSE GRAPH //////////////////////////////////////////////////////////////////////////////

class ImgNode;
typedef shared_ptr<ImgNode> NodePtr;

class ImgNode{ public:
    // Represents an image and all of the information inferred from that image
    // NOTE: This class assumes that the pictures are taken in a sequence, Doubly-Linked List
    
    /// Members ///
    string /*-----*/ imgPth; // - Image path
    Mat /*--------*/ image; // -- Image data
    Point2i /*----*/ imgSize; //- Image size [px]
    vector<KeyPoint> keyPts; // - Keypoints for image
    Mat /*--------*/ kpDesc; // - Keypoint descriptors, needed for matching
    TwoViewResult /*-*/ kpRes; // -- Result of keypoint matching
    NodePtr /*----*/ prev; // --- Parent `ImgNode`
    NodePtr /*----*/ next; // --- Successor `ImgNode`s
    Mat /*--------*/ relXform; // Transform relative to `prev` node
    Mat /*--------*/ absXform; // Camera pose for this image in the lab frame

    ImgNode( string path, const Mat& sourceImg );
    
};


vector<NodePtr> images_to_nodes( string path, string ext = "jpg" ); // Populate a vector of nodes with paths and images


