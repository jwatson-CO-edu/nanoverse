// g++ 01_SfM_bgn.cpp `pkg-config --cflags --libs opencv4` -std=c++17 -lopencv_xfeatures2d -I /usr/local/include/opencv4/ -o load-img.out

////////// DEV PLAN & NOTES ////////////////////////////////////////////////////////////////////////
/*
##### Sources #####
* https://imkaywu.github.io/tutorials/sfm/


##### DEV PLAN #####
[Y] Load images in a dir, 2024-02-02: Req'd `pkg-config`
[Y] 00 SURF Example, 2024-02-02: Finally, Finally, Finally
[>] ORB Example, Compute ORB features for one image
[>] Basic SfM Tutorial: https://imkaywu.github.io/tutorials/sfm/
    [Y] Compute ORB for all images, 2024-02-02: Easy!
    [>] Feature matching
        [>] Match features between each pair of images in the input image set,
            https://docs.opencv.org/3.4/d5/d6f/tutorial_feature_flann_matcher.html
            [>] Brute force
            [ ] Approximate nearest neighbour library, such as FLANN, ANN, Nanoflann
        [ ] Bi-directional verification
    [ ] Relative pose estimation
        [ ] For each pair of images with sufficient number of matches, compute relative pose
        [ ] N-view triangulation
        [ ] Bundle adjustment
    [ ] Two-view SfM
        [ ] Render PC
    [ ] N-view SfM
        [ ] iterative step that merges multiple graphs into a global graph, for graph in graphs
            [ ] pick graph g in graphs having maximum number of common tracks with the global graph
            [ ] merge tracks of global graph with those of graph g
            [ ] triangulate tracks
            [ ] perform bundle adjustment
            [ ] remove track outliers based on baseline angle and reprojection error
            [ ] perform bundle adjustment
        [ ] Render PC 
            [ ] Point Cloud
            [ ] Camera poses
            [ ] Fly around
[ ] Advanced Textured SfM
    [ ] SfM PC
    [ ] Delaunay at each camera view
    [ ] Consolidate triangulations
    [ ] Project textures at each camera view
    [ ] Consolidate textures @ facet objects
    [ ] Write individual textures to a single image file
    [ ] OBJ Output
    [ ] Render OBJ (or other appropriate format)
[ ] Virtual Sculpture Garden (Dangerous!)
    [ ] Place sculptures tastefully
    [ ] Add FPV
    [ ] Post walkthrough to IG
*/


////////// INIT ////////////////////////////////////////////////////////////////////////////////////

///// Includes /////

/// Standard ///
#include <vector>
using std::vector;
#include <string>
using std::string;
#include <filesystem>
using std::filesystem::directory_iterator;
#include <iostream>
using std::cout, std::endl, std::flush;
#include <memory>
using std::shared_ptr;
#include <map>
using std::pair;
#include <cmath>
using std::abs;

/// Special ///
#include <opencv2/opencv.hpp>
#include "opencv2/core.hpp"
using cv::CommandLineParser;
#include <opencv2/core/utility.hpp>
using cv::Mat, cv::String;
#include <opencv2/imgcodecs.hpp>
using cv::imread, cv::IMREAD_COLOR, cv::IMREAD_GRAYSCALE;
#include "opencv2/highgui.hpp"
using namespace cv::samples;
using cv::waitKey;
#include "opencv2/features2d.hpp"
using cv::Ptr, cv::KeyPoint, cv::Point, cv::FeatureDetector;
#include "opencv2/xfeatures2d.hpp"
using cv::ORB, cv::normL2Sqr;



////////// UTILITY FUNCTIONS ///////////////////////////////////////////////////////////////////////

vector<string> list_files_at_path( string path ){
    // List all the files found at a path
    vector<string> rtnNams;
    string /*---*/ path_i;
    for (const auto & entry : directory_iterator( path ) ){  
        path_i = entry.path().string();
        rtnNams.push_back( path_i );  
    }
    return rtnNams;
}



////////// GLOBALS /////////////////////////////////////////////////////////////////////////////////
string _IMG_PATH = "data/SfM/00_sculpture";



////////// IMAGE PROCESSING ////////////////////////////////////////////////////////////////////////


vector<Mat> fetch_images_at_path( string path, uint limit = 0 ){
    // Load all the images found at a path
    vector<string> fNames = list_files_at_path( path );
    uint /*-----*/ Nimg   = fNames.size();
    Mat /*------*/ img;  
    vector<Mat>    images;
    uint /*-----*/ i = 0;
    for( string fName : fNames ){
        if( (limit > 0) && (i > (limit-1)) ) break;
        img = imread( fName, IMREAD_GRAYSCALE );
        images.push_back( img );
        cout << "Loaded: " << fName << ", Bytes: " << (img.total() * img.elemSize()) << endl;
        i++;
    }
    cout << endl << "Got " << images.size() << " images!" << endl;
    return images;
}

vector<vector<KeyPoint>> calc_ORB_keypoints_from_images( const vector<Mat>& images, ulong N = 10000 ){
    // Caclulate keypoints for every image in the input vector
    vector<vector<KeyPoint>> rtnKps;
    vector<KeyPoint> /*---*/ kp;
    Ptr<FeatureDetector>     detector = ORB::create( N );
    cout << "\t";
    for( const Mat& image : images ){
        detector->detect( image, kp );
        rtnKps.push_back( kp );
        cout << "." << flush;
    }
    cout << endl;
    return rtnKps;
}

////////// KEYPOINT DETECTION & CORRELATION ////////////////////////////////////////////////////////


class CamShot{ public:
    // Represents an image and all of the information inferred from that image
    
    /// Members ///
    string /*-----*/ path;
    Mat /*--------*/ imag;
    vector<KeyPoint> kpts;
    // FIXME: ESTIMATED POSE

    /// Constructors ///
    CamShot( string fPath, const Mat& image, const vector<KeyPoint>& keypoints ){
        path = fPath;
        imag = Mat( image );
        kpts = keypoints;
        // FIXME: INIT ESTIMATED POSE
    };
};
typedef shared_ptr<CamShot> shotPtr;

float keypoint_diff_mag( const KeyPoint& a, const KeyPoint& b ){
    // Calculate keypoint difference as the angle diff scaled by fraction of overlapping area
    return KeyPoint::overlap( a, b ) * abs( a.angle - b.angle );
}

struct KpMatch{
    // Matching keypoints for use in `ShotPair`
    size_t prevDex;
    size_t nextDex;
    float  diff;

    bool p_has_i( size_t i ){  return ((prevDex==i)||(nextDex==i));  } // Does this involve index `i`?
};

class ShotPair{ public:
    // Represents correspondences between two `CamShot`s

    /// Members ///
    shotPtr /*---*/ prev = nullptr;
    shotPtr /*---*/ next = nullptr;
    vector<KpMatch> matches;

    /// Constructor(s) ///

    ShotPair( shotPtr& p, shotPtr& n ){
        prev = shotPtr( p );
        next = shotPtr( n );
    }

    /// Methods ///

    void brute_force_match(){
        // Linear search for best matches, O(n^2)
        KeyPoint kp_p, kp_n;
        size_t   iM = 0, jM = 0;
        size_t   Np = prev->kpts.size();
        size_t   Nn = next->kpts.size();
        float    dMin, d;
        size_t   div = 1000;

        matches.clear();

        cout << "Brute Force Keypoint Matching ";

        for( size_t ip = 0; ip < Np; ++ip ){
            dMin = 1e9;
            kp_p = prev->kpts[ip];
            for( size_t jn = 0; jn < Nn; ++jn ){
                kp_n = next->kpts[jn];
                d    = keypoint_diff_mag( kp_p, kp_n );
                if( d < dMin ){
                    dMin = d;
                    iM   = ip;
                    jM   = jn;
                }
            }
            matches.push_back( KpMatch{iM,jM,dMin} );
            if( (ip%div) == 0 ) cout << "." << flush;
        }
        cout << endl;
        // FIXME: MAKE MATCHES UNIQUE (KEEP BEST EDGE OUT OF ONE-TO-MANY GRAPHS)
        for( size_t ip = 0; ip < Np; ++ip ){

        }
    }
};


vector<shotPtr> shots_from_images( const vector<string>& paths, const vector<Mat>& images ){
    // Construct a camera shot object for every photo in the vector
    vector<shotPtr> /*----*/ rtnShots;
    cout << "About to get keypoints ... " << flush;
    vector<vector<KeyPoint>> kps = calc_ORB_keypoints_from_images( images );
    cout << "Obtained!" << endl;
    size_t /*-------------*/ N   = images.size();
    for( size_t i = 0; i < N; ++i ){
        rtnShots.push_back( shotPtr( new CamShot( paths[i], images[i], kps[i] ) ) );
    }
    return rtnShots;
}

////////// MAIN ////////////////////////////////////////////////////////////////////////////////////
int main(){

    vector<string> fNames = list_files_at_path( _IMG_PATH );
    for( string fName : fNames ) cout << fName << endl;
    cout << endl;
    vector<Mat> images = fetch_images_at_path( _IMG_PATH );
    cout << endl;
    vector<shotPtr> shots = shots_from_images( fNames, images );
    cout << endl;
    for( shotPtr& shot : shots ){
        cout << shot->path << ", KP: " << shot->kpts.size() << endl;
    }
    return 0;
}