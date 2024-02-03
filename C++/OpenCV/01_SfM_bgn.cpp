// g++ 01_SfM_bgn.cpp -std=c++17 -o load-img.out `pkg-config --cflags --libs opencv4`
// g++ 01_SfM_bgn.cpp -std=c++17 -I /usr/local/include/opencv4/ -o load_images.out `pkg-config --cflags --libs opencv4`
// g++ 01_SfM_bgn.cpp -std=c++17 -o load_images.out

////////// DEV PLAN & NOTES ////////////////////////////////////////////////////////////////////////
/*
##### Sources #####
* https://imkaywu.github.io/tutorials/sfm/


##### DEV PLAN #####
[Y] Load images in a dir, 2024-02-02: Req'd `pkg-config`
[Y] 00 SURF Example, 2024-02-02: Finally, Finally, Finally
[>] ORB Example, Compute ORB features for one image
[>] Basic SfM Tutorial: https://imkaywu.github.io/tutorials/sfm/
    [>] Compute ORB for all images
    [ ] Feature matching
        [ ] Match features between each pair of images in the input image set,
            https://docs.opencv.org/3.4/d5/d6f/tutorial_feature_flann_matcher.html
            [ ] Brute force
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
using cv::Ptr, cv::KeyPoint;
#include "opencv2/xfeatures2d.hpp"
using cv::xfeatures2d::ORB;



////////// UTILITY FUNCTIONS ///////////////////////////////////////////////////////////////////////

vector<string> list_files_at_path( string path, bool prepend = false ){
    // List all the files found at a path
    vector<string> rtnNams;
    string /*---*/ path_i;
    for (const auto & entry : directory_iterator( path ) ){  
        if( prepend )
            path_i = path + "/" + entry.path().string();
        else
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
    vector<string> fNames = list_files_at_path( path, true );
    uint /*-----*/ Nimg   = fNames.size();
    vector<Mat>    images;
    uint /*-----*/ i = 0;
    for( string fName : fNames ){
        if( (limit > 0) && (i > (limit-1)) ) break;
        images.push_back( imread( fName, IMREAD_COLOR ) );
        cout << "Loaded: " << fName << endl;
        i++;
    }
    cout << endl << "Got " << images.size() << " images!" << endl;
    return images;
}

vector<vector<KeyPoint>> calc_keypoints_from_images( const vector<Mat>& images ){
    // Caclulate keypoints for every image in the input vector
    vector<vector<KeyPoint>> rtnKps;
    vector<KeyPoint> /*---*/ kp;
    Ptr<FeatureDetector>     detector = ORB::create();
    for( Mat& image : images ){
        detector->detect( image, kp );
        rtnKps.push_back( kp );
    }
    return rtnKps;
}

////////// KEYPOINT DETECTION & CORRELATION ////////////////////////////////////////////////////////

class CamShot{ public:
    // Represents an image and all of the information inferred from that image
    
    /// Members ///
    Mat /*--------*/ img;
    vector<KeyPoint> kps;
    // FIXME: ESTIMATED POSE

    /// Constructors ///
    CamShot( const Mat& image, const vector<KeyPoint>& keypoints ){
        img = Mat( image );
        kps = keypoints;
        // FIXME: INIT ESTIMATED POSE
    };
};
typedef shared_ptr<CamShot> shotPtr;



////////// MAIN ////////////////////////////////////////////////////////////////////////////////////
int main(){

    vector<string> fNames = list_files_at_path( _IMG_PATH );
    for( string fName : fNames ) cout << fName << endl;
    cout << endl;
    vector<Mat> images = fetch_images_at_path( _IMG_PATH, 10 );
    return 0;
}