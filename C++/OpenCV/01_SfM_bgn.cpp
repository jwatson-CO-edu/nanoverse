// g++ 01_SfM_bgn.cpp `pkg-config --cflags --libs opencv4` -std=c++17 -lopencv_xfeatures2d -I /usr/local/include/opencv4/ -o load-kp.out

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
            [Y] Brute force, 2024-02-08: Not the longest wait ever
            [ ] Approximate nearest neighbour library, such as FLANN, ANN, Nanoflann
            https://docs.opencv.org/3.4/d5/d6f/tutorial_feature_flann_matcher.html
        [>] Bi-directional verification
            [Y] Brute force, 2024-02-08: Have to wait quite a while to get mutual matches from brute force,
                5321 matched keypoint pairs for two images with 50000 features each.
            [ ] Approximate nearest neighbour
    [>] Relative pose estimation
        [ ] For each pair of images with sufficient number of matches, compute relative pose
            * 2024-02-08: For now, assume that pictures with consecutive alpha filenames are related by virtue
                          of being taken in burst fashion while orbiting the subject. Close the loop. No search needed.
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
using std::abs, std::min;

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

/// Local ///
#include "helpers.hpp"


////////// UTILITY FUNCTIONS ///////////////////////////////////////////////////////////////////////

vector<string> list_files_at_path( string path, bool sortAlpha = true ){
    // List all the files found at a path
    vector<string> rtnNams;
    string /*---*/ path_i;
    for (const auto & entry : directory_iterator( path ) ){  
        path_i = entry.path().string();
        rtnNams.push_back( path_i );  
    }
    if( sortAlpha )  std::sort( rtnNams.begin(),rtnNams.end() );
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

float keypoint_dist_sqr( const KeyPoint& a, const KeyPoint& b ){
    // Return the squared L2 norm between two points in pixels squared
    return pow(a.pt.x - b.pt.x, 2) + pow(a.pt.y - b.pt.y, 2);
}

float keypoint_diff_mag( const KeyPoint& a, const KeyPoint& b, float areaFactor = 50.0f, float angleFactor = 50.0f/M_PI ){
    // Calculate keypoint difference as the angle diff scaled by fraction of overlapping area
    //     <Fraction of non-overlapping area>*<constant> + <Angle difference>*<constant>
    return keypoint_dist_sqr( a, b ) + 
           (1.0f - KeyPoint::overlap( a, b ))*areaFactor + 
           abs( a.angle - b.angle )*angleFactor;
}

size_t N_KM = 0;

struct KpMatch{
    // Matching keypoints for use in `ShotPair`
    size_t ID;
    size_t prevDex;
    size_t nextDex;
    float  diff;

    KpMatch( size_t prevDex_, size_t nextDex_, float diff_ ){
        // Assign a unique ID and set values
        ID = N_KM;
        prevDex = prevDex_;
        nextDex = nextDex_;
        diff    = diff_;
        ++N_KM;
    }

    bool p_same_indices( const KpMatch& other ){
        // Return true if both matches contain the same indices
        return ( ((prevDex == other.prevDex) && (nextDex == other.nextDex))
                 ||
                 ((prevDex == other.nextDex) && (nextDex == other.prevDex)) );
    }
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

    void brute_force_match( ulong topK ){
        // Linear search for best matches, O(n^2)
        KeyPoint /*--*/ kp_p, kp_n;
        size_t /*----*/ iM = 0, jM = 0;
        size_t /*----*/ Np = prev->kpts.size();
        size_t /*----*/ Nn = next->kpts.size();
        float /*-----*/ dMin, d;
        size_t /*----*/ div = 1000;
        vector<KpMatch> PtoN;
        vector<KpMatch> NtoP;

        // TBD: Eliminate one-to-many matches?
        // vector<KpMatch> candidates;
        // vector<size_t>  delVec;
        // bool /*------*/ oneID = false;

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
            PtoN.push_back( KpMatch{iM,jM,dMin} );
            if( (ip%div) == 0 ) cout << "." << flush;
        }
        cout << " Complete! Find reverse matches " << flush;

        for( size_t jn = 0; jn < Nn; ++jn ){
            dMin = 1e9;
            kp_n = next->kpts[jn];
            for( size_t ip = 0; ip < Np; ++ip ){
                kp_p = prev->kpts[ip];    
                d    = keypoint_diff_mag( kp_p, kp_n );
                if( d < dMin ){
                    dMin = d;
                    iM   = ip;
                    jM   = jn;
                }
            }
            NtoP.push_back( KpMatch{iM,jM,dMin} );
            if( (jn%div) == 0 ) cout << "." << flush;
        }
        cout << " Complete! Filter for mutual matches " << flush;

        // 2024-02-07: For now, allow one-to-many matching and assume the best matches will win ...
        // FIXME: CHECK IF BI-DIRECTIONAL VERIFICATION ELIMINATES ONE-TO-MANY CORRESPONDENCE!

        std::sort( PtoN.begin(), PtoN.end(), [](KpMatch &a, KpMatch &b){ return a.diff < b.diff; });
        std::sort( NtoP.begin(), NtoP.end(), [](KpMatch &a, KpMatch &b){ return a.diff < b.diff; });

        size_t K = min( min( topK, PtoN.size() ), NtoP.size() );
        
        /* To make the matching results more robust, a bi-directional verification is performed, 
           which requires that matches in each direction need to be among the top K matching points 
           of one another. */
        for( size_t i = 0; i < K; ++i ){
            for( size_t j = 0; j < K; ++j ){
                if( PtoN[i].p_same_indices( NtoP[j] ) ){
                    matches.push_back( PtoN[i] );
                    break;
                }
                if( (j%div) == 0 ) cout << "." << flush;
            }
        }
        cout << " Complete!" << endl;
    }

    void report_matches( size_t limit = 0 ){
        // Print out matches and their closeness
        cout << "There are " << matches.size() << " matched keypoint pairs." << endl;
        size_t i = 0;
        for( KpMatch& match : matches ){ 
            cout << "\t" << match.ID << ": " << match.prevDex << "<->" << match.nextDex << " = " << match.diff << endl;
            ++i;
            if( (i>0) && (i>=limit) ) break;
        }
        cout << endl;
    }
};


vector<shotPtr> shots_from_images( const vector<string>& paths, const vector<Mat>& images, ulong Nfeat = 10000 ){
    // Construct a camera shot object for every photo in the vector
    vector<shotPtr> /*----*/ rtnShots;
    cout << "About to get keypoints ... " << flush;
    vector<vector<KeyPoint>> kps = calc_ORB_keypoints_from_images( images, Nfeat );
    cout << "Obtained!" << endl;
    size_t /*-------------*/ N   = images.size();
    for( size_t i = 0; i < N; ++i ){
        rtnShots.push_back( shotPtr( new CamShot( paths[i], images[i], kps[i] ) ) );
    }
    return rtnShots;
}

////////// MAIN ////////////////////////////////////////////////////////////////////////////////////
int main(){

    ulong Nfeat = 50000;

    vector<string> fNames = list_files_at_path( _IMG_PATH );
    for( string fName : fNames ) cout << fName << endl;
    cout << endl;
    vector<Mat> images = fetch_images_at_path( _IMG_PATH, 2 );
    cout << endl;
    vector<shotPtr> shots = shots_from_images( fNames, images, Nfeat );
    cout << endl;
    for( shotPtr& shot : shots ){
        cout << shot->path << ", KP: " << shot->kpts.size() << endl;
    }

    ShotPair sp{ shots[0], shots[1] };
    sp.brute_force_match( (ulong)(Nfeat/2) );
    sp.report_matches( 100 );

    return 0;
}