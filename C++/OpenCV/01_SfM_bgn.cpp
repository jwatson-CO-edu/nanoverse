// g++ 01_SfM_bgn.cpp `pkg-config --cflags --libs opencv4` -std=c++17 -lopencv_xfeatures2d -I /usr/local/include/opencv4/ -o save-kp.out

////////// DEV PLAN & NOTES ////////////////////////////////////////////////////////////////////////
/*
##### Sources #####
* Image Features
    - https://docs.opencv.org/4.x/db/d70/tutorial_akaze_matching.html
* Structure from Motion
    - https://imkaywu.github.io/tutorials/sfm/
    - https://docs.opencv.org/4.9.0/d9/d0c/group__calib3d.html#ga59b0d57f46f8677fb5904294a23d404a
    - https://en.wikipedia.org/wiki/Bundle_adjustment


##### DEV PLAN #####
[Y] Load images in a dir, 2024-02-02: Req'd `pkg-config`
[Y] 00 SURF Example, 2024-02-02: Finally, Finally, Finally
[Y] ORB Example, Compute ORB features for one image, 2024-02-0X: Can specify how many. More might offer 
    greater match opportunities for matches, but take longer to compute correspondence. Do not forget 
    the AKAZE alternative: https://docs.opencv.org/4.x/db/d70/tutorial_akaze_matching.html
[>] Basic SfM Tutorial: https://imkaywu.github.io/tutorials/sfm/
    [Y] Compute ORB for all images, 2024-02-02: Easy!
    [P] Feature matching
        [P] Match features between each pair of images in the input image set,
            [Y] Brute force, 2024-02-08: Not the longest wait ever
            [P] Approximate nearest neighbour library, such as FLANN, ANN, Nanoflann
            https://docs.opencv.org/3.4/d5/d6f/tutorial_feature_flann_matcher.html
        [>] Bi-directional verification
            [Y] Brute force, 2024-02-08: Have to wait quite a while to get mutual matches from brute force,
                5321 matched keypoint pairs for two images with 50000 features each.
            [P] Approximate nearest neighbour
    [>] Relative pose estimation
        * Extrinsic and Intrinsic: 
            - The extrinsic parameters of a camera depend on its location and orientation and have nothing to do with 
              its internal parameters such as focal length, the field of view, etc. 
            - The intrinsic parameters of a camera depend on how it captures the images. 
              Parameters such as focal length, aperture, field-of-view, resolution, etc 
              govern the intrinsic matrix of a camera model.
        * K: Instrinsic Matrix, from calibration
        * F: Fundamental Matrix, 7DOF
            - The Fundamental matrix can be estimated using the 7-point algorithm + RANSAC
        * E: Essential Matrix, E = (K^T)FK
        * The fundamental matrix F is a generalization of the essential matrix E
        * One way to get a 3D position from a pair of matching points from two images is to take the fundamental matrix, 
          compute the essential matrix, and then to get the rotation and translation between the cameras from the 
          essential matrix. This, of course, assumes that you know the intrinsics of your camera. 
          Also, this would give you up-to-scale reconstruction, with the translation being a unit vector.
        [>] For each pair of images with sufficient number of matches, compute relative pose
            * 2024-02-08: For now, assume that pictures with consecutive alpha filenames are related by virtue
                          of being taken in burst fashion while orbiting the subject. Close the loop. No search needed.
            [Y] Compute the fundamental matrix F from match coordinates, 2024-02-10: Very fast, once you have matches!
            [>] Compute the essential matrix E from intrinsic matrix K and fundamental matrix F
        [ ] N-view triangulation
            [ ] Compute correspondences for each pair of images
            [ ] Compute the extrinsics of each camera shot: What is the relationship to the essential matrix?
            [ ] Find correspondence tracks across images
            [ ] class `Structure_Point`: https://github.com/imkaywu/open3DCV/blob/master/src/sfm/structure_point.h
            [ ] class `Graph`: https://github.com/imkaywu/open3DCV/blob/master/src/sfm/graph.h
            [ ] Nonlinear Triangulation
                [ ] pick graph g in graphs having maximum number of common tracks with the global graph
                [ ] merge tracks of global graph with those of graph g
                [ ] triangulate tracks
        [ ] Bundle adjustment: Optimize a reprojection error with respect to all estimated parameters
    [ ] Two-view SfM
        [ ] Render PC
    [ ] N-view SfM
        [ ] iterative step that merges multiple graphs into a global graph, for graph in graphs
            Loop:
            [ ] perform bundle adjustment
            [ ] remove track outliers based on baseline angle and reprojection error
        [ ] Render PC 
            [ ] Point Cloud
            [ ] Camera poses
            [ ] Fly around
[ ] Advanced Textured SfM
    [ ] SfM Point Cloud
    [ ] Delaunay at each camera view
        [ ] Project onto camera plane
        [ ] Run Delaunay
        [ ] Store candidate triangulation
        [ ] Score Triangles: Higher penalty is worse
            [ ] Angle between normal and ray to triangle center
            [ ] Narrowness
    [ ] Consolidate triangulations
        [ ] Prefer low-penalty triangles
        [ ] Find and fix holes: Complicated?
    [ ] Consolidate textures @ facet objects
        [ ] Project textures at each camera view
        [ ] Prefer textures with sharp pixels
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
using std::string, std::to_string, std::stof;
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
#include <fstream>
using std::ofstream;
#include <sstream>
using std::stringstream;

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
using cv::Ptr, cv::KeyPoint, cv::Point2f, cv::FeatureDetector;
#include "opencv2/xfeatures2d.hpp"
using cv::ORB, cv::normL2Sqr;
#include <opencv2/calib3d.hpp>
using cv::findFundamentalMat, cv::FM_7POINT;

/// Local ///
#include "helpers.hpp"


////////// UTILITY FUNCTIONS ///////////////////////////////////////////////////////////////////////

Mat deserialize_2d_Mat_f( string input, size_t Mrows, size_t Ncols, char sep = ',' ){
    // Deserialize an OpenCV `CV_32F` matrix stored row-major in a comma-separated list in a string
    Mat rtnMat = Mat::zeros( Mrows, Ncols, CV_32F );
    vector<string> tokens = split_string_on_char( input, sep );
    if( tokens.size() < Mrows*Ncols )  return rtnMat; // Return the zero matrix if there are insufficient elements
    size_t k = 0;
    for( size_t i = 0; i < Mrows; ++i ){
        for( size_t j = 0; j < Ncols; ++j ){
            rtnMat.at<float>(i,j) = stof( tokens[k] );
            ++k;
        }
    }
    return rtnMat;
}

// Get everything after the last ':' in a string
string get_line_arg( string line ){  return get_last( split_string_on_char( line, ':' ) );  }

vector<float> get_comma_sep_floats( string input ){
    // Separate a string on commas and attempt conversion of each part into float
    vector<string> parts = split_string_on_char( input, ',' );
    vector<float>  rtnVec;
    for( string& part : parts ){  rtnVec.push_back( stof( part ) );  }
    return rtnVec;
}

////////// GLOBALS /////////////////////////////////////////////////////////////////////////////////
string _IMG_PATH = "data/SfM/00_sculpture";



////////// IMAGE PROCESSING ////////////////////////////////////////////////////////////////////////

void fetch_images_at_path( string path, vector<string>& fNames, vector<Mat>& images, uint limit = 0, string ext = "jpg" ){
    // Load all the images found at a path
    fNames.clear();
    images.clear();
    fNames = list_files_at_path_w_ext( path, ext, true );
    uint /*-----*/ Nimg   = fNames.size();
    Mat /*------*/ img;  
    uint /*-----*/ i = 0;
    cout << "Found: " << Nimg << " files ... " << flush;
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
    Mat /*--------*/ xfrm;

    /// Constructors ///
    CamShot( string fPath, const Mat& image, const vector<KeyPoint>& keypoints ){
        path = fPath;
        imag = Mat( image );
        kpts = keypoints;
        xfrm = Mat::eye( 4, 4, CV_32F );
    };

    CamShot( string fPath, const Mat& image, const vector<KeyPoint>& keypoints, const Mat& xform ){
        path = fPath;
        imag = Mat( image );
        kpts = keypoints;
        xfrm = xform;
    };
    
    /// Methods ///
    string serialize(){
        // Write reconstruction information to a string
        stringstream outStr;
        outStr << "path:" << path << '\n';
        outStr << "xfrm:" << serialize_Mat_2D<float>( xfrm ) << '\n';
        for( const KeyPoint& kp :kpts ){
            outStr << "kpnt:" << kp.pt.x << ',' << kp.pt.y << ',' << kp.angle << ',' << kp.size << ",\n";
        }
        return outStr.str();
    }

    void serialize( string oPath ){
        // Write reconstruction information to a file
        ofstream outFile;
        outFile.open( oPath );
        outFile << serialize();
        outFile.close();
    }

    static CamShot deserialize( string iPath ){
        // Load a `CamShot` from serialized data
        vector<string>   lines = read_lines( iPath );
        string /*-----*/ path_ = get_line_arg( lines[0] );
        Mat /*--------*/ img = imread( path_, IMREAD_GRAYSCALE );
        Mat /*--------*/ xfrm_ = deserialize_2d_Mat_f( get_line_arg( lines[1] ), 3, 3, ',' );
        size_t /*-----*/ Nlin  = lines.size();
        KeyPoint /*---*/ kp_i;
        vector<float>    kpData;
        vector<KeyPoint> kps;
        for( size_t i = 2; i < Nlin; ++i ){
            kp_i   = KeyPoint{};
            kpData = get_comma_sep_floats( get_line_arg( lines[1] ) );
            if( kpData.size() < 4 ) continue;
            kp_i.pt.x  = kpData[0];
            kp_i.pt.y  = kpData[1];
            kp_i.angle = kpData[2];
            kp_i.size  = kpData[3];
            kps.push_back( kp_i );
        }
        return CamShot( path_, img, kps, xfrm_ );
    }
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

    /// Members ///
    size_t ID;
    size_t prevDex;
    size_t nextDex;
    float  diff;

    /// Constructor(s) ///
    KpMatch( size_t prevDex_, size_t nextDex_, float diff_ ){
        // Assign a unique ID and set values
        ID = N_KM;
        prevDex = prevDex_;
        nextDex = nextDex_;
        diff    = diff_;
        ++N_KM;
    }

    /// Methods ///
    bool p_same_indices( const KpMatch& other ){
        // Return true if both matches contain the same indices
        return ( ((prevDex == other.prevDex) && (nextDex == other.nextDex))
                 ||
                 ((prevDex == other.nextDex) && (nextDex == other.prevDex)) );
    }

    string serialize(){
        // Write reconstruction information to a string
        stringstream outStr;
        outStr << ID << ',' << prevDex << ',' << nextDex << ',' << diff << ',';
        return outStr.str();
    }
};



class ShotPair{ public:
    // Represents correspondences between two `CamShot`s

    /// Members ///
    shotPtr /*---*/ prev = nullptr; 
    shotPtr /*---*/ next = nullptr;
    vector<KpMatch> matches;
    Mat /*-------*/ F;

    /// Constructor(s) ///

    ShotPair( shotPtr& p, shotPtr& n ){
        prev = shotPtr( p );
        next = shotPtr( n );
    }

    /// Methods ///

    string serialize(){
        // Write reconstruction information to a string
        stringstream outStr;
        outStr << "prev:" << prev->path << '\n';
        outStr << "next:" << next->path << '\n';
        outStr << "fMtx:" << serialize_Mat_2D<float>( F ) << '\n';
        for( KpMatch& match : matches ){ 
            outStr << "mtch:" << match.serialize() << '\n';
        }
        return outStr.str();
    }

    void serialize( string oPath ){
        // Write reconstruction information to a file
        ofstream outFile;
        outFile.open( oPath );
        outFile << serialize();
        outFile.close();
    }

    // static ShotPair deserialize( string iPath ){
    //     // Load a `ShotPair` from serialized data
    //     vector<string> lines = read_lines( iPath );
    //     string prvPath = get_line_arg( lines[0] );
    //     string nxtPath = get_line_arg( lines[1] );
    //     // Mat    fMtx    = // FIXME, START HERE: LOAD FUNDIE MATRIX
    // }

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
            }
            if( (i%div) == 0 ) cout << "." << flush;
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

    // Match Coordinate Getters // 
    Point2f prev_pt_i( size_t i ){  return prev->kpts[ matches[i].prevDex ].pt;  }
    Point2f next_pt_i( size_t i ){  return next->kpts[ matches[i].nextDex ].pt;  }

    Mat load_prev_kp(){
        // Get a matrix of the previous shot correspondence point coordinates
        size_t  Mrows = matches.size();
        Mat     P   = Mat::zeros( Mrows, 2, CV_32F );
        Point2f pt_i;
        for( size_t i = 0; i < Mrows; ++i ){
            pt_i = prev_pt_i(i);
            P.at<float>(i,0) = pt_i.x;
            P.at<float>(i,1) = pt_i.y;
        }
        return P;
    }

    Mat load_next_kp(){
        // Get a matrix of the next shot correspondence point coordinates
        size_t  Mrows = matches.size();
        Mat     N     = Mat::zeros( Mrows, 2, CV_32F );
        Point2f pt_i;
        for( size_t i = 0; i < Mrows; ++i ){
            pt_i = next_pt_i(i);
            N.at<float>(i,0) = pt_i.x;
            N.at<float>(i,1) = pt_i.y;
        }
        return N;
    }

    void calc_fundamental_matx_F(){
        // Get the fundamental matrix from corresponding keypoints in two images
        Mat P = load_prev_kp();
        Mat N = load_next_kp();
        cout << "About to obtain F ..." << flush;
        Mat F = findFundamentalMat( P, N, FM_7POINT );
        cout << " Obtained!" << endl;
        cout << F << endl;
    }
};
typedef shared_ptr<ShotPair> pairPtr;

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

vector<pairPtr> pairs_from_shots( vector<shotPtr>& shotsVec, bool wrapEnd = true ){
    // Construct a picture correspondence object for pair of shots in the vector, optionally in a ring
    vector<pairPtr> rtnPairs;
    size_t /*----*/ N = shotsVec.size();
    for( size_t i = 1; i < N; ++i ){
        rtnPairs.push_back( pairPtr( new ShotPair{ shotsVec[i-1], shotsVec[i] } ) );
    }
    if( wrapEnd )
        rtnPairs.push_back( pairPtr( new ShotPair{ shotsVec[N-1], shotsVec[0] } ) );
    return rtnPairs;
}

////////// PHOTOGRAMMETRY & RECONSTRUCTION /////////////////////////////////////////////////////////

class Photogrammetry{ public:
    // Full desription and solution to a photogrammetry problem

    /// Members ///
    string /*----*/ picDir;
    vector<string>  picPaths;
    vector<Mat>     images;
    vector<shotPtr> shots;
    vector<pairPtr> pairs;
    ulong /*-----*/ Nfeat;
    ulong /*-----*/ Ktop;

    /// Constructor(s) ///
    Photogrammetry( string pDir, ulong Nfeat_ = 25000, ulong Ktop_ = 12500 ){
        picDir   = pDir;
        if( picDir[ picDir.size()-1 ] != '/' ){  picDir += '/';  }
        cout << "Finding files ... " << flush;
        fetch_images_at_path( pDir, picPaths, images );
        cout << "Creating shots ... " << flush;
        shots    = shots_from_images( picPaths, images, Nfeat_ );
        cout << "Creating pairs ... " << flush;
        pairs    = pairs_from_shots( shots, true );
        cout << "COMPLETE" << endl << endl;
        Nfeat    = Nfeat_;
        Ktop     = Ktop_;
    }

    /// Methods ///

    void save_problem( string shotPrefix = "shot_", string pairPrefix = "pair_" ){
        // Save all relevant reconstruction data except for the actual images
        string outPath;
        string fNum;
        string pad;
        size_t i = 0;
        uint   Npl = to_string( shots.size() ).size();

        // Save all camera shots for recovery
        cout << "About to serialize `CamShot`s ..." << endl;
        for( shotPtr& shot : shots ){
            fNum    = to_string( i );
            if( Npl-fNum.size() > 0 )  pad = string( Npl-fNum.size(), '0');  else  pad = "";
            outPath = picDir + shotPrefix + pad + fNum + ".CamShot";
            cout << "Write: " << outPath << " ... " << flush;
            shot->serialize( outPath );
            cout << "Done!" << endl;
            ++i;
        }

        i = 0;
        cout << "About to serialize `ShotPair`s ..." << endl;
        for( pairPtr& pair : pairs ){
            fNum    = to_string( i );
            if( Npl-fNum.size() > 0 )  pad = string( Npl-fNum.size(), '0');  else  pad = "";
            outPath = picDir + pairPrefix + pad + fNum + ".ShotPair";
            cout << "Write: " << outPath << " ... " << flush;
            pair->serialize( outPath );
            cout << "Done!" << endl;
            ++i;
        }
    }

    void brute_force_match( ulong topK = 0 ){
        // Perform matching on all pairs
        size_t N = pairs.size();
        size_t i = 1;
        if( topK == 0 )  topK = Ktop;
        for( pairPtr& pair : pairs ){  
            cout << i << " of " << N << ": " << flush;
            pair->brute_force_match( topK );
            ++i;
        }
    }
};

////////// MAIN ////////////////////////////////////////////////////////////////////////////////////
int main(){

    ulong Nfeat  = 25000;
    ulong Ktop   = Nfeat/2;

    cout << "Instantiating problem ... " << endl;
    Photogrammetry pg{ _IMG_PATH, Nfeat, Ktop };
    
    cout << "About to match ... " << endl;
    pg.brute_force_match();
    cout << endl << "##### CALCULATIONS COMPLETE #####" << endl;

    cout << endl << "Serializing reconstruction data ... " << endl;
    pg.save_problem();
    cout << endl << "##### PROBLEM SAVED #####" << endl;
    

    return 0;
}