// g++ 01_SfM_bgn.cpp `pkg-config --cflags --libs opencv4` -std=c++17 -lopencv_xfeatures2d -I /usr/local/include/opencv4/ -o build-struct.out

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
        [Y] Bi-directional verification
            [Y] Brute force, 2024-02-08: Have to wait quite a while to get mutual matches from brute force,
                5321 matched keypoint pairs for two images with 50000 features each.
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
        [Y] For each pair of images with sufficient number of matches, compute relative pose, 2024-02-19: OBTAINED
            * 2024-02-08: For now, assume that pictures with consecutive alpha filenames are related by virtue
                          of being taken in burst fashion while orbiting the subject. Close the loop. No search needed.
            [Y] Compute the fundamental matrix F from match coordinates, 2024-02-10: Very fast, once you have matches!
            [Y] Compute the essential matrix E from intrinsic matrix K and fundamental matrix F, 2024-02-19: OBTAINED
        [>] N-view triangulation
            [Y] Compute correspondences for each pair of images, 2024-02-19: OBTAINED
            [>] Compute the extrinsics of each camera shot: 
                [Y] Translation, 2024-02-19: OBTAINED
                [Y] Orientation, 2024-02-19: OBTAINED
                [N] Q: What is the relationship to the essential matrix?
                [>] Render all relative poses as a sanity check
                [>] Serialize all relative poses
            [>] Find correspondence tracks across `ShotPair`s: class `StructureNode`
                [>] Track, 3D Location, Color
                    [>] Need to trace back to keypoints?
                    [>] ISSUE: Something is VERY slow about building tracks!
                        [>] Print out number of tracks vs how fast they are growing
                [ ] Need to store error info?
                 * Ref: `Structure_Point`: https://github.com/imkaywu/open3DCV/blob/master/src/sfm/structure_point.h
            [ ] class `Graph`: https://github.com/imkaywu/open3DCV/blob/master/src/sfm/graph.h
                [ ] Q: Does this offer anything that `StructureNode` does not?
            [ ] Nonlinear Triangulation
                [ ] pick graph g in graphs having maximum number of common tracks with the global graph
                [ ] merge tracks of global graph with those of graph g
                [ ] triangulate tracks
        [ ] Bundle adjustment: Optimize a reprojection error with respect to all estimated parameters
    [ ] N-view SfM
        [ ] iterative step that merges multiple graphs into a global graph, for graph in graphs
            Loop:
            [ ] perform bundle adjustment
            [ ] remove track outliers based on baseline angle and reprojection error
        [ ] Render PC 
            [ ] Point Cloud
            [ ] Camera poses
            [ ] Fly around
        [ ] Improvements
            [ ] Filter the max distance for keypoint matches
            [ ] Approximate nearest neighbour matching for keypoints
[ ] Advanced Textured SfM
    [ ] SfM Point Cloud
    [ ] Iterative Delaunay at each camera view
        [ ] Order points by increasing distance from the camera location for this shot
        [ ] For each point
            [ ] Project onto camera plane
            [ ] Run Iterative Delaunay 
            [ ] Compute points occluded by facets and remove from this view
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
#include <memory>
using std::shared_ptr;
#include <map>
using std::pair;
#include <list>
using std::list;
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
using cv::Mat, cv::String, cv::Vec;
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
using cv::findEssentialMat, cv::FM_7POINT, cv::recoverPose;

/// Local ///
#include "helpers.hpp"


////////// UTILITY FUNCTIONS ///////////////////////////////////////////////////////////////////////

Mat deserialize_2d_Mat_f( string input, int Mrows, int Ncols, char sep = ',' ){
    // Deserialize an OpenCV `CV_32F` matrix stored row-major in a comma-separated list in a string
    Mat rtnMat = Mat::zeros( Mrows, Ncols, CV_32F );

    // cout << "Got input: " << input << endl;

    vector<string> tokens = split_string_on_char( input, sep );
    // cout << "Separated input: " << endl;
    // for( string& token : tokens ) cout << '\t' << token << endl;


    if( tokens.size() < Mrows*Ncols )  return rtnMat; // Return the zero matrix if there are insufficient elements
    int k = 0;
    float val;
    string item;
    for( int i = 0; i < Mrows; ++i ){
        for( int j = 0; j < Ncols; ++j ){
            // cout << endl << "\t\tAt (" << i << ',' << j << "), " << k << flush;
            // cout << ", " << tokens[k] << flush;
            // item = tokens[k];
            // item += '\0'
            // cout << ", " << stof( tokens[k] ) << flush;
            try{
                rtnMat.at<float>(i,j) = stof( tokens[k] );
            }catch (const std::out_of_range& e) {
                cout << "Out of Range error." << endl;
                rtnMat.at<float>(i,j) = nanf("");
            }
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

size_t str_to_size_t( string input ){
    // Convert the `input` to a `size_t`
    stringstream sstream;
    size_t /*-*/ result;
    sstream >> result;
    return result;
}

vector<size_t> get_comma_sep_size_t( string input ){
    // Separate a string on commas and attempt conversion of each part into size_t
    vector<string> parts = split_string_on_char( input, ',' );
    vector<size_t> rtnVec;
    size_t /*---*/ result;
    for( string& part : parts ){  
        result = str_to_size_t( part );
        rtnVec.push_back( result );  
    }
    return rtnVec;
}


////////// IMAGE PROCESSING ////////////////////////////////////////////////////////////////////////

void fetch_images_at_path( string path, vector<string>& fNames, vector<Mat>& images, 
                           uint limit = 0, string ext = "jpg" ){
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

    static shared_ptr<CamShot> deserialize( string iPath ){
        // Load a `CamShot` from serialized data
        vector<string>   lines = read_lines( iPath );
        string /*-----*/ path_ = get_line_arg( lines[0] );
        Mat /*--------*/ img = imread( path_, IMREAD_GRAYSCALE );
        Mat /*--------*/ xfrm_ = deserialize_2d_Mat_f( get_line_arg( lines[1] ), 4, 4, ',' );
        size_t /*-----*/ Nlin  = lines.size();
        KeyPoint /*---*/ kp_i;
        vector<float>    kpData;
        vector<KeyPoint> kps;
        for( size_t i = 2; i < Nlin; ++i ){
            kp_i   = KeyPoint{};
            kpData = get_comma_sep_floats( get_line_arg( lines[i] ) );
            if( kpData.size() < 4 ) continue;
            kp_i.pt.x  = kpData[0];
            kp_i.pt.y  = kpData[1];
            kp_i.angle = kpData[2];
            kp_i.size  = kpData[3];
            kps.push_back( kp_i );
        }
        return shared_ptr<CamShot>( new CamShot( path_, img, kps, xfrm_ ) );
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
    KpMatch(){
        // Assign a unique ID and set values
        ID = N_KM;
        prevDex = 0;
        nextDex = 0;
        diff    = 0.0f;
        ++N_KM;
    }

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
    Mat /*-------*/ E; // Output essential matrix.
    Mat /*-------*/ R; // Output rotation matrix.
    Mat /*-------*/ t; // Output translation vector.
    vector<KpMatch> matches;
    

    /// Constructor(s) ///

    ShotPair( shotPtr& p, shotPtr& n ){
        prev = shotPtr( p );
        next = shotPtr( n );
        E = Mat::zeros(3,3,CV_32F);; // Output essential matrix.
        R = Mat::zeros(3,3,CV_32F);; // Output rotation matrix.
        t = Mat::zeros(3,1,CV_32F); // Output translation vector.
    }

    ShotPair( shotPtr& p, shotPtr& n, Mat& E_, Mat& R_, Mat& t_, vector<KpMatch>& matches_ ){
        prev    = shotPtr( p );
        next    = shotPtr( n );
        E /*-*/ = E_;
        R /*-*/ = R_;
        t /*-*/ = t_;
        matches = matches_;
    }

    /// Methods ///

    string serialize(){
        // Write reconstruction information to a string
        stringstream outStr;
        outStr << "prev:" << prev->path << '\n';
        outStr << "next:" << next->path << '\n';
        outStr << "fMtx:" << serialize_Mat_2D<double>( E ) << '\n'; // THESE BECAME DOUBLE DURING POSE RECOVERY
        outStr << "rMtx:" << serialize_Mat_2D<double>( R ) << '\n';
        outStr << "tVec:" << serialize_Mat_2D<double>( t ) << '\n';

        // cout << "Translation:" << endl << t << endl;
        // cout << "Wrote: " << serialize_Mat_2D<double>( t ) << endl << endl;
        // cout << "Orientation:" << endl << R << endl << endl;
        // cout << "Wrote: " << serialize_Mat_2D<double>( R ) << endl << endl;

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

    static shared_ptr<ShotPair> deserialize( string iPath, const vector<shotPtr>& shots ){
        // Load a `ShotPair` from serialized data
        vector<string>  lines   = read_lines( iPath );
        string /*----*/ prvPath = get_line_arg( lines[0] );
        string /*----*/ nxtPath = get_line_arg( lines[1] );
        Mat /*-------*/ E_ /**/ = deserialize_2d_Mat_f( get_line_arg( lines[2] ), 4, 4 ); // Output essential matrix.
        Mat /*-------*/ R_ /**/ = deserialize_2d_Mat_f( get_line_arg( lines[3] ), 3, 3 ); // Output rotation matrix.
        Mat /*-------*/ t_ /**/ = deserialize_2d_Mat_f( get_line_arg( lines[4] ), 3, 1 ); // Output translation vector.
        size_t /*----*/ Nlin    = lines.size();
        KpMatch /*---*/ mtc_i;
        vector<float>   mtcData;
        vector<KpMatch> mtcs;
        shotPtr /*---*/ prev_ = nullptr;
        shotPtr /*---*/ next_ = nullptr;
        for( size_t i = 5; i < Nlin; ++i ){
            mtc_i   = KpMatch{};
            mtcData = get_comma_sep_floats( get_line_arg( lines[i] ) );
            if( mtcData.size() < 4 ) continue;
            mtc_i.ID /**/ = (size_t) mtcData[0];
            mtc_i.prevDex = (size_t) mtcData[1];
            mtc_i.nextDex = (size_t) mtcData[2];
            mtc_i.diff    = mtcData[3];
            mtcs.push_back( mtc_i );
        }
        for( const shotPtr& shot : shots ){
            if( shot->path == prvPath )  prev_ = shotPtr( shot );
            if( shot->path == nxtPath )  next_ = shotPtr( shot );
            if( (prev_ != nullptr) && (next_ != nullptr) )  break;
        }
        return shared_ptr<ShotPair>( new ShotPair{ prev_, next_, E_, R_, t_, mtcs } );
    }

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
            cout << "\t" << match.ID << ": " << match.prevDex << "<->" << match.nextDex << " = " 
                 << match.diff << endl;
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

    void recover_relative_pose( Mat& camCal ){
        // Get the fundamental matrix from corresponding keypoints in two images
        Mat P = load_prev_kp();
        Mat N = load_next_kp();
        cout << "About to obtain relative pose from " << matches.size() << " matches ..." << flush;
        E = findEssentialMat( P, N, camCal );
        recoverPose( E, P, N, camCal, R, t );
        cout << " Obtained!" << endl;
        // cout << "Translation:" << endl << t << endl;
        // cout << "Orientation:" << endl << R << endl << endl;
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



////////// 3D GRAPH STRUCTURE //////////////////////////////////////////////////////////////////////
typedef pair<shotPtr,size_t> shotKpDex; // Indicates a camera view + `KeyPoint` index


class StructureNode{ public:
    // A colored point in physical space, that points to photo keypoints that govern its location

    /// Members ///

    list<shotKpDex> track; // LL of keypoints that govern location and color
    Vec3f /*-----*/ coord; // World frame location
    Vec3i /*-----*/ color; // Average color

    StructureNode( const shotPtr& shot, size_t index ){
        // Begin a new track with the beginning 
        track.push_back( shotKpDex( shotPtr( shot ), index ) );
        coord = {0,0,0};
        color = {0,0,0};
    }

    /// Methods ///

    void append_to_track( shotPtr shot, size_t index ){
        // Extend the track by one keypoint
        track.emplace_back( shotKpDex{ shotPtr( shot ), index } );
    }
};
typedef shared_ptr<StructureNode> nodePtr;


vector<nodePtr> get_nodes_from_pairs( vector<pairPtr>& pairs ){
    // Gather contiguous correspondences between keypoints that span one or more `ShotPair`s
    vector<nodePtr> rtnNodes;
    size_t /*----*/ Npairs = pairs.size();
    bool /*------*/ nodeFound;
    // 1. For every contiguous pair of camera shots
    // NOTE: This function assumes pairs were already formed in a ring
    // FIXME: NEEDS CHECKING
    for( pairPtr& pair : pairs ){ 
        cout << rtnNodes.size() << ", " << flush;
        // 2. For every correspondence in that pair
        for( KpMatch& match : pair->matches ){
            nodeFound = false;
            // A. Search ends of existing tracks
            for( nodePtr& node : rtnNodes ){
                if( match.prevDex == node->track.back().second ){
                    node->append_to_track( shotPtr( pair->next ), match.nextDex );
                    nodeFound = true;
                }
            }
            // B. Else start new tracks
            if( !nodeFound ){
                rtnNodes.push_back( nodePtr( new StructureNode{ shotPtr( pair->next ), match.nextDex } ) );
            }
        }
    }
    return rtnNodes;
}


////////// PHOTOGRAMMETRY & RECONSTRUCTION /////////////////////////////////////////////////////////

class Photogrammetry{ public:
    // Full desription and solution to a photogrammetry problem

    ///// Members /////////////////////////////////////////////////////////

    /// Params ///
    string picDir; // Directory of (assumed) in-order 360deg walk-around of reconstructed object
    ulong  Nfeat; //- # of features to find in each image
    ulong  Ktop; // - Max # of mutual correspondences to consider
    Mat    camCal; // Camera calibration matrix
    /// Reconstruction Data ///
    vector<string>  picPaths; // Paths to individual imates
    vector<Mat>     images; // - All images loaded as matrices
    vector<shotPtr> shots; // -- Image + `KeyPoint`s, FIXME: DOUBLE STORAGE
    vector<pairPtr> pairs; // -- `KeyPoint` correspondences
    vector<nodePtr> nodes; // -- 3D points based on multiple correspondences


    ///// Constructor(s) //////////////////////////////////////////////////

    Photogrammetry( string pDir, ulong Nfeat_, ulong Ktop_ ){
        picDir = pDir;
        if( picDir[ picDir.size()-1 ] != '/' ){  picDir += '/';  }
        cout << "Finding files ... " << flush;
        fetch_images_at_path( pDir, picPaths, images );
        cout << "Creating shots ... " << flush;
        shots = shots_from_images( picPaths, images, Nfeat_ );
        images.clear(); // Fix double storage
        cout << "Creating pairs ... " << flush;
        pairs = pairs_from_shots( shots, true );
        cout << "COMPLETE" << endl << endl;
        Nfeat = Nfeat_;
        Ktop  = Ktop;
    }

    Photogrammetry( string pDir, string mainExt = "Photogrammetry",
                    string shotExt = "CamShot", string pairExt = "ShotPair" ){
        // Load
        picDir = pDir;
        if( picDir[ picDir.size()-1 ] != '/' ){  picDir += '/';  }
        vector<string> mPaths = list_files_at_path_w_ext( picDir, mainExt );
        cout << "About to load problem ... " << flush;
        if( mPaths.size() )
            load_problem( mPaths[0], shotExt, pairExt );
        else
            cout << "NO main solution file EXISTS!" << endl;
    }


    ///// De/Serialization ////////////////////////////////////////////////

    string serialize(){
        // Write root-level reconstruction information to a string
        stringstream outStr;
        outStr << "pDir:" << picDir << endl;
        outStr << "feat:" << Nfeat  << endl;
        outStr << "kTop:" << Ktop   << endl;
        return outStr.str();
    }

    void serialize( string oPath ){
        // Write root-level reconstruction information to a file
        ofstream outFile;
        cout << "Write: " << oPath << " ... " << flush;
        outFile.open( oPath );
        outFile << serialize();
        outFile.close();
        cout << "Done!" << endl;
    }

    void load_cam_calibration( string cPath = "", string camExt = "intrinsic" ){
        if( cPath.length() == 0 ){
            vector<string> paths = list_files_at_path_w_ext( picDir, camExt, true );
            if( !(paths.size()) ){
                cout << "NO calibration file was found!" << endl;
                return;
            }
            cPath = paths[0];
        }
        vector<string> lines = read_lines( cPath );
        if( lines.size() ){
            camCal = deserialize_2d_Mat_f( get_line_arg( lines[0] ), 3, 3, ',' );
            cout << "Camera Calibration Matrix from " << cPath << ":" << endl;
            cout << camCal << endl;
        }else{
            cout << "Calibration file " << cPath << " LACKS appropriate data!" << endl;
        }
    }

    void deserialize( string iPath ){
        // Read root-level reconstruction information from a file
        cout << "About to read main file ... " << endl;
        vector<string> lines = read_lines( iPath );
        cout << "Main file has " << lines.size() << " lines!" << endl;
        Nfeat = str_to_size_t( get_line_arg( lines[1] ) );
        Ktop  = str_to_size_t( get_line_arg( lines[2] ) );
        cout << "About to read camera calibration ... " << endl;
        load_cam_calibration();
    }

    void save_problem( string shotPrefix = "shot_", string pairPrefix = "pair_" ){
        // Save all relevant reconstruction data except for the actual images
        string outPath;
        string fNum;
        string pad;
        size_t i   = 0;
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

        outPath = picDir + "soln" + ".Photogrammetry";
        serialize( outPath );
    }

    void load_problem( string mainPath, string shotExt = "CamShot", string pairExt = "ShotPair" ){
        // Load all relevant reconstruction data including the actual images
        // NOTE: This function should NOT populate `images`. Image data exists in `shots`
        cout << "Find shot paths ..." << endl;
        vector<string> shotPaths = list_files_at_path_w_ext( picDir, shotExt, true );
        cout << "Find pair paths ..." << endl;
        vector<string> pairPaths = list_files_at_path_w_ext( picDir, pairExt, true );
        cout << "Load shots ..." << endl;
        for( string& path : shotPaths ){
            shots.push_back( CamShot::deserialize( path ) );
            cout << '.' << flush;
        } cout << endl;
        cout << "Load pairs ..." << endl;
        for( string& path : pairPaths ){
            pairs.push_back( ShotPair::deserialize( path, shots ) );
            cout << '.' << flush;
        } cout << endl;
        cout << "Load main: " << mainPath << " ... " << endl;
        deserialize( mainPath );
    }

    ///// Solver //////////////////////////////////////////////////////////

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

    void recover_relative_poses(){
        // Get the relative pose for every pair of shots
        for( pairPtr& pair : pairs ){  pair->recover_relative_pose( camCal );  }
    }

    void get_nodes(){
        // Attempt to get correspondences across 
        nodes = get_nodes_from_pairs( pairs );
    }
};

////////// GLOBALS /////////////////////////////////////////////////////////////////////////////////
string _IMG_PATH = "data/SfM/00_sculpture";

////////// MAIN ////////////////////////////////////////////////////////////////////////////////////

int main(){

    ulong Nfeat  = 25000;
    ulong Ktop   = Nfeat/2;

    cout << "Instantiating problem ... " << endl;
    Photogrammetry pg{ _IMG_PATH };
    
    // cout << "About to match ... " << endl;
    // pg.brute_force_match();

    cout << "About to calculate relative poses ... " << endl;
    pg.load_cam_calibration();
    pg.recover_relative_poses();
    pg.get_nodes();
    cout << endl << "##### CALCULATIONS COMPLETE #####" << endl;

    cout << endl << "Serializing reconstruction data ... " << endl;
    pg.save_problem();
    cout << endl << "##### PROBLEM SAVED #####" << endl;
    

    return 0;
}