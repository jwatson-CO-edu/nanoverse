////////// INIT ////////////////////////////////////////////////////////////////////////////////////
#include "SfM.hpp"



////////// POSE GRAPH //////////////////////////////////////////////////////////////////////////////

ImgNode::ImgNode( string path, const Mat& sourceImg ){
    imgPth = path;
    image  = sourceImg;
}


vector<NodePtr> images_to_nodes( string path, string ext ){
    // Populate a vector of nodes with paths and images
    NodePtr /*---*/ node_i;
    vector<NodePtr> rtnNds;
    vector<string>  fNames;
    vector<Mat>     images;
    KAZE /*------*/ kazeMaker{};
    fetch_images_at_path( path, fNames, images, 0, ext );
    uint N = images.size();
    cout << endl;
    for( size_t i = 0; i < N; ++i ){
        node_i = NodePtr{ new ImgNode{ fNames[i], images[i] } };
        kazeMaker.get_KAZE_keypoints( images[i], node_i->keyPts, node_i->kpDesc );
        // Assume pix were taken in a sequence
        if( i > 0 ){  
            node_i->prev = NodePtr{ rtnNds.back() };
            rtnNds.back()->next = NodePtr{ node_i };
        } 
        rtnNds.push_back( NodePtr{ node_i } );
        cout << node_i->keyPts.size() << " kp, " << flush;
    }
    cout << endl;
    return rtnNds;
}



////////// RELATIVE CAMERA POSE ////////////////////////////////////////////////////////////////////

Mat load_cam_calibration( string cPath ){
    // Fetch the K matrix stored as plain text
    Mat matx;
    vector<string> lines = read_lines( cPath );
    if( lines.size() ){
        matx = deserialize_2d_Mat_f( get_line_arg( lines[0] ), 3, 3, ',' );
        cout << "Camera Calibration Matrix from " << cPath << ":" << endl;
        cout << matx << endl;
    }else{
        cout << "Calibration file " << cPath << " LACKS appropriate data!" << endl;
    }
    return matx;
}


RelativePoseEstimator::RelativePoseEstimator( string kPath, float ratioThresh_, float ransacThresh_, float confidence_ ) {
    // Use FLANN matcher for efficient matching
    matcher = DescriptorMatcher::create( DescriptorMatcher::BRUTEFORCE_HAMMING );
    camera_matrix = load_cam_calibration( kPath );
    dist_coeffs = cv::Mat::zeros(5, 1, CV_64F); // Initialize distortion coefficients to zero (assuming undistorted images)
    ratioThresh  = ratioThresh_;
    ransacThresh = ransacThresh_;
    confidence = confidence_;
}
    
    
PoseResult RelativePoseEstimator::estimatePose( const vector<KeyPoint>& keypoints1, const Mat& descriptors1, 
                                                const vector<KeyPoint>& keypoints2, const Mat& descriptors2 ){
    PoseResult result;
    result.success = false;
            
    
    
    // Step 2: Match features using k-nearest neighbors (k=2)
    vector<vector<DMatch>> knn_matches;
    matcher->knnMatch(descriptors1, descriptors2, knn_matches, 2);
    
    // Step 3: Filter matches using Lowe's ratio test
    vector<DMatch>& good_matches = result.good_matches;
    for (const auto& matches : knn_matches) {
        if (matches.size() == 2 && 
            matches[0].distance < ratioThresh * matches[1].distance) {
            good_matches.push_back(matches[0]);
        }
    }
    
    // Check if enough good matches were found
    if (good_matches.size() < 8) {
        std::cout << "Not enough good matches found" << std::endl;
        return result;
    }
    
    // Step 4: Extract matched point coordinates
    vector<Point2f>& points1 = result.matched_points1;
    vector<Point2f>& points2 = result.matched_points2;
    
    for (const auto& match : good_matches) {
        points1.push_back(keypoints1[match.queryIdx].pt);
        points2.push_back(keypoints2[match.trainIdx].pt);
    }
    
    // Step 5: Calculate essential matrix using RANSAC
    cv::Mat E = cv::findEssentialMat( points1, points2, camera_matrix,
                                      cv::RANSAC, confidence, ransacThresh,
                                      cv::noArray() );
                                    
    if( E.empty() ){
        std::cout << "Could not estimate essential matrix!" << std::endl;
        return result;
    }
    
    // Step 6: Recover R and t from essential matrix
    cv::Mat R, t;
    cv::recoverPose( E, points1, points2, camera_matrix, R, t );
    
    // Store results
    result.R = R;
    result.t = t;
    result.success = true;
    
    return result;
}
    
// Utility function to visualize matches
Mat RelativePoseEstimator::visualizeMatches( const Mat& img1, const Mat& img2,
                                    const PoseResult& result) {
    Mat output;
    if (!result.success) {
        return output;
    }
    
    cv::drawMatches( img1, vector<KeyPoint>(), img2, vector<KeyPoint>(),
                        result.good_matches, output );
    return output;
}