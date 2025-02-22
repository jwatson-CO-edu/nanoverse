////////// INIT ////////////////////////////////////////////////////////////////////////////////////
#include "SfM.hpp"



////////// RELATIVE CAMERA POSE ////////////////////////////////////////////////////////////////////

vector<Mat> load_cam_calibration( string cPath ){
    // Fetch the K matrix stored as plain text
    vector<Mat> K_and_D;
    vector<string> lines = read_lines( cPath );
    if( lines.size() ){
        K_and_D.push_back( deserialize_2d_Mat_d( get_line_arg( lines[0] ), 3, 3, ',' ) );
        K_and_D.push_back( deserialize_2d_Mat_d( get_line_arg( lines[1] ), 1, 5, ',' ) );
    }else{
        cout << "Calibration file " << cPath << " LACKS appropriate data!" << endl;
    }
    return K_and_D;
}


CamData::CamData( string kPath, string imgDir, double horzFOV_, double vertFOV_, string ext ){

    vector<Mat>    image;
    vector<String> fNames;
    vector<Mat>    params = load_cam_calibration( kPath );
    fetch_images_at_path( imgDir, fNames, image, 1, ext );

    Kintrinsic  = params[0];
    distortion  = params[1];
    imgSize     = Point2i{ image[0].rows, image[0].cols };
    horzFOV_deg = horzFOV_;
    vertFOV_deg = vertFOV_;
}


TwoViewResult empty_result(){
    TwoViewResult rtnRes{};
    rtnRes.success = false;
    return rtnRes;
}


TwoViewCalculator::TwoViewCalculator( double ratioThresh_, double ransacThresh_, double confidence_ ) {
    // Use FLANN matcher for efficient matching
    matcher /**/ = DescriptorMatcher::create( DescriptorMatcher::BRUTEFORCE_HAMMING );
    ratioThresh  = ratioThresh_;
    ransacThresh = ransacThresh_;
    confidence   = confidence_;
}
    
    
TwoViewResult TwoViewCalculator::estimate_pose( const CamData& camInfo,
                                                const vector<KeyPoint>& keypoints1, const Mat& descriptors1, 
                                                const vector<KeyPoint>& keypoints2, const Mat& descriptors2 ){
    // Use keypoint collections from two images to calculate a relative pose
    // Insiration: https://claude.site/artifacts/6b2b5025-4cdf-43cd-9762-b8f4fdb74d90
    TwoViewResult result;
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
    vector<Point2d>& points1 = result.matched_points1;
    vector<Point2d>& points2 = result.matched_points2;
    
    for (const auto& match : good_matches) {
        points1.push_back(keypoints1[match.queryIdx].pt);
        points2.push_back(keypoints2[match.trainIdx].pt);
    }
    
    // Step 5: Calculate essential matrix using RANSAC
    Mat E = cv::findEssentialMat( points1, points2, camInfo.Kintrinsic,
                                  cv::RANSAC, confidence, ransacThresh,
                                  cv::noArray() );
                                    
    if( E.empty() ){
        std::cout << "Could not estimate essential matrix!" << std::endl;
        return result;
    }
    
    // Step 6: Recover R and t from essential matrix
    Mat R, t;
    cv::recoverPose( E, points1, points2, camInfo.Kintrinsic, R, t );
    
    // Store results
    result.R = R;
    result.t = t;
    result.success = true;
    
    return result;
}
    

Mat TwoViewCalculator::visualize_matches( const Mat& img1, const vector<KeyPoint>& keypoints1,
                                          const Mat& img2, const vector<KeyPoint>& keypoints2,
                                          const TwoViewResult& result ){
    // Utility function to visualize matches
    Mat output;
    if( !result.success ){
        return output;
    }
    
    cv::drawMatches( img1, keypoints1, img2, keypoints2,
                     result.good_matches, output );
    // imshow( "Keypoint Matches", output );
    // waitKey(0);
    /* The function is not implemented. Rebuild the library with Windows, GTK+ 2.x or Cocoa support. 
    If you are on Ubuntu or Debian, install libgtk2.0-dev and pkg-config, then re-run cmake or 
    configure script in function 'cvShowImage' */
    return output;
}


///// Point Cloud Creation ////////////////////////////////////////////////

PCPosPtr vec_Point3d_to_PntPos_pcd( const vector<Point3d>& pntsList, bool atCentroid ){
    // Convert a vector of OpenCV `Point3d` to a PCL XYZ PCD
    PCPosPtr rtnPCD{ new PCPos{} };
    PntPos basicPoint;
    PntPos centroid{ 0.0f, 0.0f, 0.0f };

    if( atCentroid ){
        for( const Point3d& pnt : pntsList ){
            centroid = centroid + PntPos{ (float) pnt.x, (float) pnt.y, (float) pnt.z };
        }
        centroid = centroid / (double) pntsList.size();
    }

    for( const Point3d& pnt : pntsList ){
        basicPoint.x = pnt.x;
        basicPoint.y = pnt.y;
        basicPoint.z = pnt.z;
        rtnPCD->push_back( basicPoint - centroid );
    }

    return rtnPCD;
}


PCClrPtr vec_Point3d_to_PntClr_pcd( const vector<Point3d>& pntsList, const Color& setColor, bool atCentroid ){
    // Convert a vector of OpenCV `Point3d` to a PCL XYZ PCD
    PCClrPtr rtnPCD{ new PCClr{} };
    PntClr basicPoint;
    PntClr centroid{ 0.0f, 0.0f, 0.0f };

    if( atCentroid ){
        for( const Point3d& pnt : pntsList ){
            centroid = centroid + PntClr{ (float) pnt.x, (float) pnt.y, (float) pnt.z,
                                          setColor[0], setColor[1], setColor[2], setColor[3] };
        }
        centroid = centroid / (double) pntsList.size();
    }

    for( const Point3d& pnt : pntsList ){
        basicPoint.x = pnt.x;
        basicPoint.y = pnt.y;
        basicPoint.z = pnt.z;
        basicPoint.r = setColor[0];
        basicPoint.g = setColor[1];
        basicPoint.b = setColor[2];
        basicPoint.a = setColor[3];
        rtnPCD->push_back( basicPoint - centroid );
    }

    return rtnPCD;
}


PCClrPtr vec_PntPos_to_PntClr_pcd( const PCPosPtr pntsList, const Color& setColor, bool atCentroid ){
    // Convert a vector of OpenCV `Point3d` to a PCL XYZ PCD
    PCClrPtr rtnPCD{ new PCClr{} };
    PntClr basicPoint;
    PntClr centroid{ 0.0f, 0.0f, 0.0f };

    if( atCentroid ){
        for( const PntPos& pnt : *pntsList ){
            centroid = centroid + PntClr{ (float) pnt.x, (float) pnt.y, (float) pnt.z,
                                          setColor[0], setColor[1], setColor[2], setColor[3] };
        }
        centroid = centroid / (double) pntsList->size();
    }

    for( const PntPos& pnt : *pntsList ){
        basicPoint.x = pnt.x;
        basicPoint.y = pnt.y;
        basicPoint.z = pnt.z;
        basicPoint.r = setColor[0];
        basicPoint.g = setColor[1];
        basicPoint.b = setColor[2];
        basicPoint.a = setColor[3];
        rtnPCD->push_back( basicPoint - centroid );
    }

    return rtnPCD;
}


///// Result Struct ///////////////////////////////////////////////////////


void TwoViewCalculator::generate_point_cloud( const CamData& camInfo, TwoViewResult& result, double zMin, double zMax ){
    // Insiration: https://claude.site/artifacts/f4a5b5fd-b317-4c6f-b42d-f35b92d03cbf
    
    // Check input validity
    if (result.matched_points1.size() != result.matched_points2.size() || result.matched_points1.empty()) {
        throw invalid_argument( "Keypoint vectors must be non-empty and of equal size" );
    }
    if( result.R.size() != Size(3, 3) || result.t.size() != Size(1, 3) || camInfo.Kintrinsic.size() != Size(3, 3) ){
        throw invalid_argument( "Invalid matrix dimensions" );
    }

    // Construct projection matrices
    Mat P1 = camInfo.Kintrinsic * Mat::eye( 3, 4, CV_64F );  // First camera matrix [K|0]
    // Create second camera matrix [K(R|t)]
    Mat P2( 3, 4, CV_64F );
    cv::hconcat( result.R, result.t, P2 );
    P2 = camInfo.Kintrinsic * P2;
    // cout << endl << "P1:\n" << P1 << endl << "P2:\n" << P2 << endl;
    
    Point3d /*----*/ avgPnt{ 0.0, 0.0, 0.0 };
    vector<Point3d>  points3D;
    result.relPCD = PCPosPtr{ new PCPos{} };
    array<Point3d,2> bBox  = { Point3d{1e6,1e6,1e6,}, Point3d{-1e6,-1e6,-1e6,} };
    // double /*-----*/ scale = norm( result.t, NORM_L2 );

    points3D.reserve( result.matched_points1.size() );

    // Triangulate each pair of corresponding points
    for( size_t i = 0; i < result.matched_points1.size(); i++ ){
        // Convert points to normalized homogeneous coordinates
        Point2d pt1 = result.matched_points1[i];
        Point2d pt2 = result.matched_points2[i];

        // Create matrices for triangulation
        Mat point4D;
        Mat points1( 2, 1, CV_64F );
        Mat points2( 2, 1, CV_64F );
        points1.at<double>(0) = pt1.x;
        points1.at<double>(1) = pt1.y;
        points2.at<double>(0) = pt2.x;
        points2.at<double>(1) = pt2.y;

        // Triangulate the point
        cv::triangulatePoints( P1, P2, points1, points2, point4D );

        // Convert homogeneous coordinates to 3D point
        point4D = point4D / point4D.at<double>(3);
        // point4D = point4D * (scale / point4D.at<double>(3));

        if( (zMin <= point4D.at<double>(2)) && (point4D.at<double>(2) <= zMax ) ){

            Point3d point3D(
                point4D.at<double>(0),
                point4D.at<double>(1),
                point4D.at<double>(2)
            );

            // Add to point cloud
            points3D.push_back( point3D );
            avgPnt += point3D;
            bBox[0].x = min( bBox[0].x, point3D.x );
            bBox[1].x = max( bBox[1].x, point3D.x );
            bBox[0].y = min( bBox[0].y, point3D.y );
            bBox[1].y = max( bBox[1].y, point3D.y );
            bBox[0].z = min( bBox[0].z, point3D.z );
            bBox[1].z = max( bBox[1].z, point3D.z );
        }
    }

    avgPnt /= (double) points3D.size();

    cout << "PCD Centroid: " << avgPnt << endl << endl; 

    result.relPCD   = vec_Point3d_to_PntPos_pcd( points3D, false );
    result.centroid = avgPnt;
    result.bbox     = bBox;
}



////////// POSE GRAPH //////////////////////////////////////////////////////////////////////////////

ImgNode::ImgNode( string path, const Mat& sourceImg ){
    imgPth  = path;
    image   = sourceImg;
    imgSize = Point2i{ sourceImg.rows, sourceImg.cols };
    prev    = NULL;
    next    = NULL;
    imgRes2 = empty_result();
}


vector<NodePtr> images_to_nodes( string path, string ext, const CamData& camInfo, double zMin, double zMax, double minSharp ){
    // Populate a vector of nodes with paths and images
    NodePtr /*-----*/ node_i;
    NodePtr /*-----*/ node_im1;
    vector<NodePtr>   rtnNds;
    vector<string>    fNames;
    vector<Mat> /*-*/ images;
    KAZE /*--------*/ kazeMaker{};
    TwoViewResult     res;
    TwoViewCalculator est{};
    Mat /*---------*/ undistImg;
    double /*------*/ avgSharp = 0.0;
    fetch_images_at_path( path, fNames, images, 0, ext );
    uint N = images.size();
    cout << endl;
    for( size_t i = 0; i < N; ++i ){
        undistort( images[i], undistImg, camInfo.Kintrinsic, camInfo.distortion );
        node_i = NodePtr{ new ImgNode{ fNames[i], undistImg } };
        node_i->relXform  = Mat::eye( 4, 4, CV_64F );
        node_i->absXform  = Mat::eye( 4, 4, CV_64F );
        node_i->sharpness = measure_sharpness( images[i] );
        avgSharp += node_i->sharpness;
        kazeMaker.get_KAZE_keypoints( images[i], node_i->keyPts, node_i->kpDesc );

        cout << "This Image: " << node_i->keyPts.size() << " kp, Sharpness: " << node_i->sharpness << endl;

        if( node_i->sharpness < minSharp ){
            cout << "Image " << i << " TOO BLURRY!" << endl;
            continue;
        }
        
        // Assume pix were taken in a sequence
        if( i > 0 ){  
            node_im1     = NodePtr{ rtnNds.back() };
            node_i->prev = node_im1;
            rtnNds.back()->next = NodePtr{ node_i };

            res = est.estimate_pose( 
                camInfo, 
                node_im1->keyPts, node_im1->kpDesc, 
                node_i->keyPts  , node_i->kpDesc 
            );

            // cout << "Rotation:\n" << res.R << endl << "Translation:\n" << res.t << endl;

            if( res.success ){
                for( size_t j = 0; j < 3; ++j ){
                    for( size_t k = 0; k < 3; ++k ){
                        node_i->relXform.at<double>( j, k ) = res.R.at<double>( j, k );
                    }
                    // node_i->relXform.at<double>( j, 3 ) = res.t.at<double>( j, 0 );
                    node_i->relXform.at<double>( j, 3 ) = res.t.at<double>( j, 0 );
                }

                // cout << "Realtive Pose:\n" << node_i->relXform << endl << endl;

                node_i->absXform = node_im1->absXform * node_i->relXform;

                // cout << node_im1->absXform << "\nX\n" << node_i->relXform << "\n=\n" << node_i->absXform << endl << endl;

                est.generate_point_cloud( camInfo, res, zMin, zMax );
            }else{
                cout << "WARNING: Disjoint sequence created at index " << i <<"!" << endl;
            }
            node_i->imgRes2 = res;
        }
        
        rtnNds.push_back( NodePtr{ node_i } );
    }
    avgSharp /= (double) N;
    cout << endl << "Average Sharpness for Series: " << avgSharp << endl;
    return rtnNds;
}


PCPosPtr node_seq_to_PntPos_pcd( NodePtr firstNode, bool suppressCloud ){
    NodePtr  currNode = firstNode;
    PCPosPtr rtnCloud{ new PCPos{} };
    matXef   xform;
    while( currNode ){
        // 0. If there were points computed, Then get clouds
        if( currNode->imgRes2.success ){
            // 1. Store the relative cloud        
            currNode->imgRes2.absPCD = PCPosPtr{ new PCPos{} };
            // 2. Transform and Store the absolute cloud
            xform = OCV_matx_to_Eigen3_matx_f( currNode->absXform );
            // cout << endl << currNode->absXform << endl << xform << endl;
            transformPointCloud( *(currNode->imgRes2.relPCD), *(currNode->imgRes2.absPCD), xform );

            if( !suppressCloud ){
                // 3. Append points to the return cloud
                for( size_t i = 0; i < currNode->imgRes2.absPCD->size(); ++i ){
                    rtnCloud->push_back( (*(currNode->imgRes2.absPCD))[i] );
                }
            }
        }
        currNode = currNode->next;
    }
    return rtnCloud;
}


PCClrPtr colorize_node_seq_pcd( NodePtr firstNode, const Color& firstColor, const Color& lastColor, bool suppressCloud ){
    NodePtr  currNode = firstNode;
    PCClrPtr rtnCloud{ new PCClr{} };
    matXef   xform;
    size_t   N;
    size_t   i = 0;
    XColor   span = get_XColor( lastColor ) - get_XColor( firstColor );
    XColor   clr1 = get_XColor( firstColor );
    Color    clr_i;

    while( currNode ){
        ++N;
        currNode = currNode->next;
    }
    currNode = firstNode;
    while( currNode ){
        clr_i = get_Color( clr1 + span * (1.0*i/N) );
        // 0. If there were points computed, Then get clouds
        if( currNode->imgRes2.success ){
            // 1. Store the relative cloud        
            currNode->imgRes2.color   = clr_i;
            currNode->imgRes2.relCPCD = vec_PntPos_to_PntClr_pcd( currNode->imgRes2.relPCD, clr_i, false );
            currNode->imgRes2.absCPCD = vec_PntPos_to_PntClr_pcd( currNode->imgRes2.absPCD, clr_i, false );
            
            if( !suppressCloud ){
                // 3. Append points to the return cloud
                for( size_t i = 0; i < currNode->imgRes2.absCPCD->size(); ++i ){
                    rtnCloud->push_back( (*(currNode->imgRes2.absCPCD))[i] );
                }
            }
        }
        currNode = currNode->next;
        ++i;
    }
    return rtnCloud;
}