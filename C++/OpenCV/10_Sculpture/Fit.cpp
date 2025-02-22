////////// INIT ////////////////////////////////////////////////////////////////////////////////////
#include "SfM.hpp"



////////// PCD CLEANUP /////////////////////////////////////////////////////////////////////////////

PCPosPtr position_only( PCClrPtr clrCloud ){
    // Strip color information from the cloud
    PCPosPtr rtnCloud{ new PCPos{} };
    for( const PntClr& pnt : *clrCloud ){  rtnCloud->push_back(  PntPos{ pnt.x, pnt.y, pnt.z }  );  }
    return rtnCloud;
}


vector<double> get_coeffs( pcl::ModelCoefficients::Ptr PCLcoeff ){
    vector<double> rtnCoeffs;
    for( float& val : PCLcoeff->values ){  rtnCoeffs.push_back( (double) val );  }
    return rtnCoeffs;
}


void remove_one_plane_RANSAC( TwoViewResult& res, double dThresh ){
    // Attempt to fit a plane to the points
    PCPosPtr /*--------------*/ remCloud = PCPosPtr{ new PCPos{} };
    pcl::ExtractIndices<PntPos> extractor;
    IndicesPtr /*------------*/ inliers{ new Indices };
    pcl::ModelCoefficients::Ptr coefficients( new pcl::ModelCoefficients );
    
    extractor.setInputCloud( res.relPCD );

    // Create segmentation object
    pcl::SACSegmentation<PntPos> seg;
    seg.setMethodType( pcl::SAC_RANSAC );
    seg.setOptimizeCoefficients( true );
    seg.setModelType( pcl::SACMODEL_PLANE );
    seg.setDistanceThreshold( dThresh ); // Adjust as needed

    // Segment the plane
    
    seg.setInputCloud( res.relPCD );
    seg.segment( *inliers, *coefficients );
    
    // Extract **Inliers**
    extractor.setNegative( false ); // To extract points in the indices
    extractor.setIndices( inliers );
    extractor.filter( *remCloud );
    res.removedPlanes.push_back( PlanePoints{
        get_coeffs( coefficients ),
        dThresh,
        remCloud
    } );

    // Extract **Outliers**
    extractor.setNegative( true ); // To extract points NOT in the indices
    extractor.setIndices( inliers );
    extractor.filter( *res.relPCD );
    cout << "Removed:Retained " << inliers->indices.size() << ":" << res.relPCD->size() << " points!" << endl;
}


void remove_major_planes_from_clouds( const NodePtr firstNode, double dThresh ){
    // Remove a plane from each of the relative point clouds
    NodePtr currNode = firstNode;
    size_t  i /*--*/ = 0;
    while( currNode ){
        cout << "Processing node " << i << " ... ";
        // 0. If there were points computed, Then prune cloud
        if( currNode->imgRes2.success ){
            remove_one_plane_RANSAC( currNode->imgRes2, dThresh );
        }
        cout << "DONE!" << endl;
        currNode = currNode->next;
        ++i;
    }
}