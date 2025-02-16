////////// INIT ////////////////////////////////////////////////////////////////////////////////////
#include "SfM.hpp"



////////// PCD CLEANUP /////////////////////////////////////////////////////////////////////////////

PCPosPtr position_only( PCClrPtr clrCloud ){
    // Strip color information from the cloud
    PCPosPtr rtnCloud{ new PCPos{} };
    for( const PntClr& pnt : *clrCloud ){  rtnCloud->push_back(  PntPos{ pnt.x, pnt.y, pnt.z }  );  }
    return rtnCloud;
}


void find_ground_plane_RANSAC( PCClrPtr cloud, double dThresh ){
    // Attempt to fit a plane to the points

    // Create segmentation object
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    seg.setMethodType( pcl::SAC_RANSAC );
    seg.setOptimizeCoefficients( true );
    seg.setModelType( pcl::SACMODEL_PLANE );
    seg.setDistanceThreshold( dThresh ); // Adjust as needed

    // Segment the plane
    IndicesPtr /*------------*/ inliers{ new Indices };
    pcl::ModelCoefficients::Ptr coefficients( new pcl::ModelCoefficients );
    seg.setInputCloud( position_only( cloud ) );
    seg.segment( *inliers, *coefficients );
}