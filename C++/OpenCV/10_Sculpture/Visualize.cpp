////////// INIT ////////////////////////////////////////////////////////////////////////////////////
#include "SfM.hpp"



////////// PCD VIZ /////////////////////////////////////////////////////////////////////////////////

pcl::visualization::PCLVisualizer::Ptr simpleVis( PCXYZPtr cloud ){
    // --------------------------------------------
    // -----Open 3D viewer and add point cloud-----
    // --------------------------------------------
    // Source: https://github.com/PointCloudLibrary/pcl/blob/master/doc/tutorials/content/sources/pcl_visualizer/pcl_visualizer_demo.cpp
    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer( "3D Viewer" ));
    viewer->setBackgroundColor( 0, 0, 0 );
    viewer->addPointCloud<pcl::PointXYZ>( cloud, "sample cloud" );
    viewer->setPointCloudRenderingProperties( pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud" );
    viewer->addCoordinateSystem( 1.0 );
    viewer->initCameraParameters( );
    return ( viewer );
}