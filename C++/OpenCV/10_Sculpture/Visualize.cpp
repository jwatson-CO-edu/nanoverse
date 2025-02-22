////////// INIT ////////////////////////////////////////////////////////////////////////////////////
#include "SfM.hpp"



////////// PCD VIZ /////////////////////////////////////////////////////////////////////////////////

VizPtr simpleVis( PCPosPtr cloud ){
    // --------------------------------------------
    // -----Open 3D viewer and add point cloud-----
    // --------------------------------------------
    // Source: https://github.com/PointCloudLibrary/pcl/blob/master/doc/tutorials/content/sources/pcl_visualizer/pcl_visualizer_demo.cpp
    VizPtr viewer (new pcl::visualization::PCLVisualizer( "3D Viewer" ));
    viewer->setBackgroundColor( 0, 0, 0 );
    viewer->addPointCloud<pcl::PointXYZ>( cloud, "sample cloud" );
    viewer->setPointCloudRenderingProperties( pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud" );
    viewer->addCoordinateSystem( 1.0 );
    viewer->initCameraParameters( );
    return viewer;
}


VizPtr rgbaVis( PCClrPtr cloud ){
    // --------------------------------------------
    // -----Open 3D viewer and add point cloud-----
    // --------------------------------------------
    // Source: https://github.com/PointCloudLibrary/pcl/blob/master/doc/tutorials/content/sources/pcl_visualizer/pcl_visualizer_demo.cpp
    VizPtr viewer( new VizPCL( "3D Viewer" ) );
    viewer->setBackgroundColor( 0, 0, 0 );
    pcl::visualization::PointCloudColorHandlerRGBField<PntClr> rgb( cloud );
    viewer->addPointCloud<PntClr>( cloud, rgb, "sample cloud" );
    viewer->setPointCloudRenderingProperties( pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud" );
    viewer->addCoordinateSystem( 1.0 );
    viewer->initCameraParameters();
    return viewer;
}

pcl::ModelCoefficients get_coeffs( const vector<double>& coeffsVec ){
    pcl::ModelCoefficients rtnCoeffs{};
    rtnCoeffs.header.frame_id = "default";
    rtnCoeffs.header.seq /**/ = 0;
    rtnCoeffs.header.stamp    = 0;
    for( const double& val : coeffsVec ){  rtnCoeffs.values.push_back( (float) val );  }
    return rtnCoeffs;
}

    

////////// STRUCTURE VIZ ///////////////////////////////////////////////////////////////////////////

VizPtr viz_current_soln( const NodePtr firstNode ){
    VizPtr   viewer{ new VizPCL( "3D Viewer" ) };
    NodePtr  currNode = firstNode;
    PCClrPtr totCloud = PCClrPtr{ new PCClr{} };
    size_t   i /*--*/ = 0;
    
    while( currNode ){
        if( currNode->imgRes2.success ){
            
            *totCloud += *(currNode->imgRes2.absCPCD);
            for( const PlanePoints& plane : currNode->imgRes2.removedPlanes ){
                viewer->addPlane( get_coeffs( plane.planeEq ), string_format( "plane_%lu", i ) );
            }
        }
        currNode = currNode->next;
        ++i;
    }

    cout << "Total cloud has " << totCloud->size() << " points!, About to display ..." << endl;

    viewer->addPointCloud( totCloud );
    return viewer;
}