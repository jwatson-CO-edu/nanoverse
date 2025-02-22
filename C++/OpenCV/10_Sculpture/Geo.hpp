#ifndef GEO_HPP
#define GEO_HPP

////////// INIT ////////////////////////////////////////////////////////////////////////////////////

///// Includes /////

/// Local ///
#include "SfM.hpp"
#include "Delaunay.hpp"



////////// 2D Mesh (double) ////////////////////////////////////////////////////////////////////////

class DynVF2d{ public: 
    // Dynamic 2D Mesh (w/ Adjacency)
    vector<vec2d> V; // Vertices
    vector<vec3u> F; // Triangles
    vector<vec3u> A; // Triangle Adjacencies

    DynVF2d( const vector<vec2d>& V_, const vector<vec3u>& F_ );

    size_t size();
    void   calc_adjacencies();
};


// Triangulate the collection of points into a 2D triangle mesh
DynVF2d Delaunay_from_points( const vector<vec2d>& pnts );
vector<vec2d> cv_pnts_to_V_points( const vector<Point2d>& pnts ); // Save OpenCV points as native format
// Triangulate the collection of keypoints into a 2D triangle mesh
DynVF2d Delaunay_from_keypoints( const vector<Point2d>& pnts );



////////// END /////////////////////////////////////////////////////////////////////////////////////

#endif