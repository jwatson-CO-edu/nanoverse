////////// INIT ////////////////////////////////////////////////////////////////////////////////////
#include "Geo.hpp"



////////// 2D Mesh (double) ////////////////////////////////////////////////////////////////////////

DynVF2d::DynVF2d( const vector<vec2d>& V_, const vector<vec3u>& F_ ){
    V = V_;
    F = F_;
}


size_t DynVF2d::size(){  return F.size();  }


DynVF2d Delaunay_from_points( const vector<vec2d>& pnts ){
    // Triangulate the collection of points into a 2D triangle mesh
    size_t     N    = pnts.size(); // ---------- Number of points
    XYZ* /*-*/ pxyz = new XYZ[ N + 3 ]; // ----- The vertex array pxyz must be big enough to hold 3 more points than N
    ITRIANGLE* f    = new ITRIANGLE[ 3 * N ]; // The triangle array 'v' should be malloced to 3 * N
    int /*- */ ntri = 0; // -------------------- Number of resulting triangles
    vector<vec2d> V;  V.reserve(N);
    vector<vec3u> F;
    for( size_t i = 0; i < N; ++i ){  pxyz[i] = XYZ{ pnts[i][0], pnts[i][1] };  }
    
    // The vertex array must be sorted in increasing x values say: qsort(p,nv,sizeof(XYZ),XYZCompare);
    qsort( pxyz, N, sizeof( XYZ ) , XYZCompare );
    for( size_t i = 0; i < N; ++i ){  V.push_back( vec2d{ pnts[i][0], pnts[i][1] } );  }

    Triangulate( N, pxyz, f, ntri );

    for( int i = 0 ; i < ntri ; i++ ){  F.push_back( vec3u{ (size_t) f[i].p1, (size_t) f[i].p2, (size_t) f[i].p3 } );  }

    return DynVF2d{ V, F };
}


vector<vec2d> cv_pnts_to_V_points( const vector<Point2d>& pnts ){
    // Save OpenCV points as native format
    vector<vec2d> rtnPts;  
    rtnPts.reserve( pnts.size() );
    for( const Point2d& cvPnt : pnts ){  rtnPts.push_back( vec2d{ cvPnt.x, cvPnt.y } );  }
    return rtnPts;
}


DynVF2d Delaunay_from_keypoints( const vector<Point2d>& pnts ){  
    // Triangulate the collection of keypoints into a 2D triangle mesh
    return Delaunay_from_points( cv_pnts_to_V_points( pnts ) );
}