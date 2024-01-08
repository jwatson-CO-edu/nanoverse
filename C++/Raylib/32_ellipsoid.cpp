// g++ 31_star-map.cpp -std=c++17 -lraylib -O3 -o starMap.out
// Animated star-map with normal maps, Inspired by the Ahsoka end-credits sequence


////////// INIT ////////////////////////////////////////////////////////////////////////////////////

///// Imports ////////////////////////////////////

/// Local ///
#include "rl_toybox.hpp" // Also includes "utils.cpp"



////////// HELPERS /////////////////////////////////////////////////////////////////////////////////



////////// TOYS ////////////////////////////////////////////////////////////////////////////////////

class Ellipsoid : public DynaMesh { public:
    // An ellipsoid constructed from a subdivided icosahedron

    /// Members ///
    Vector3 radii;

    /// Constructors ///
    Ellipsoid( float xRad, float yRad, float zRad, const Vector3& cntr, ubyte div = 3, Color color = BLUE ) : 
            DynaMesh( 20 * (div*(div+1)/2 + (div-1)*(div)/2) ) {
        // Create a 3D ellipsoid as a stack of ellipses
        radii = {xRad, yRad, zRad};
        Sphere  s{ max( max( xRad, yRad ), zRad ) , cntr, div, color };

        Vector3 point;
        float   phi, theta;
        float   elScl, zScl;
        ubyte   i;
        triPnts facet;

        for( triPnts& tri : s.tris ){
            i = 0;
            for( Vector3& pnt : tri ){
                // Get the angle from vertical
                phi   = atan2f( sqrt( pnt.x * pnt.x + pnt.y * pnt.y ), pnt.z );
                elScl = cosf( phi );
                zScl  = sinf( phi );
                // Get the angle in the XY-plane
                theta = atan2f( pnt.y, pnt.x );
                // Set the point
                facet[i] = {elScl*xRad*cosf(theta), elScl*yRad*sinf(theta), zScl*zRad};
                i++;
            }
            push_triangle_w_norms( facet );
        }

        // Set color
        set_uniform_color( color );
        load_mesh_buffers( true, true );
    }
};