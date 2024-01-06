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
    Ellipsoid( float xRad, float yRad, float zRad, const Vector3& cntr, ubyte div = 3, Color color = BLUE ) : 
            DynaMesh( 20 * (div*(div+1)/2 + (div-1)*(div)/2) ) {
        // Create a 3D ellipsoid as a stack of ellipses
        Sphere s{ max( max( xRad, yRad ), zRad ) , cntr, div, color };
        for( triPnts& tri : s.tris ){
            // Get the angle in the X-Z Plane
        }
    }
};