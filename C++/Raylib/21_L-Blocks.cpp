// g++ 21_L-Blocks.cpp -std=c++17 -lraylib -O3
// Building as an L-System


////////// INIT ////////////////////////////////////////////////////////////////////////////////////

///// Imports ////////////////////////////////////

/// Local ///
#include "rl_toybox.hpp"



////////// STRUCTS /////////////////////////////////////////////////////////////////////////////////

class Cuboid : public DynaMesh{

    vvec3 V;
    Color colr;

    Cuboid( float xLen, float yLen, float zLen, Color color ) : DynaMesh( 12 ){
        float xHalf = xLen/2.0;
        float yHalf = yLen/2.0;
        float zHalf = zLen/2.0;
        colr = color;

        // 1. Establish vertices
        V.push_back( Vector3{ -xHalf, -yHalf, -zHalf } );
        V.push_back( Vector3{ -xHalf, -yHalf,  zHalf } );
        V.push_back( Vector3{ -xHalf,  yHalf, -zHalf } );
        V.push_back( Vector3{ -xHalf,  yHalf,  zHalf } );
        V.push_back( Vector3{  xHalf, -yHalf, -zHalf } );
        V.push_back( Vector3{  xHalf, -yHalf,  zHalf } );
        V.push_back( Vector3{  xHalf,  yHalf, -zHalf } );
        V.push_back( Vector3{  xHalf,  yHalf,  zHalf } );

        // 2. Build tris
        push_triangle_w_norms( { V[0], V[3], V[2] } );
        push_triangle_w_norms( { V[0], V[1], V[3] } );
        push_triangle_w_norms( { V[6], V[4], V[0] } );
        push_triangle_w_norms( { V[6], V[0], V[2] } );
        push_triangle_w_norms( { V[0], V[4], V[5] } );
        push_triangle_w_norms( { V[0], V[5], V[1] } );
        push_triangle_w_norms( { V[7], V[6], V[2] } );
        push_triangle_w_norms( { V[7], V[2], V[3] } );
        push_triangle_w_norms( { V[4], V[6], V[7] } );
        push_triangle_w_norms( { V[4], V[7], V[5] } ); 
        push_triangle_w_norms( { V[1], V[5], V[7] } );
        push_triangle_w_norms( { V[1], V[7], V[3] } );

        // 3. Set color
        for( ubyte i = 0; i < 12; ++i ){  clrs.push_back( {colr, colr, colr} );  }
    }
};