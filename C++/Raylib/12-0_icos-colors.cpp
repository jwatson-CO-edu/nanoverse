// g++ 12-0_icos-colors.cpp -std=c++17 -lraylib

////////// INIT ////////////////////////////////////////////////////////////////////////////////////

///// Imports ////////////////////////////////////
/// Standard ///
#include <iostream>
using std::cout, std::endl, std::ostream;
#include <algorithm> 
using std::min, std::max;
#include <set>
using std::set;
#include <map>
using std::map, std::pair;
#include <list>
using std::list;

/// Raylib ///
#include <raylib.h>
#include <raymath.h>
#include <rlgl.h>
#define RLIGHTS_IMPLEMENTATION

/// Local ///
#include "utils.hpp"
#include "rl_toybox.hpp"
#include "rlights.h"


///// Type Aliases ///////////////////////////////
typedef unsigned short  ushort;


class Icosahedron_r : public TriModel{ public:

    // ~ Constants ~
	float sqrt5 = sqrt( 5.0f ); // ----------------------------------- Square root of 5
	float phi   = ( 1.0f + sqrt5 ) * 0.5f; // ------------------------- The Golden Ratio
	float ratio = sqrt( 10.0f + ( 2.0f * sqrt5 ) ) / ( 4.0f * phi ); // ratio of edge length to radius
	
	// ~ Variables ~
	Vector3 center;
	float   radius;
	float   a; 
	float   b; 
    vvec3   V;

    Icosahedron_r( float rad , const Vector3& cntr ) : TriModel(20){
        center = cntr;
        radius = rad;
        a /**/ = ( radius / ratio ) * 0.5;
        b /**/ = ( radius / ratio ) / ( 2.0f * phi );

        // Define the icosahedron's 12 vertices:
        V.push_back( Vector3Add( center, Vector3{  0,  b, -a } ) );
        V.push_back( Vector3Add( center, Vector3{  b,  a,  0 } ) );
        V.push_back( Vector3Add( center, Vector3{ -b,  a,  0 } ) );
        V.push_back( Vector3Add( center, Vector3{  0,  b,  a } ) );
        V.push_back( Vector3Add( center, Vector3{  0, -b,  a } ) );
        V.push_back( Vector3Add( center, Vector3{ -a,  0,  b } ) );
        V.push_back( Vector3Add( center, Vector3{  0, -b, -a } ) );
        V.push_back( Vector3Add( center, Vector3{  a,  0, -b } ) );
        V.push_back( Vector3Add( center, Vector3{  a,  0,  b } ) );
        V.push_back( Vector3Add( center, Vector3{ -a,  0, -b } ) );
        V.push_back( Vector3Add( center, Vector3{  b, -a,  0 } ) );
        V.push_back( Vector3Add( center, Vector3{ -b, -a,  0 } ) );

        // Define the icosahedron's 20 triangular faces:
        load_tri( V[ 2], V[ 1], V[ 0] );
        load_tri( V[ 1], V[ 2], V[ 3] );
        load_tri( V[ 5], V[ 4], V[ 3] );
        load_tri( V[ 4], V[ 8], V[ 3] );
        load_tri( V[ 7], V[ 6], V[ 0] );
        load_tri( V[ 6], V[ 9], V[ 0] );
        load_tri( V[11], V[10], V[ 4] );
        load_tri( V[10], V[11], V[ 6] );
        load_tri( V[ 9], V[ 5], V[ 2] );
        load_tri( V[ 5], V[ 9], V[11] );
        load_tri( V[ 8], V[ 7], V[ 1] );
        load_tri( V[ 7], V[ 8], V[10] );
        load_tri( V[ 2], V[ 5], V[ 3] );
        load_tri( V[ 8], V[ 1], V[ 3] );
        load_tri( V[ 9], V[ 2], V[ 0] );
        load_tri( V[ 1], V[ 7], V[ 0] );
        load_tri( V[11], V[ 9], V[ 6] );
        load_tri( V[ 7], V[10], V[ 6] );
        load_tri( V[ 5], V[11], V[ 4] );
        load_tri( V[10], V[ 8], V[ 4] );
    }
};