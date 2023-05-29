// g++ 12-2_icos-party.cpp -std=c++17 -lraylib

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
#include <deque>
using std::deque;
#include <memory>
using std::shared_ptr;

/// Raylib ///
#include <raylib.h>
#include <raymath.h>
#include <rlgl.h>
// #define RLIGHTS_IMPLEMENTATION

/// Local ///
#include "utils.hpp"
#include "rl_toybox.hpp"
// #include "rlights.h"


///// Type Aliases ///////////////////////////////
typedef unsigned short /*--*/ ushort;



////////// TOYS ////////////////////////////////////////////////////////////////////////////////////

class Icosahedron_r : public TriModel{ public:

    // ~ Constants ~
	float sqrt5 = sqrt( 5.0f ); // ----------------------------------- Square root of 5
	float phi   = ( 1.0f + sqrt5 ) * 0.5f; // ------------------------- The Golden Ratio
	float ratio = sqrt( 10.0f + ( 2.0f * sqrt5 ) ) / ( 4.0f * phi ); // ratio of edge length to radius
	
	// ~ Variables ~
	float radius;
	float a; 
	float b; 
    vvec3 V;

    // ~ Appearance ~
    Color baseClr;
    Color lineClr;

    // ~ Appearance ~
    bool  anim;
    float rolVel;
    float ptcVel;
    float yawVel;

    Icosahedron_r( float rad , const Vector3& cntr, bool active = true ) : TriModel(20){
        // Compute the vertices and faces
        XYZ    = cntr;
        radius = rad;
        a /**/ = ( radius / ratio ) * 0.5;
        b /**/ = ( radius / ratio ) / ( 2.0f * phi );

        // ~ Appearance ~
        baseClr = BLACK;
        lineClr = WHITE;

        // ~ Animation ~
        anim = active;
        float loRate = -0.01f;
        float hiRate =  0.01f;
        rolVel = randf( loRate, hiRate );
        ptcVel = randf( loRate, hiRate );
        yawVel = randf( loRate, hiRate );

        // Define the icosahedron's 12 vertices:
        V.push_back( Vector3{  0,  b, -a } );
        V.push_back( Vector3{  b,  a,  0 } );
        V.push_back( Vector3{ -b,  a,  0 } );
        V.push_back( Vector3{  0,  b,  a } );
        V.push_back( Vector3{  0, -b,  a } );
        V.push_back( Vector3{ -a,  0,  b } );
        V.push_back( Vector3{  0, -b, -a } );
        V.push_back( Vector3{  a,  0, -b } );
        V.push_back( Vector3{  a,  0,  b } );
        V.push_back( Vector3{ -a,  0, -b } );
        V.push_back( Vector3{  b, -a,  0 } );
        V.push_back( Vector3{ -b, -a,  0 } );

        // Define the icosahedron's 20 triangular faces: CCW-out
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


    void update(){  rotate_RPY( rolVel, ptcVel, yawVel );  } // Rotate
    

    ///// Rendering //////////////////////////////
    // WARNING: Requires window init to call!

    void load_geo(){
        // Get the model ready for drawing
        build_mesh_unshared();
        // build_normals_flat_unshared();
        load_mesh();
    }

    void draw(){
        // Draw the model
        if( anim ){  update();  }
        DrawModel(      model, XYZ, 1.00, baseClr );  
        DrawModelWires( model, XYZ, 1.02, lineClr );
    }
};

class TestClass{ public:
    vector<shared_ptr<Icosahedron_r>> icosPtrs;

    TestClass( float rad , const Vector3& cntr, bool active = true ){
        icosPtrs.push_back( shared_ptr<Icosahedron_r>( new Icosahedron_r{ rad, cntr, active } ) );
    }

    void load_geo(){
        for( shared_ptr<Icosahedron_r> icos : icosPtrs ){  icos->load_geo();  }  
    }

    void draw(){
        for( shared_ptr<Icosahedron_r> icos : icosPtrs ){  icos->draw();  }  
    }
};


////////// MAIN ////////////////////////////////////////////////////////////////////////////////////

int main(){

    rand_seed();

    Icosahedron_r /*-------------*/ stackIcos{ 10.0f, Vector3{ 5.0f, 5.0f, 0.0f } };
    Icosahedron_r* /*------------*/ rawIcos = new Icosahedron_r{ 10.0f, Vector3{ 5.0f, 25.0f, 0.0f } };
    shared_ptr<Icosahedron_r> /*-*/ smartIcos = shared_ptr<Icosahedron_r>( new Icosahedron_r{ 10.0f, Vector3{ 25.0f, 25.0f, 0.0f } } );
    map<uint,shared_ptr<TestClass>> testMap;
    testMap[0] = shared_ptr<TestClass>( new TestClass{ 10.0f, Vector3{ 25.0f, 5.0f, 0.0f } } );

    /// Window Init ///
    InitWindow( 600, 600, "Icosahedron Party!" );
    SetTargetFPS( 60 );
    rlEnableSmoothLines();
    rlDisableBackfaceCulling();


    stackIcos.load_geo();
    rawIcos->load_geo();
    smartIcos->load_geo();
    for( pair<uint,shared_ptr<TestClass>> elem : testMap ){  elem.second->load_geo();  }

    // Camera
    Camera camera = Camera{
        Vector3{ 15.0, 15.0, 50.0 }, // Position
        Vector3{ 15.0, 15.0,  0.0 }, // Target
        Vector3{  0.0,  1.0,  0.0 }, // Up
        45.0, // ---------------------- FOV_y
        0 // -------------------------- Projection mode
    };


    ////////// RENDER LOOP /////////////////////////////////////////////////////////////////////////

    while( !WindowShouldClose() ){

        /// Begin Drawing ///
        BeginDrawing();
        BeginMode3D( camera );
        ClearBackground( BLUE );

        ///// DRAW LOOP //////////////////////////
        stackIcos.draw();
        rawIcos->draw();
        smartIcos->draw();
        for( pair<uint,shared_ptr<TestClass>> elem : testMap ){  elem.second->draw();  }

        /// End Drawing ///
        EndMode3D();
        EndDrawing();
    }


    return 0;
}