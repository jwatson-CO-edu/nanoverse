// g++ 12-1_rlBegin-Plume.cpp -std=c++17 -lraylib

////////// INIT ////////////////////////////////////////////////////////////////////////////////////

///// Imports ////////////////////////////////////
/// Standard ///
#include <iostream>
using std::cout, std::endl, std::ostream;
#include <algorithm> 
using std::min, std::max;
#include <deque>
using std::deque;

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


////////// CLASSES /////////////////////////////////////////////////////////////////////////////////

class Plume{ public:
    // A streaking ribbon with decreasing opacity from head to tail to simulate exhaust
    // NOTE: This class assumes that backface culling is turned OFF

    /// Members ///
    uint /*--------------*/ Npairs; // -- Number of coordinate pairs allowed
    Color /*-------------*/ color; // --- Base color of the plume
    float /*-------------*/ headAlpha; // Beginning opacity
    float /*-------------*/ tailAlpha; // Ending    opacity
    deque<array<Vector3,2>> coords; // -- Ribbon data, listed from head to tail

    /// Constructors ///

    Plume( uint N, Color c, float Ah, float At ){
        Npairs    = N;
        color     = c;
        headAlpha = Ah;
        tailAlpha = At;
    }

    /// Methods ///

    void push_coord_pair( const Vector3& c1, const Vector3& c2 ){
        // Add coordinates to the head of the plume
        if( coords.size() >= Npairs )  coords.pop_back(); // If queue is full, drop the tail element
        coords.push_front(  array<Vector3,2>{ c1, c2 }  );
    }

    void draw(){
        // Render plume as a batch job
        uint    Nsize = coords.size()-1;        
        float   R     = color.r/255.0f;
        float   G     = color.g/255.0f;
        float   B     = color.b/255.0f;
        float   Aspan = headAlpha - tailAlpha;
        Vector3 c1, c2, c3, c4;
        float   A_i;
        float   A_ip1;
        
        // Begin triangle batch job
        rlBegin( RL_TRIANGLES );

        for( uint i = 0; i < Nsize; i++){
            c1    = coords[i  ][0];
            c2    = coords[i  ][1];
            c3    = coords[i+1][0];
            c4    = coords[i+1][1];
            A_i   = tailAlpha + Aspan*(Nsize-(i  ))/(1.0f*Nsize);
            A_ip1 = tailAlpha + Aspan*(Nsize-(i+1))/(1.0f*Nsize);

            if( i%2==0 ){
                /// Triangle 1: c2, c1, c3 ///
                // t1.p1 //
                rlColor4f(R, G, B, A_i);
                rlVertex3f(c2.x, c2.y, c2.z);
                // t1.p2 //
                rlVertex3f(c1.x, c1.y, c1.z);
                // t1.p3 //
                rlColor4f(R, G, B, A_ip1);
                rlVertex3f(c3.x, c3.y, c3.z);

                /// Triangle 2: c3, c4, c2 ///
                // t2.p1 //
                rlVertex3f(c3.x, c3.y, c3.z);
                // t2.p2 //
                rlVertex3f(c4.x, c4.y, c4.z);
                // t2.p3 //
                rlColor4f(R, G, B, A_i);
                rlVertex3f(c2.x, c2.y, c2.z);
            }else{
                /// Triangle 1: c1, c3, c4 ///
                // t1.p1 //
                rlColor4f(R, G, B, A_i);
                rlVertex3f(c1.x, c1.y, c1.z);
                // t1.p2 //
                rlColor4f(R, G, B, A_ip1);
                rlVertex3f(c3.x, c3.y, c3.z);
                // t1.p3 //
                rlVertex3f(c4.x, c4.y, c4.z);

                /// Triangle 2: c4, c2, c1 ///
                // t2.p1 //
                rlVertex3f(c4.x, c4.y, c4.z);
                // t2.p2 //
                rlColor4f(R, G, B, A_i);
                rlVertex3f(c2.x, c2.y, c2.z);
                // t2.p3 //
                rlVertex3f(c1.x, c1.y, c1.z);
            }
        }
        // End triangle batch job
        rlEnd();
    }
};



////////// MAIN ////////////////////////////////////////////////////////////////////////////////////

int main(){

    rand_seed();

    /// Window Init ///
    InitWindow( 600, 600, "Plume!" );
    SetTargetFPS( 60 );
    rlEnableSmoothLines();
    rlDisableBackfaceCulling();

    // Camera
    Camera camera = Camera{
        Vector3{ 15.0, 15.0, 50.0 }, // Position
        Vector3{ 15.0, 15.0,  0.0 }, // Target
        Vector3{  0.0,  1.0,  0.0 }, // Up
        45.0, // ---------------------- FOV_y
        0 // -------------------------- Projection mode
    };

    Plume plume{ 6, BLUE, 1.0f, 0.0f };
    plume.push_coord_pair( Vector3{  0.0f, 0.0f, 0.0f }, Vector3{  0.0f, 30.0f, 0.0f } );
    plume.push_coord_pair( Vector3{  6.0f, 0.0f, 0.0f }, Vector3{  6.0f, 30.0f, 0.0f } );
    plume.push_coord_pair( Vector3{ 12.0f, 0.0f, 0.0f }, Vector3{ 12.0f, 30.0f, 0.0f } );
    plume.push_coord_pair( Vector3{ 18.0f, 0.0f, 0.0f }, Vector3{ 18.0f, 30.0f, 0.0f } );
    plume.push_coord_pair( Vector3{ 24.0f, 0.0f, 0.0f }, Vector3{ 24.0f, 30.0f, 0.0f } );
    plume.push_coord_pair( Vector3{ 30.0f, 0.0f, 0.0f }, Vector3{ 30.0f, 30.0f, 0.0f } );

    while( !WindowShouldClose() ){
        
        /// Begin Drawing ///
        BeginDrawing();
        BeginMode3D( camera );
        ClearBackground( BLACK );

        ///// DRAW LOOP //////////////////////////
        plume.draw();

        /// End Drawing ///
        EndMode3D();
        EndDrawing();
    }

    // UnloadModel( icos.model );
    // UnloadMesh( icos.mesh );

    return 0;
}