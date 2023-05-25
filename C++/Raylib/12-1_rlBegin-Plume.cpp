// g++ 12-1_icos-colors.cpp -std=c++17 -lraylib

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
        uint    Nsize = coords.size();
        Vector3 c1, c2, c3, c4;
        float   R  = color.r/255.0f;
        float   G  = color.g/255.0f;
        float   B  = color.b/255.0f;
        float   Aspan = headAlpha - tailAlpha;
        float   A_i;
        float   A_ip1;
        
        // Begin triangle batch job
        rlBegin( RL_TRIANGLES );

        for( uint i = 0; i < (Nsize-1); i++){
            c1    = coords[i  ][0];
            c2    = coords[i  ][1];
            c3    = coords[i+1][0];
            c4    = coords[i+1][1];
            A_i   = tailAlpha + Aspan*(Npairs-(i+1))/(1.0f*Npairs);
            A_ip1 = tailAlpha + Aspan*(Npairs-(i+2))/(1.0f*Npairs);

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
        }

        // End triangle batch job
        rlEnd();
    }
};



////////// MAIN ////////////////////////////////////////////////////////////////////////////////////

int main(){

    rand_seed();

    const Vector2 res{ 1200, 600 };
    // Icosahedron_r icos{ 10.0f, Vector3{0,0,0} };
    // Camera
    Camera camera{
        Vector3{ 30.0, 30.0, 30.0 }, // Position
        Vector3{  0.0,  0.0,  0.0 }, // Target
        Vector3{  0.0,  0.0,  1.0 }, // Up
        45.0, // -------------------- FOV_y
        0 // ------------------------ Projection mode
    };

    /// Window Init ///
    InitWindow( (int) res.x, (int) res.y, "ICOSAHEDRON" );
    SetTargetFPS( 60 );
    rlEnableSmoothLines();
    // rlDisableBackfaceCulling();

     ////////// Shader Init: Pre-Window /////////////////////////////////////////////////////////////

    // Fade shader
    // cout << "About to load shader ..." << endl;
    Shader fade = LoadShader( "shaders/12-1_fade-test.vs", "shaders/12-1_fade-test.fs" );
    // cout << "Shader loaded!" << endl;
    
    // cout << "About to get shader location ..." << endl;
    fade.locs[SHADER_LOC_MATRIX_MODEL] = GetShaderLocation( fade, "matModel" );
    

    // RenderTexture2D target = LoadRenderTexture( res.x, res.y );

    // icos.load_geo();

    while( !WindowShouldClose() ){
        
        /// Begin Drawing ///
        BeginDrawing();
        BeginMode3D( camera );
        ClearBackground( BLACK );

        ///// DRAW LOOP //////////////////////////
        // setModelShader( &icos.model, &fade );
        // icos.draw();

        /// End Drawing ///
        EndMode3D();
        EndDrawing();
    }

    // UnloadModel( icos.model );
    // UnloadMesh( icos.mesh );

    return 0;
}