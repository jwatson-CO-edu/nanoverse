// g++ 01-1_hello-colors.cpp -std=gnu++17 -lraylib

////////// INIT ////////////////////////////////////////////////////////////////////////////////////

/// Raylib Imports ///
#include "raylib.h"
/// Local Imports ///
#include "rl_toybox.hpp"

// settings
const unsigned int SCR_WIDTH  = 800;
const unsigned int SCR_HEIGHT = 600;

int main(){

    ///// Graphics Setup //////////////////////////////////////////////////

    InitWindow( 600, 600, "Hello, Colors!" );
    SetTargetFPS( 60 );

    TriModel tri{1};
    tri.load_tri(
        Vector3{  0.0, 30.0, 0.0 }, 
        Vector3{ 30.0,  0.0, 0.0 }, 
        Vector3{ 30.0, 30.0, 0.0 }
    );
    tri.set_XYZ( 0.0f, 0.0f, 0.0f );

    // Vertex colors
    uint Nvrt = 3;
    tri.mesh.colors = (ubyte *)MemAlloc(Nvrt*4*sizeof(ubyte)); // 3 vertices, 4 coordinates each (r,g,b,a)
    tri.mesh.colors[0] = 255;  tri.mesh.colors[1] =   0;  tri.mesh.colors[ 2] =   0;  tri.mesh.colors[ 3] = 255;  
    tri.mesh.colors[4] =   0;  tri.mesh.colors[5] = 255;  tri.mesh.colors[ 6] =   0;  tri.mesh.colors[ 7] = 255;  
    tri.mesh.colors[8] =   0;  tri.mesh.colors[9] =   0;  tri.mesh.colors[10] = 255;  tri.mesh.colors[11] = 255;  
    tri.load_geo();

    // Camera
    Camera camera = Camera{
        Vector3{ 15.0, 15.0, 50.0 }, // Position
        Vector3{ 15.0, 15.0,  0.0 }, // Target
        Vector3{  0.0,  1.0,  0.0 }, // Up
        45.0, // ---------------------- FOV_y
        0 // -------------------------- Projection mode
    };

    ///// Drawing Loop ////////////////////////////////////////////////////
    
    while( !WindowShouldClose() ){
        BeginDrawing();
        BeginMode3D( camera );
        ClearBackground( BLACK );
        // BeginShaderMode( hello );

            // setModelShader( &tri.model, &hello );
            tri.draw();

        // EndShaderMode();
        EndMode3D();

        DrawFPS( 30, 60 );
        DrawText( "Using vertex colors", 30, 30, 20, LIGHTGRAY );

        EndDrawing();
    }

    ///// Cleanup /////////////////////////////////////////////////////////

    CloseWindow();

    return 0;
}