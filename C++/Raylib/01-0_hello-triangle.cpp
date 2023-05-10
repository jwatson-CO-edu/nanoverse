// g++ 01-0_hello-triangle.cpp -lraylib

/// Raylib Imports ///
#include "raylib.h"
/// Local Imports ///
#include "rl_toybox.hpp"

// settings
const unsigned int SCR_WIDTH  = 800;
const unsigned int SCR_HEIGHT = 600;

int main(){

    InitWindow( 600, 600, "Hello, Triangle!" );
    SetTargetFPS( 60 );

    Shader hello = LoadShader( "shaders/hello.vs", "shaders/hello.fs" );
    TriModel tri{1};
    tri.load_tri(
        Vector3{  0.0, 30.0, 0.0 }, 
        Vector3{ 30.0,  0.0, 0.0 }, 
        Vector3{ 30.0, 30.0, 0.0 }
    );
    tri.set_XYZ( 0.0f, 0.0f, 0.0f );
    tri.load_geo();

    // Camera
    Camera camera = Camera{
        Vector3{ 15.0, 15.0, 50.0 }, // Position
        Vector3{ 15.0, 15.0,  0.0 }, // Target
        Vector3{  0.0,  1.0,  0.0 }, // Up
        45.0, // -------------------- FOV_y
        0 // ------------------------ Projection mode
    };

    while( !WindowShouldClose() ){
        BeginDrawing();
        BeginMode3D( camera );
        ClearBackground( BLACK );
        // BeginShaderMode( hello );

            // DrawTriangle3D( 
            //     Vector3{  0.0, 30.0, 0.0 }, 
            //     Vector3{ 30.0,  0.0, 0.0 }, 
            //     Vector3{ 30.0, 30.0, 0.0 },
            //     BLUE
            // );

        // EndShaderMode();
        EndMode3D();
        DrawFPS( 30, 60 );

        DrawText( "Simplest Shader Test", 30, 30, 20, LIGHTGRAY );

        EndDrawing();
    }

    CloseWindow();

    return 0;
}