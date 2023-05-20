// g++ 01-1_hello-colors.cpp -std=gnu++17 -lraylib

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

    Shader hello = LoadShader( "shaders/helloColor.vs", "shaders/helloColor.fs" );
    // hello.locs[SHADER_LOC_VERTEX_POSITION] = GetShaderLocation(hello, "vertexPosition");
    // hello.locs[SHADER_LOC_MATRIX_MODEL]    = GetShaderLocation(hello, "matModel");
    
    // Vertex colors?
    float* colors = new float[12];
    colors[0] = 1.0f;  colors[1] = 0.0f;  colors[ 2] = 0.0f;  colors[ 3] = 1.0f; // Red
    colors[4] = 0.0f;  colors[5] = 1.0f;  colors[ 6] = 0.0f;  colors[ 7] = 1.0f; // Green 
    colors[8] = 0.0f;  colors[9] = 0.0f;  colors[10] = 1.0f;  colors[11] = 1.0f; // Blue

    // Yes: https://github.com/ChrisDill/raylib-instancing/blob/master/src/instancing/particles_instanced.c
    // ???: https://www.reddit.com/r/raylib/comments/vs6qcx/rldrawvertexarrayelements_not_working_as_expected/

    // Shader attribute locations
    int vtxColorAttrib = rlGetLocationAttrib( hello.id, "vertexColor" );
    rlEnableVertexAttribute( vtxColorAttrib );
    rlSetVertexAttribute(
        vtxColorAttrib, //- Attribute index
        3, // ------------- Size, number of elements 
        RL_FLOAT, // ------ Type
        true, // ---------- Normalized?
        sizeof(float)*4, // Stride
        (void*) colors // - Pointer
    );
    rlSetVertexAttributeDivisor( vtxColorAttrib, 1 );

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
        45.0, // ---------------------- FOV_y
        0 // -------------------------- Projection mode
    };

    


    while( !WindowShouldClose() ){
        BeginDrawing();
        BeginMode3D( camera );
        ClearBackground( BLACK );
        // BeginShaderMode( hello );

            setModelShader( &tri.model, &hello );
            tri.draw();

        // EndShaderMode();
        EndMode3D();

        DrawFPS( 30, 60 );
        DrawText( "Simplest Shader Test", 30, 30, 20, LIGHTGRAY );

        EndDrawing();
    }

    CloseWindow();
    delete colors;

    return 0;
}