// https://www.raylib.com/cheatsheet/cheatsheet.html
// gcc 01_cube.cpp -lraylib

#include "raylib.h"
#include "rlgl.h" // `rlDisableBackfaceCulling`

// #include <raylib.h>

int main( void ){

    InitWindow(800, 450, "raylib [core] example - basic window");
    SetTargetFPS( 60 );
    rlDisableBackfaceCulling(); 
    rlEnableSmoothLines();

    // Camera
    Camera camera = Camera{
        Vector3{ 10.0, 10.0, 10.0 }, // Position
        Vector3{  0.0, 0.0, 0.0 }, // Target
        Vector3{  0.0, 1.0, 0.0 }, // Up
        45.0, // -------------------- FOV_y
        0 // ------------------------ Projection mode
    };


    while( !WindowShouldClose() ){
        
        BeginDrawing();
        BeginMode3D( camera );
        ClearBackground( BLACK );

            DrawCube(
                Vector3{  0.0, 0.0, 0.0 }, 
                1.0, // float width, 
                1.0, // float height,
                1.0, // float length, 
                LIGHTGRAY
            );
        
        EndMode3D();

        DrawText( "Congrats! You created your first window!", 190, 100, 20, LIGHTGRAY );

        EndDrawing();
    }

    CloseWindow();

    return 0;
}