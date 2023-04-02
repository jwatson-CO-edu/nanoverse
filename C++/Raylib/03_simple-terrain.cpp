// https://www.raylib.com/cheatsheet/cheatsheet.html
// gcc 03_simple-terrain.cpp -lraylib

////////// INIT ////////////////////////////////////////////////////////////////////////////////////

#include <stdlib.h>     /* srand, rand */
#include <time.h>       /* time */

#include "raylib.h"
#include "raymath.h"
#include "rlgl.h" // `rlDisableBackfaceCulling`

typedef unsigned long ulong;



////////// GLOBALS /////////////////////////////////////////////////////////////////////////////////

const ulong Mrows = 10;
const ulong Ncols = 10;

float scale = 10.0f;
float zOfst = scale / 50;
float verts[Mrows][Ncols][3];



////////// HELPER FUNCTIONS ////////////////////////////////////////////////////////////////////////

float randf(){
    // Return a pseudo-random number between 0.0 and 1.0
    return  1.0f * rand() / RAND_MAX;
}



////////// MAIN ////////////////////////////////////////////////////////////////////////////////////

int main(){
    srand( time( NULL ) );

    InitWindow(800, 450, "Simplest terrain");
    SetTargetFPS( 60 );
    rlDisableBackfaceCulling(); 
    rlEnableSmoothLines();

    // Camera
    Camera camera = Camera{
        Vector3{ 70.0, 70.0, 70.0 }, // Position
        Vector3{  50.0, 50.0, 0.0 }, // Target
        Vector3{  0.0, 0.0, 1.0 }, // Up
        45.0, // -------------------- FOV_y
        0 // ------------------------ Projection mode
    };

    /// Init terrain ///
    for( ulong i = 0; i < Mrows; i++ ){
        for( ulong j = 0; j < Ncols; j++ ){
            verts[i][j][0] = i*scale;
            verts[i][j][1] = j*scale;
            verts[i][j][2] = randf()*scale;
        }
    }
    Color terrainClr = GREEN;

    while( !WindowShouldClose() ){
        
        /// Begin Drawing ///
        BeginDrawing();
        BeginMode3D( camera );
        ClearBackground( BLACK );

        Vector3 v1, v2, v3;
        Vector3 offset = Vector3{ 0.0f, 0.0f, zOfst };

        ///// DRAW LOOP //////////////////////////

        for( ulong i = 0; i < Mrows-1; i++ ){
            for( ulong j = 0; j < Ncols-1; j++ ){
                v1 = Vector3{ verts[i+1][j  ][0], verts[i+1][j  ][1], verts[i+1][j  ][2] };
                v2 = Vector3{ verts[i  ][j  ][0], verts[i  ][j  ][1], verts[i  ][j  ][2] };
                v3 = Vector3{ verts[i  ][j+1][0], verts[i  ][j+1][1], verts[i  ][j+1][2] };
                DrawTriangle3D( v1, v2, v3, terrainClr );
                DrawLine3D( 
                    Vector3Add( v1, offset ), 
                    Vector3Add( v2, offset ), 
                    BLACK
                );
                DrawLine3D( 
                    Vector3Add( v2, offset ), 
                    Vector3Add( v3, offset ), 
                    BLACK
                );
                DrawLine3D( 
                    Vector3Add( v3, offset ), 
                    Vector3Add( v1, offset ), 
                    BLACK
                );
                v2 = Vector3{ verts[i  ][j+1][0], verts[i  ][j+1][1], verts[i  ][j+1][2] };
                v3 = Vector3{ verts[i+1][j+1][0], verts[i+1][j+1][1], verts[i+1][j+1][2] };
                DrawTriangle3D( v1, v2, v3, terrainClr );
                DrawLine3D( 
                    Vector3Add( v1, offset ), 
                    Vector3Add( v2, offset ), 
                    BLACK
                );
                DrawLine3D( 
                    Vector3Add( v2, offset ), 
                    Vector3Add( v3, offset ), 
                    BLACK
                );
                DrawLine3D( 
                    Vector3Add( v3, offset ), 
                    Vector3Add( v1, offset ), 
                    BLACK
                );
            }
        }


        /// End Drawing ///
        EndMode3D();
        EndDrawing();
    }

    return 0;
}