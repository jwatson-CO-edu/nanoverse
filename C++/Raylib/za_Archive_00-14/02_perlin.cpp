// g++ 02_perlin.cpp -std=c++17 -lraylib

////////// INIT ////////////////////////////////////////////////////////////////////////////////////

///// Imports ////////////////////////////////////

/// Standard Imports ///
#include <stdlib.h>     /* srand, rand */
#include <time.h>       /* time */
#include <iostream>
using std::cout, std::endl, std::ostream;
#include <vector>
using std::vector;

/// Raylib ///
#include "raylib.h"
#include "raymath.h" //for vector3 math

///// Type Defines ///////////////////////////////
typedef vector<float> /*---*/ vf;
typedef vector<vector<float>> vvf;
typedef unsigned char /*---*/ ubyte;


////////// HELPER FUNCTIONS ////////////////////////////////////////////////////////////////////////

///// Image & Color Functions ////////////////////

Color get_image_pixel_color_at( Image& img, ulong row, ulong col ){
    // Get the Color object representing the color values at `row` and `col`
    // https://www.reddit.com/r/raylib/comments/pol80c/comment/hcxhhhf
    // WARNING: This function does not check image bounds!
    Color* colors = LoadImageColors( img );
    return colors[ row * img.width + col ];
}

float get_image_pixel_red_intensity( Image& img, ulong row, ulong col ){
    // Get the red intensity at the given `row` and `col`
    return 1.0f * get_image_pixel_color_at( img, row, col ).r / 255.0f;
}

vvf get_subimage_red_intensity( Image& img, ulong rowUL, ulong colUL, ulong rowLR, ulong colLR ){
    vvf rtnImg;
    vf  rowRtn;
    for( ulong i = rowUL; i < rowLR; i++ ){
        rowRtn.clear();
        for( ulong j = colUL; j < colLR; j++ ){
            rowRtn.push_back( get_image_pixel_red_intensity( img, i, j ) );
        }   
        rtnImg.push_back( rowRtn );
    }
    return rtnImg;
}

ostream& operator<<( ostream& os , const Color& clr ) { 
    // ostream '<<' operator for vectors
    // NOTE: This function assumes that the ostream '<<' operator for T has already been defined
    os << "{R: "  << ((int) clr.r);
    os << ", G: " << ((int) clr.g);
    os << ", B: " << ((int) clr.b);
    os << ", A: " << ((int) clr.a);
    os << "}";
    return os; // You must return a reference to the stream!
}


////////// MAIN ////////////////////////////////////////////////////////////////////////////////////

int main(void){

    srand( time( NULL ) );

    // Initialization
    //--------------------------------------------------------------------------------------
    const int screenWidth = 800;
    const int screenHeight = 450;

    InitWindow( screenWidth, screenHeight, "raylib example." );

    // Create a Perlin Image
    Image perlinImage = GenImagePerlinNoise( screenWidth, screenHeight, 0, 0, 1.6f ); 

    cout << get_image_pixel_color_at( perlinImage, 300, 300 ) << endl;
    
    // Images are inside the CPU and not fast. Use Textures(GPU) for quick manipulation.
    
    
    Texture2D texture = { 0 };

    texture = LoadTextureFromImage(perlinImage);

    // Unload image data (CPU RAM)
    UnloadImage(perlinImage);

    SetTargetFPS(60);               // Set our game to run at 60 frames-per-second
    //--------------------------------------------------------------------------------------

    // Main game loop
    while (!WindowShouldClose())    // Detect window close button or ESC key
    {
        // Update
        //----------------------------------------------------------------------------------
        
        //----------------------------------------------------------------------------------
        // Draw
        //----------------------------------------------------------------------------------
        BeginDrawing();

            ClearBackground(RAYWHITE);
                
            DrawTexture(texture, 0, 0, WHITE);           

        EndDrawing();
        //----------------------------------------------------------------------------------
    }

    // De-Initialization
    //--------------------------------------------------------------------------------------
    UnloadTexture( texture );       // Unload texture from VRAM
    CloseWindow();        // Close window and OpenGL context
    //--------------------------------------------------------------------------------------

    return 0;


}