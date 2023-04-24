// g++ 13_synthwave-dream.cpp -std=c++17 -lraylib

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
typedef unsigned short /*--*/ ushort;









////////// MAIN ////////////////////////////////////////////////////////////////////////////////////

int main(){

    rand_seed();

    const Vector2 res = { 1200, 600 };
    ulong Mrows = 50,
    /*-*/ Nrows = 50;
    float tCellScale = 10.0f,
    /*-*/ wingspan   = 10.0f;

    /// Scene Init: Pre-Window ///

    TerrainGrid terrainTiles{ tCellScale, Mrows, Nrows};
    terrainTiles.populate_neighbors_of( {0,0} );

    // TerrainTile terrain{ tCellScale, Mrows, Nrows };
    cout << "Terrain built!" << endl;
    
    DeltaGlider  glider{ wingspan };
	float /*--*/ frameRotateRad = 3.1416/120.0;
    float /*--*/ frameThrust    = 36.0/60.0;

    /// Window Init ///
    InitWindow( (int) res.x, (int) res.y, "Terrain Gen + Glider + Bloom Shader" );
    SetTargetFPS( 60 );
    rlEnableSmoothLines();
    rlDisableBackfaceCulling();

    /// Scene Init: Post-Window ///
    cout << "About to load geo ..." << endl;
    glider.load_geo();
    cout << "\tGlider loaded!" << endl;
    terrainTiles.load_geo();
    cout << "\tTerrain loaded!" << endl;
    cout << "Geo loaded!" << endl;
    glider.set_XYZ( 
        Nrows*tCellScale/2.0f, 
        Nrows*tCellScale/2.0f, 
        terrainTiles.tiles[{0,0}]->get_greatest_elevation()+10.0f
    );
    glider.rotate_RPY( 0.0, 3.1416/2.0, 3.1416 );

    FlightFollowThirdP_Camera camera{
        25.0,
		glider.get_XYZ(),
		glider.T
    };


    ////////// Shader Init: Pre-Window /////////////////////////////////////////////////////////////

    // bloom shader
    Shader bloom = LoadShader( 0, "shaders/bloom.fs" );
    bloom.locs[SHADER_LOC_MATRIX_MODEL] = GetShaderLocation(bloom, "matModel");

    RenderTexture2D target = LoadRenderTexture( res.x, res.y );



    ////////// RENDER LOOP /////////////////////////////////////////////////////////////////////////

    while( !WindowShouldClose() ){

        /// UPDATE PHASE /////////////////////////

        // Keyboard input
        if( IsKeyDown( KEY_Z     ) ){  glider.rotate_RPY( -frameRotateRad,  0.0           ,  0.0            );  }
		if( IsKeyDown( KEY_X     ) ){  glider.rotate_RPY(  frameRotateRad,  0.0           ,  0.0            );  }
		if( IsKeyDown( KEY_UP    ) ){  glider.rotate_RPY(  0.0           ,  frameRotateRad,  0.0            );  }
		if( IsKeyDown( KEY_DOWN  ) ){  glider.rotate_RPY(  0.0           , -frameRotateRad,  0.0            );  }
        if( IsKeyDown( KEY_LEFT  ) ){  glider.rotate_RPY(  0.0           ,  0.0           ,  frameRotateRad );  }
		if( IsKeyDown( KEY_RIGHT ) ){  glider.rotate_RPY(  0.0           ,  0.0           , -frameRotateRad );  }

		// gamepad input
		if( IsGamepadAvailable(0) ){
            glider.rotate_RPY( 
                 frameRotateRad*GetGamepadAxisMovement( 0, GAMEPAD_AXIS_RIGHT_X ),  
                 0.0,  
                 0.0            
            );
            glider.rotate_RPY( 
                 0.0, 
                -frameRotateRad*GetGamepadAxisMovement( 0, GAMEPAD_AXIS_RIGHT_Y ), 
                 0.0 
            );
			glider.rotate_RPY( 
                0.0, 
                0.0, 
                -frameRotateRad*GetGamepadAxisMovement( 0, GAMEPAD_AXIS_LEFT_X ) 
            );
		}

        glider.z_thrust( frameThrust );
        camera.update_target_position( glider.get_XYZ() );
		camera.advance_camera();
        terrainTiles.mark_visible_and_expand_border( camera );

        ///// DRAW PHASE /////////////////////////
        
        BeginTextureMode( target ); // - Enable drawing to texture
            ClearBackground( BLACK ); // Clear texture background
            BeginMode3D( camera ); // -- Begin 3d mode drawing
                terrainTiles.draw();
                glider.draw();
            EndMode3D(); //- End 3d mode drawing, returns to orthographic 2d mode
        EndTextureMode(); // End drawing to texture (now we have a texture available for next passes)
        
        BeginDrawing();
            ClearBackground( BLACK ); // Clear screen background

            // Render generated texture using selected postprocessing shader
            BeginShaderMode( bloom );
                // NOTE: Render texture must be y-flipped due to default OpenGL coordinates (left-bottom)
                DrawTextureRec(
                    target.texture, 
                    (Rectangle){ 0, 0, (float)target.texture.width, (float)-target.texture.height }, 
                    (Vector2){ 0, 0 }, 
                    WHITE 
                );
            EndShaderMode();
            DrawFPS( 10, 10 );
        EndDrawing();
    }



    ////////// CLEANUP /////////////////////////////////////////////////////////////////////////////

    UnloadShader( bloom );
    UnloadRenderTexture( target );    
    UnloadModel( glider.model  );
    CloseWindow(); // Close window and OpenGL context

    return 0;
}