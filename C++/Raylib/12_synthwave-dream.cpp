// g++ 12_synthwave-dream.cpp -std=c++17 -lraylib

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



////////// TOYS ////////////////////////////////////////////////////////////////////////////////////

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
    vvec3 /*--*/ gliderPoints;
    Vector3 /**/ vec1, vec2;

    Plume plume{ 30, ORANGE, 1.0f, 0.0f };
    vvec3 skltn;

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
    // leftVortex.load_geo();
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
        
        skltn = glider.get_skeleton_points_in_world_frame();
        vec1  = Vector3Subtract( skltn[1], skltn[0] );
        vec2  = Vector3Subtract( skltn[2], skltn[0] );
        vec1  = Vector3Add( Vector3Scale( vec1, 1.0f/Vector3Length(vec1)*wingspan*0.125 ), skltn[0] );
        vec2  = Vector3Add( Vector3Scale( vec2, 1.0f/Vector3Length(vec2)*wingspan*0.125 ), skltn[0] );
        plume.push_coord_pair( vec1, vec2 );

        camera.update_target_position( glider.get_XYZ() );
		camera.advance_camera();
        terrainTiles.mark_visible_and_expand_border( camera );

        ///// DRAW PHASE /////////////////////////
        
        BeginTextureMode( target ); // - Enable drawing to texture
            ClearBackground( BLACK ); // Clear texture background
            BeginMode3D( camera ); // -- Begin 3d mode drawing
                terrainTiles.draw();
                glider.draw();
                plume.draw();
                // leftVortex.draw();
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