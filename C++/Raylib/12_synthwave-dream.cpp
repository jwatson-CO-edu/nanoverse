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


class Plume : public TriModel{ public:
	// Queue of triangles representing a plume that decays from back to front, leading to trailing

    /// Members ///

    list<array<Vector3,2>> coords; // - 3D coordinates that define the plume, back of list is the head
	ushort /*-----------*/ N_seg; // -- Number of total segments the plume can contain
	ushort /*-----------*/ Np1_crd; //- Number of pairs of coordinates the plume can contain
	Color /*------------*/ color; // -- Color of the plume
	float /*------------*/ bgnAlpha; // Alpha value at the head of the plume
	float /*------------*/ endAlpha; // Alpha value at the tail of the plume

    /// Constructors & Destructors ///

    void build_triangles_with_color(){
        // Turn the train of coord pairs into a mesh
        list<array<Vector3,2>>::iterator it /*-*/ = coords.begin();
        array<Vector3,2> /*-----------*/ lastPair = *it;
        array<Vector3,2> /*-----------*/ currPair;
        Vector3 /*--------------------*/ c1, c2, l3, l4;
        Color /*----------------------*/ currClr;
        Color /*----------------------*/ lastClr;
        ulong /*----------------------*/ Npair = coords.size();
        it++;

        // Erase old triangles
        tris.clear();
        tClr.clear();
        
        for( ulong i = 1; i < Npair; i++ ){
            currPair = *it;
            c1 = currPair[0];
            c2 = currPair[1];
            l3 = lastPair[0];
            l4 = lastPair[1];
            currClr = color;
            lastClr = color;
            currClr.a = 255; // (ubyte) ((1.0f*i)/(Npair-1)*255.0f);
            lastClr.a = 255; // (ubyte) ((1.0f*(i-1))/(Npair-1)*255.0f);
            if( i%2 == 0 ){
                load_tri( c1, c2, l3 );
                load_tri_colors( currClr, currClr, lastClr );
                load_tri( l3, c2, l4 );
                load_tri_colors( lastClr, currClr, lastClr );
            }else{
                load_tri( c1, c2, l4 );
                load_tri_colors( currClr, currClr, lastClr );
                load_tri( l3, c1, l4 );
                load_tri_colors( lastClr, currClr, lastClr );
            }
            lastPair = currPair;
            it++;
        }
        mesh.triangleCount = tris.size();
        mesh.vertexCount   = tris.size()*3;
    }

    short push_segment( const Vector3& pnt1, const Vector3& pnt2 ){
        // Push leading to back, Pop trailing from front, Reload model
		if( coords.size() >= Np1_crd )  coords.pop_front();
        coords.push_back(  {pnt1, pnt2}  );
        if( coords.size() >= 2 )
            visible = true;
        else
            visible = false;
		return coords.size();
    }

    Plume( ushort maxSegments, Color plumeColor, float leadingAlpha = 1.0, float trailingAlpha = 0.0 ) : TriModel( maxSegments*2 ){
        // Allocate for max segments, Init as none
        N_seg    = maxSegments; // - Number of total segments the plume can contain
        Np1_crd  = maxSegments+1; // Number of pairs of coordinates the plume can contain
        color    = plumeColor; // -- Color of the plume
        bgnAlpha = leadingAlpha; //- Alpha value at the head of the plume
        endAlpha = trailingAlpha; // Alpha value at the tail of the plume
        visible  = false; // ------- No segments, No visibility
        // No segments, No drawing
        mesh.triangleCount = 0;
        mesh.vertexCount   = 0;
    }

    ~Plume(){
        // Free all allocated memory
        if( mesh.vertices )  delete mesh.vertices;
        if( mesh.indices  )  delete mesh.indices;
        if( mesh.normals  )  delete mesh.normals;
        if( mesh.colors   )  delete mesh.colors;
        UnloadModel( model );  
    }

    /// Methods ///


    ///// Rendering //////////////////////////////
    // WARNING: Requires window init to call!

    void load_geo(){
        // Get the model ready for drawing
        cout << "About to build triangles ..." << endl;
        build_triangles_with_color();
        cout << "About to build mesh ..." << endl;
        build_mesh_unshared();
        cout << "About to build colors ..." << endl;
        build_colors_unshared();
        cout << "About to build normals ..." << endl;
        build_normals_flat_unshared();
        cout << "About to upload mesh ..." << endl;
        UploadMesh( &mesh, true );
        cout << "About to upload model ..." << endl;
        model = LoadModelFromMesh( mesh );
        init_pose();
        model.transform = T;
        cout << "`load_geo` done!" << endl;
    }

    void reload_geo(){
        // Reconfigurate geometry
        cout << "About to build triangles ..." << endl;
        build_triangles_with_color();
        cout << "About to build mesh ..." << endl;
        build_mesh_unshared();
        cout << "About to build colors ..." << endl;
        build_colors_unshared( false );
        cout << "About to build normals ..." << endl;
        build_normals_flat_unshared( false );
        cout << "`reload_geo` done!" << endl;
        
        // UnloadModel( model );
        // UploadMesh( &mesh, true );
        // model = LoadModelFromMesh( mesh );
        // model.transform = T;
    }

    void draw(){
        // if( (tris.size() >= 2) && visible ){
        // if( visible ){
            DrawModel( model, Vector3{ 0.0f, 0.0f, 0.0f }, 1.0, color ); 
        // }
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

    Plume leftVortex{ 20, WHITE };

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
    leftVortex.load_geo();
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

        gliderPoints = glider.get_skeleton_points_in_world_frame();
        cout << gliderPoints[0] << gliderPoints[1] << leftVortex.tris.size() << endl;
        leftVortex.push_segment( gliderPoints[0], gliderPoints[1] );
        // leftVortex.reload_geo();
        // leftVortex.load_geo();

        ///// DRAW PHASE /////////////////////////
        
        BeginTextureMode( target ); // - Enable drawing to texture
            ClearBackground( BLACK ); // Clear texture background
            BeginMode3D( camera ); // -- Begin 3d mode drawing
                terrainTiles.draw();
                glider.draw();
                leftVortex.draw();
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