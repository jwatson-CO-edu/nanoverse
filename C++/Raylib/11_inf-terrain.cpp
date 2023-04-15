// g++ 11_inf-terrain.cpp -std=c++17 -lraylib

////////// INIT ////////////////////////////////////////////////////////////////////////////////////

///// Imports ////////////////////////////////////
/// Standard ///
#include <iostream>
using std::cout, std::endl, std::ostream;
#include <algorithm> 
using std::min;

/// Raylib ///
#include <raylib.h>
#include <raymath.h>
#include <rlgl.h>
#define RLIGHTS_IMPLEMENTATION

/// Local ///
#include "utils.h"
#include "rl_toybox.h"
#include "rlights.h"


///// Type Aliases ///////////////////////////////
typedef unsigned short /*--*/ ushort;



////////// TERRAIN /////////////////////////////////////////////////////////////////////////////////

enum NEIGHBORS{
    X_POS,  X_NEG,
    Y_POS,  Y_NEG,
};

class TerrainPlate : public TriModel { public:
    // Rectangular plate of randomized terrain

    /// Member Data ///

    vector<vector<Vector3>> pts; // -- Grid points in 3D space
    float /*-------------*/ scl; // -- Scale of each cell
    ulong /*-------------*/ M; // ---- Number of rows
    ulong /*-------------*/ N; // ---- Number of cells per row
    Color /*-------------*/ gndClr; // Triangle fill color
    Color /*-------------*/ linClr; // Triangle line color
    float /*-------------*/ offset; // Z bump for lines
    Vector3 /*-----------*/ posn1; //- Facet drawing origin
    Vector3 /*-----------*/ posn2; //- Line  drawing origin
    float /*-------------*/ pScale; // Perlin scale param

    /// Constructors & Helpers ///

    void gen_heightmap_uniform_random(){
        // Generate a heightmap with uniform random sampling
        vector<Vector3> row;
        for( ulong i = 0; i < M; i++ ){
            row.clear();
            for( ulong j = 0; j < N; j++ ){
                row.push_back(  Vector3{ 
                    posn1.x + j*scl + randf( -scl*0.25, +scl*0.25 ), 
                    posn1.y + i*scl + randf( -scl*0.25, +scl*0.25 ),
                    posn1.z + randf()*scl 
                }  );
            }
            pts.push_back( row );
        }
    }

    void gen_heightmap_perlin( float pScale = 1.6f ){
        // Create a Perlin Image
        // Calc corners
        float factor =   15.0f;
        int   hI     = 1000; // Height of the image
        int   wI     = 1000; // Width  of the image
        ulong rowUL  = randi( 0, hI - M );
        ulong colUL  = randi( 0, wI - N );
        ulong rowLR  = rowUL + M;
        ulong colLR  = colUL + N;
        // Select subimage
        Image perlinImage = GenImagePerlinNoise( hI, wI, 0, 0, pScale ); 
        vvf   zValues     = get_subimage_red_intensity( perlinImage, rowUL, colUL, rowLR, colLR );
        // Generate heightmap
        vector<Vector3> row;
        float accum = 0;
        float elem;
        for( ulong i = 0; i < M; i++ ){
            row.clear();
            for( ulong j = 0; j < N; j++ ){
                elem = zValues[i][j]*scl*factor;
                accum += elem;
                row.push_back(  Vector3{ 
                    posn1.x + j*scl + randf( -scl*0.25, +scl*0.25 ), 
                    posn1.y + i*scl + randf( -scl*0.25, +scl*0.25 ),
                    posn1.z + elem
                }  );
            }
            pts.push_back( row );
        }
        accum /= M*N;
        for( ulong i = 0; i < M; i++ ){
            for( ulong j = 0; j < N; j++ ){
                pts[i][j].z -= accum;
            }
        }
    }

    void build_triangles(){
        // Turn the heightmap into a mesh
        for( ulong i = 0; i < M-1; i++ ){
            for( ulong j = 0; j < N-1; j++ ){
                Vector3 v1, v2, v3, v4;
                // Load points
                v1 = pts[i  ][j  ];
                v2 = pts[i  ][j+1];
                v3 = pts[i+1][j  ];
                v4 = pts[i+1][j+1];
                // Randomize cross right or cross left
                if( randf() < 0.5f ){
                    load_tri( v1, v2, v3 );
                    load_tri( v3, v2, v4 );
                }else{
                    load_tri( v1, v2, v4 );
                    load_tri( v3, v1, v4 );
                }
            }
        }
    }

    TerrainPlate( float scale = 10.0f, ulong Mrows = 10, ulong Ncols = 10 ) : TriModel( (Mrows-1)*(Ncols-1)*2 ){
        // Generate points and load triangles
        
        // 0. Init
        M /**/ = Mrows;
        N /**/ = Ncols;
        scl    = scale;
        offset = scale/200.0f;
        gndClr = BLACK; // GREEN;
        linClr = MAGENTA; // WHITE; // BLACK;
        posn1  = Vector3{ 0.0f, 0.0f, 0.0f   };
        posn2  = Vector3{ 0.0f, 0.0f, offset };
        pScale = 10.0f;

        // 1. Generate points
        gen_heightmap_perlin( pScale );

        // 2. Turn the heightmap into a mesh
        build_triangles();
    }

    TerrainPlate( const TerrainPlate& OtherPlate, NEIGHBORS placement ){
        // 0. Inherit params from neighbor
        M /**/ = OtherPlate.M;
        N /**/ = OtherPlate.N;
        scl    = OtherPlate.scl;
        offset = OtherPlate.offset;
        gndClr = OtherPlate.gndClr;
        linClr = OtherPlate.linClr;
        pScale = OtherPlate.pScale;
        
        // Placement Offset
        switch( placement ){

            case X_POS:
                posn1  = Vector3Add( OtherPlate.posn1, Vector3{  N*scl,  0.0f , 0.0f   } );
                posn2  = Vector3Add( posn1           , Vector3{  0.0f ,  0.0f , offset } );
                // 1. Generate points
                gen_heightmap_perlin( pScale );
                // 2. Stitch
                for( ulong i = 0; i < M; i++ ){  pts[i][0] = OtherPlate.pts[i][N-1];  }
                break;

            case X_NEG:
                posn1  = Vector3Add( OtherPlate.posn1, Vector3{ -N*scl,  0.0f , 0.0f   } );
                posn2  = Vector3Add( posn1           , Vector3{  0.0f ,  0.0f , offset } );
                // 1. Generate points
                gen_heightmap_perlin( pScale );
                // 2. Stitch
                for( ulong i = 0; i < M; i++ ){  pts[i][N-1] = OtherPlate.pts[i][0];  }
                break;

            case Y_POS:
                posn1  = Vector3Add( OtherPlate.posn1, Vector3{  0.0f ,  M*scl, 0.0f   } );
                posn2  = Vector3Add( posn1           , Vector3{  0.0f ,  0.0f , offset } );
                // 1. Generate points
                gen_heightmap_perlin( pScale );
                // 2. Stitch
                for( ulong j = 0; j < N; j++ ){  pts[0][j] = OtherPlate.pts[M-1][j];  }
                break;

            case Y_NEG:
                posn1  = Vector3Add( OtherPlate.posn1, Vector3{  0.0f , -M*scl, 0.0f   } );
                posn2  = Vector3Add( posn1           , Vector3{  0.0f ,  0.0f , offset } );
                // 1. Generate points
                gen_heightmap_perlin( pScale );
                // 2. Stitch
                for( ulong j = 0; j < N; j++ ){  pts[M-1][j] = OtherPlate.pts[0][j];  }
                break;
        }

        // 2. Turn the heightmap into a mesh
        build_triangles();
    }

    /// Methods ///

    float get_greatest_elevation(){
        // Get the highest point in the terrain
        float top = -1000.0f;
        for( ulong i = 0; i < M; i++ ){
            for( ulong j = 0; j < N; j++ ){
                if( pts[i][j].z > top )  top = pts[i][j].z;
            }
        }
        return top;
    }

    ///// Rendering //////////////////////////////
    // WARNING: Requires window init to call!

    void load_geo(){
        // Get the model ready for drawing
        build_mesh_unshared();
        build_normals_flat_unshared();
        load_mesh();
    }

    void draw(){
        // Draw facets, shift up, draw lines
        DrawModel(      model, posn1, 1.0, gndClr );  
        DrawModelWires( model, posn2, 1.0, linClr );
    }
};



////////// MAIN ////////////////////////////////////////////////////////////////////////////////////

int main(){
    const Vector2 res = { 1200, 600 };

    rand_seed();

    /// Scene Init: Pre-Window ///
    vector<TerrainPlate*> terrainTiles;
    terrainTiles.push_back( new TerrainPlate{ 10.0f, 25, 25 } );
    terrainTiles.push_back( new TerrainPlate{ *terrainTiles[0], X_POS } );
    terrainTiles.push_back( new TerrainPlate{ *terrainTiles[0], X_NEG } );
    // terrainTiles.push_back( new TerrainPlate{ *terrainTiles[0], Y_POS } );
    // terrainTiles.push_back( new TerrainPlate{ *terrainTiles[0], Y_NEG } );
    DeltaGlider  glider{ 10.0f };
	float /*--*/ frameRotateRad = 3.1416/120.0;
    float /*--*/ frameThrust    = 12.0/60.0;

    /// Window Init ///
    InitWindow( (int) res.x, (int) res.y, "Terrain Gen + Glider + Bloom Shader" );
    SetTargetFPS( 60 );
    rlEnableSmoothLines();
    rlDisableBackfaceCulling();

    /// Scene Init: Post-Window ///
    for( TerrainPlate* tile : terrainTiles ){  tile->load_geo();  } 
    glider.load_geo();
    glider.set_XYZ( 
        25*10/2.0f, 
        25*10/2.0f, 
        terrainTiles[0]->get_greatest_elevation()+10.0f 
    );
    glider.rotate_RPY( 0.0, 3.1416/2.0, 0.0 );

    FlightFollowThirdP_Camera camera{
        25.0,
		glider.get_XYZ(),
		glider.T
    };

    /// Shader Init: Pre-Window ///

    // bloom shader
    Shader bloom = LoadShader(0, "shaders/bloom.fs");
    RenderTexture2D target = LoadRenderTexture( res.x, res.y );

    while( !WindowShouldClose() ){

        /// UPDATE PHASE /////////////////////////

        // Keyboard input
        if( IsKeyDown( KEY_Z     ) ){  glider.rotate_RPY( -frameRotateRad,  0.0           ,  0.0            );  }
		if( IsKeyDown( KEY_X     ) ){  glider.rotate_RPY(  frameRotateRad,  0.0           ,  0.0            );  }
		if( IsKeyDown( KEY_UP    ) ){  glider.rotate_RPY(  0.0           ,  frameRotateRad,  0.0            );  }
		if( IsKeyDown( KEY_DOWN  ) ){  glider.rotate_RPY(  0.0           , -frameRotateRad,  0.0            );  }
        if( IsKeyDown( KEY_LEFT  ) ){  glider.rotate_RPY(  0.0           ,  0.0,             frameRotateRad );  }
		if( IsKeyDown( KEY_RIGHT ) ){  glider.rotate_RPY(  0.0           ,  0.0,            -frameRotateRad );  }

		// gamepad input
		if( IsGamepadAvailable(0) ){
			glider.rotate_RPY( 0.0, 0.0, -frameRotateRad*GetGamepadAxisMovement(0, GAMEPAD_AXIS_LEFT_X) );
			glider.rotate_RPY( 0.0, -frameRotateRad*GetGamepadAxisMovement(0, GAMEPAD_AXIS_LEFT_Y), 0.0 );
		}

        camera.update_target_position( glider.get_XYZ() );
		camera.advance_camera();
        glider.z_thrust( frameThrust );


        ///// DRAW PHASE /////////////////////////
        
        BeginTextureMode( target );       // Enable drawing to texture
            ClearBackground( BLACK );  // Clear texture background
            BeginMode3D( camera );        // Begin 3d mode drawing
                // terrain.draw();
                for( TerrainPlate* tile : terrainTiles ){  tile->draw();  } 
                glider.draw();
            EndMode3D();                // End 3d mode drawing, returns to orthographic 2d mode
        EndTextureMode();               // End drawing to texture (now we have a texture available for next passes)
        
        BeginDrawing();
            ClearBackground( BLACK );  // Clear screen background

            // Render generated texture using selected postprocessing shader
            BeginShaderMode( bloom );
                // NOTE: Render texture must be y-flipped due to default OpenGL coordinates (left-bottom)
                DrawTextureRec(
                    target.texture, 
                    (Rectangle){ 0, 0, (float)target.texture.width, (float)-target.texture.height }, 
                    (Vector2){ 0, 0 }, 
                    WHITE // BLACK // WHITE
                );
            EndShaderMode();
            DrawFPS( 10, 10 );

        EndDrawing();
    }

    UnloadShader( bloom );

    // UnloadModel( terrain.model );
    for( TerrainPlate* tile : terrainTiles ){  UnloadModel( tile->model );  } 

    UnloadModel( glider.model  );
    
    UnloadRenderTexture( target );

    CloseWindow(); // Close window and OpenGL context

    return 0;
}