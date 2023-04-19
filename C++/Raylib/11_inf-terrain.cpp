// g++ 11_inf-terrain.cpp -std=c++17 -lraylib

////////// INIT ////////////////////////////////////////////////////////////////////////////////////

///// Imports ////////////////////////////////////
/// Standard ///
#include <iostream>
using std::cout, std::endl, std::ostream;
#include <algorithm> 
using std::min, std::max;

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
    // Vector3 /*-----------*/ posn2; //- Line  drawing origin
    float /*-------------*/ pScale; // Perlin scale param

    /// Constructors & Helpers ///

    void gen_heightmap_uniform_random(){
        // Generate a heightmap with uniform random sampling
        vector<Vector3> row;
        for( ulong i = 0; i < M; i++ ){
            for( ulong j = 0; j < N; j++ ){
                row.push_back(  Vector3{ 
                    posn1.x + j*scl + randf( -scl*0.25, +scl*0.25 ), 
                    posn1.y + i*scl + randf( -scl*0.25, +scl*0.25 ),
                    posn1.z + randf()*scl 
                }  );
            }
            pts.push_back( row );
            row.clear();
        }
    }

    void gen_heightmap_perlin( float pScale = 1.6f ){
        // Create a Perlin Image
        // Calc corners
        float factor =   5.0f;
        int   hI     = 3*M; // Height of the image
        int   wI     = 3*N; // Width  of the image
        ulong rowUL  = randi( 0, hI - M );
        ulong colUL  = randi( 0, wI - N );
        ulong rowLR  = rowUL + M;
        ulong colLR  = colUL + N;
        // Select subimage
        Image  perlinImage = GenImagePerlinNoise( hI, wI, 0, 0, pScale ); 
        Color* perlinClrs  = LoadImageColors( perlinImage );
        vvf    zValues     = get_subimage_red_intensity( perlinClrs, perlinImage.width, rowUL, colUL, rowLR, colLR );
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
        if( perlinClrs )  free( perlinClrs );
        UnloadImage( perlinImage );  

        // zValues.clear();
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
        
        cout << "Basic `TerrainPlate` constructor!" << endl;

        // 0. Init
        M /**/ = Mrows;
        N /**/ = Ncols;
        scl    = scale;
        offset = scale/200.0f;
        gndClr = BLACK; // GREEN;
        linClr = MAGENTA; // WHITE; // BLACK;
        posn1  = Vector3{ 0.0f, 0.0f, 0.0f   };
        // posn2  = Vector3{ 0.0f, 0.0f, offset };
        pScale = 10.0f;

        // 1. Generate points
        gen_heightmap_perlin( pScale );

        // 2. Turn the heightmap into a mesh
        // build_triangles();
    }

    void stitch_X_POS_of( const TerrainPlate& OtherPlate ){
        float z1, z2;
        // 2. Stitch
        for( ulong i = 0; i < M; i++ ){  pts[i][0] = Vector3{ OtherPlate.pts[i][N-1] };  }
        // 3. Smooth
        for( ulong i = 0; i < M; i++ ){  
            z1 = pts[i][2].z;
            z2 = pts[i][0].z;
            pts[i][1].z = randf_bn( z1, z2 );
            // z1 = OtherPlate.pts[i][N-3].z;
            // z2 = OtherPlate.pts[i][N-1].z;
            // OtherPlate.pts[i][N-2].z = randf_bn( z1, z2 );
        }
    }

    void stitch_X_NEG_of( const TerrainPlate& OtherPlate ){
        float z1, z2;
        // 2. Stitch
        for( ulong i = 0; i < M; i++ ){  pts[i][N-1] = Vector3{ OtherPlate.pts[i][0] };  }
        // 3. Smooth
        for( ulong i = 0; i < M; i++ ){  
            z1 = pts[i][N-3].z;
            z2 = pts[i][N-1].z;
            pts[i][N-2].z = randf_bn( z1, z2 );
            // z1 = OtherPlate.pts[i][2].z;
            // z2 = OtherPlate.pts[i][0].z;
            // OtherPlate.pts[i][1].z = randf_bn( z1, z2 );
        }
    }

    void stitch_Y_POS_of( const TerrainPlate& OtherPlate ){
        float z1, z2;
        // 2. Stitch
        for( ulong j = 0; j < N; j++ ){  pts[0][j] = Vector3{ OtherPlate.pts[M-1][j] };  }
        // 3. Smooth
        for( ulong j = 0; j < N; j++ ){  
            z1 = pts[2][j].z;
            z2 = pts[0][j].z;
            pts[1][j].z = randf_bn( z1, z2 );
            // z1 = OtherPlate.pts[M-3][j].z;
            // z2 = OtherPlate.pts[M-1][j].z;
            // OtherPlate.pts[M-2][j].z = randf_bn( z1, z2 );
        }
    }

    void stitch_Y_NEG_of( const TerrainPlate& OtherPlate ){
        float z1, z2;
        // 2. Stitch
        for( ulong j = 0; j < N; j++ ){  pts[M-1][j] = Vector3{ OtherPlate.pts[0][j] };   }      
        // 3. Smooth
        for( ulong j = 0; j < N; j++ ){  
            z1 = pts[M-3][j].z;
            z2 = pts[M-1][j].z;
            pts[M-2][j].z = randf_bn( z1, z2 );
            // z1 = OtherPlate.pts[2][j].z;
            // z2 = OtherPlate.pts[0][j].z;
            // OtherPlate.pts[1][j].z = randf_bn( z1, z2 );
        }          
    }

    TerrainPlate( const TerrainPlate& OtherPlate, NEIGHBORS placement ) : TriModel( (OtherPlate.M-1)*(OtherPlate.N-1)*2 ){
        // Generate points and load triangles

        cout << "Offset `TerrainPlate` constructor!" << endl;

        // 0. Inherit params from neighbor
        M /**/ = OtherPlate.M;
        N /**/ = OtherPlate.N;
        scl    = OtherPlate.scl;
        offset = OtherPlate.offset;
        gndClr = OtherPlate.gndClr;
        linClr = OtherPlate.linClr;
        pScale = OtherPlate.pScale;

        // cout << gndClr << ", " << linClr << endl;
        
        // Placement Offset
        switch( placement ){

            case X_POS:
                posn1  = Vector3Add( OtherPlate.posn1, Vector3{  1.0f*(N-1)*scl,  0.0f , 0.0f   } );
                // 1. Generate points
                gen_heightmap_perlin( pScale );
                // stitch_X_POS_of( OtherPlate );
                break;

            case X_NEG:
                posn1  = Vector3Add( OtherPlate.posn1, Vector3{  -1.0f*(N-1)*scl,  0.0f , 0.0f   } );
                // 1. Generate points
                gen_heightmap_perlin( pScale );
                // stitch_X_NEG_of( OtherPlate );
                break;

            case Y_POS:
                posn1  = Vector3Add( OtherPlate.posn1, Vector3{  0.0f ,  1.0f*(M-1)*scl, 0.0f   } );
                // 1. Generate points
                gen_heightmap_perlin( pScale );
                // stitch_Y_POS_of( OtherPlate );
                break;

            case Y_NEG:
                posn1  = Vector3Add( OtherPlate.posn1, Vector3{  0.0f , -1.0f*(M-1)*scl, 0.0f   } );
                // 1. Generate points
                gen_heightmap_perlin( pScale );
                // stitch_Y_NEG_of( OtherPlate );
                break;
        }
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

    vector<NEIGHBORS> occupied_neighbors( const vector< TerrainPlate* >& tiles ){
        // Return a vector of occupied neighbors
    }

    void stitch_neighbors( const vector< TerrainPlate* >& tiles ){
        // Find an stitch to existing neighbors
        float radius = max( 1.0f*(M-1)*scl, 1.0f*(N-1)*scl ) * 1.25f;
        float dist   = 10000.0f;
        for( TerrainPlate* tile : tiles ){
            dist = Vector3Distance( tile->posn1, posn1 );
            if( (dist < radius) && (dist > 0.25f*radius) ){

                // X_POS of
                if((posn1.x - tile->posn1.x) > ( 0.5f * radius)){
                    stitch_X_POS_of( *tile );

                // X_NEG of
                }else if((posn1.x - tile->posn1.x) < (-0.5f * radius)){
                    stitch_X_NEG_of( *tile );
                
                // Y_POS of
                }else if((posn1.y - tile->posn1.y) > ( 0.5f * radius)){
                    stitch_Y_POS_of( *tile );
                
                // Y_NEG of
                }else if((posn1.y - tile->posn1.y) < (-0.5f * radius)){
                    stitch_Y_NEG_of( *tile );
                }
            }
        }
    }

    ///// Rendering //////////////////////////////
    // WARNING: Requires window init to call!

    void load_geo(){
        // Get the model ready for drawing
        cout << "\t\t`build_triangles` ..." << endl;
        build_triangles();
        cout << "\t\t`build_mesh_unshared` ..." << endl;
        build_mesh_unshared();
        cout << "\t\t`build_normals_flat_unshared` ..." << endl;
        build_normals_flat_unshared();
        cout << "\t\t`load_mesh` ..." << endl;
        load_mesh();
    }

    void draw(){
        // Draw facets, shift up, draw lines
        DrawModel(      model, Vector3{ 0.0f, 0.0f, 0.0f   }, 1.0, gndClr );  
        DrawModelWires( model, Vector3{ 0.0f, 0.0f, offset }, 1.0, linClr );
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
    vector< TerrainPlate* > terrainTiles;
    cout << "Create tile 1 ..." << endl;
    terrainTiles.push_back( new TerrainPlate{ tCellScale, Mrows, Nrows } );
    cout << "Create tile 2 ..." << endl;
    terrainTiles.push_back( new TerrainPlate{ *(terrainTiles.back()), X_POS } );  terrainTiles.back()->stitch_neighbors( terrainTiles );
    terrainTiles.push_back( new TerrainPlate{ *(terrainTiles.back()), Y_POS } );  terrainTiles.back()->stitch_neighbors( terrainTiles );
    terrainTiles.push_back( new TerrainPlate{ *(terrainTiles.back()), X_NEG } );  terrainTiles.back()->stitch_neighbors( terrainTiles );
    terrainTiles.push_back( new TerrainPlate{ *(terrainTiles.back()), X_NEG } );  terrainTiles.back()->stitch_neighbors( terrainTiles );
    terrainTiles.push_back( new TerrainPlate{ *(terrainTiles.back()), X_NEG } );  terrainTiles.back()->stitch_neighbors( terrainTiles );
    terrainTiles.push_back( new TerrainPlate{ *(terrainTiles.back()), Y_NEG } );  terrainTiles.back()->stitch_neighbors( terrainTiles );
    terrainTiles.push_back( new TerrainPlate{ *(terrainTiles.back()), X_POS } );  terrainTiles.back()->stitch_neighbors( terrainTiles );

    // TerrainPlate terrain{ tCellScale, Mrows, Nrows };
    cout << "Terrain built!" << endl;
    
    DeltaGlider  glider{ wingspan };
	float /*--*/ frameRotateRad = 3.1416/120.0;
    float /*--*/ frameThrust    = 12.0/60.0;

    /// Window Init ///
    InitWindow( (int) res.x, (int) res.y, "Terrain Gen + Glider + Bloom Shader" );
    SetTargetFPS( 60 );
    rlEnableSmoothLines();
    rlDisableBackfaceCulling();

    /// Scene Init: Post-Window ///
    // terrain.load_geo();
    cout << "About to load geo ..." << endl;
    glider.load_geo();
    cout << "\tGlider loaded!" << endl;
    for( TerrainPlate* plate : terrainTiles ){
        plate->load_geo();
    }
    cout << "\tTerrain loaded!" << endl;
    cout << "Geo loaded!" << endl;
    glider.set_XYZ( 
        Nrows*tCellScale/2.0f, 
        Nrows*tCellScale/2.0f, 
        // terrain.get_greatest_elevation()+10.0f 
        terrainTiles[0]->get_greatest_elevation()+10.0f
    );
    glider.rotate_RPY( 0.0, 3.1416/2.0, 3.1416 );

    FlightFollowThirdP_Camera camera{
        25.0,
		glider.get_XYZ(),
		glider.T
    };



    ////////// Shader Init: Pre-Window /////////////////////////////////////////////////////////////

    // load a shader and set up some uniforms
    // Shader fog = LoadShader( "shaders/fogLight.vs", "shaders/fogLight.fs" );
    // fog.locs[SHADER_LOC_MATRIX_MODEL] = GetShaderLocation( fog, "matModel" );
    // fog.locs[SHADER_LOC_VECTOR_VIEW]  = GetShaderLocation( fog, "viewPos"  );

    // int amb = GetShaderLocation( fog, "ambient" );
    // float ambClr[4] = {0.2, 0.2, 0.2, 1.0};
    // SetShaderValue( fog, amb, &ambClr, SHADER_UNIFORM_VEC4 );

    // int fogC = GetShaderLocation( fog, "fogColor" );
    // float fogClr[4] = {0.0, 0.0, 0.0, 1.0};
    // SetShaderValue( fog, fogC, &fogClr, SHADER_UNIFORM_VEC4 );

    // int   fogD /*-*/ = GetShaderLocation( fog, "FogDensity" );
    // float fogDensity = 0.015f;
    // SetShaderValue( fog, fogD, &fogDensity, SHADER_UNIFORM_FLOAT );

    // Fog applies to all terrain tiles
    // for( TerrainPlate* plate : terrainTiles ){  plate->model.materials[0].shader = fog;  }

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

        // update the light shader with the camera view position
        // SetShaderValue( fog, fog.locs[SHADER_LOC_VECTOR_VIEW], &camera.position.x, SHADER_UNIFORM_VEC3 );

        glider.z_thrust( frameThrust );


        ///// DRAW PHASE /////////////////////////
        
        BeginTextureMode( target );       // Enable drawing to texture
            ClearBackground( BLACK );  // Clear texture background
            BeginMode3D( camera );        // Begin 3d mode drawing
                for( TerrainPlate* plate : terrainTiles ){  plate->draw();  }
                // terrain.draw();  
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



    ////////// CLEANUP /////////////////////////////////////////////////////////////////////////////

    UnloadShader( bloom );
    // UnloadShader( fog   );
    UnloadRenderTexture( target );

    // UnloadModel( terrain.model );
    for( TerrainPlate* plate : terrainTiles ){  
        UnloadModel( plate->model );  
        delete plate;
    }
    
    UnloadModel( glider.model  );

    CloseWindow(); // Close window and OpenGL context

    return 0;
}