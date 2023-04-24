// g++ 11_inf-terrain.cpp -std=c++17 -lraylib

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



////////// TERRAIN /////////////////////////////////////////////////////////////////////////////////

enum NEIGHBORS{
    X_POS,  X_NEG,
    Y_POS,  Y_NEG,
};
NEIGHBORS NEIGHBORHOOD[4] = {  X_POS,  X_NEG,  Y_POS,  Y_NEG  };

class TerrainTile : public TriModel { public:
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
    float /*-------------*/ pScale; // Perlin scale param
    bool /*--------------*/ loaded;

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

    TerrainTile( float scale, ulong Mrows, ulong Ncols, Vector3 origin ) : TriModel( (Mrows-1)*(Ncols-1)*2 ){
        // Generate points and load triangles
        
        cout << "Basic `TerrainTile` constructor!" << endl;

        // 0. Init
        M /*-*/ = Mrows;
        N /*-*/ = Ncols;
        scl     = scale;
        offset  = scale/200.0f;
        gndClr  = BLACK; 
        linClr  = MAGENTA;
        posn1   = origin; 
        pScale  = 10.0f;
        visible = true;
        loaded  = false;

        // 1. Generate points
        gen_heightmap_perlin( pScale );
    }

    ~TerrainTile(){
        // Free all allocated memory
        if( mesh.vertices )  delete mesh.vertices;
        if( mesh.indices  )  delete mesh.indices;
        if( mesh.normals  )  delete mesh.normals;
        UnloadModel( model );  
    }

    void stitch_X_POS_of( const TerrainTile& OtherPlate ){
        float z1, z2;
        // 2. Stitch
        for( ulong i = 0; i < M; i++ ){  pts[i][0] = Vector3{ OtherPlate.pts[i][N-1] };  }
        // 3. Smooth
        for( ulong i = 0; i < M; i++ ){  
            z1 = pts[i][2].z;
            z2 = pts[i][0].z;
            pts[i][1].z = randf_bn( z1, z2 );
        }
    }

    void stitch_X_NEG_of( const TerrainTile& OtherPlate ){
        float z1, z2;
        // 2. Stitch
        for( ulong i = 0; i < M; i++ ){  pts[i][N-1] = Vector3{ OtherPlate.pts[i][0] };  }
        // 3. Smooth
        for( ulong i = 0; i < M; i++ ){  
            z1 = pts[i][N-3].z;
            z2 = pts[i][N-1].z;
            pts[i][N-2].z = randf_bn( z1, z2 );
        }
    }

    void stitch_Y_POS_of( const TerrainTile& OtherPlate ){
        float z1, z2;
        // 2. Stitch
        for( ulong j = 0; j < N; j++ ){  pts[0][j] = Vector3{ OtherPlate.pts[M-1][j] };  }
        // 3. Smooth
        for( ulong j = 0; j < N; j++ ){  
            z1 = pts[2][j].z;
            z2 = pts[0][j].z;
            pts[1][j].z = randf_bn( z1, z2 );
        }
    }

    void stitch_Y_NEG_of( const TerrainTile& OtherPlate ){
        float z1, z2;
        // 2. Stitch
        for( ulong j = 0; j < N; j++ ){  pts[M-1][j] = Vector3{ OtherPlate.pts[0][j] };   }      
        // 3. Smooth
        for( ulong j = 0; j < N; j++ ){  
            z1 = pts[M-3][j].z;
            z2 = pts[M-1][j].z;
            pts[M-2][j].z = randf_bn( z1, z2 );
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

    vector<Vector3> get_corners() const{
        // Get the positions of the corners of the tile
        vector<Vector3> corners;
        cout << "About to check size ..." << endl;
        cout << "How many points?: " << pts.size() << endl;
        if( pts.size() ){
            cout << "There are corners to get ..." << endl;
            corners.push_back(  pts[0  ][0  ]  );
            corners.push_back(  pts[M-1][0  ]  );
            corners.push_back(  pts[M-1][N-1]  );
            corners.push_back(  pts[0  ][N-1]  );
        }
        return corners;
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
        loaded = true;
    }

    void draw(){
        // Draw facets, shift up, draw lines
        DrawModel(      model, Vector3{ 0.0f, 0.0f, 0.0f   }, 1.0, gndClr );  
        DrawModelWires( model, Vector3{ 0.0f, 0.0f, offset }, 1.0, linClr );
    }
};

class TerrainGrid{ public:
    // Managing structure for multiple `TerrainTile`s

    /// Members ///
    map<array<int,2>,TerrainTile*> tiles;
    float /*--------------------*/ sclCell;
    ulong /*--------------------*/ MrowsTile;
    ulong /*--------------------*/ NcolsTile;

    /// Constructors & Destructors ///

    TerrainGrid( float cellScale, ulong Mrows, ulong Nrows ){
        // Create a new grid with the center tile populated
        // Set params
        sclCell   = cellScale;
        MrowsTile = Mrows;
        NcolsTile = Nrows;
        // Populate center tile
        tiles[ {0,0} ] = new TerrainTile{ sclCell, MrowsTile, NcolsTile, Vector3{ 0.0f, 0.0f, 0.0f } };
    }

    ~TerrainGrid(){
        // Delete all tiles
        for( pair<array<int,2>,TerrainTile*> elem : tiles ){  delete elem.second;  }
        tiles.clear();
    }

    /// Methods ///

    bool p_cell_occupied( const array<int,2>& addr ){  return (tiles.count( addr ) > 0);  } // Is the cell at `addr` occupied?

    vector<array<int,2>> neighbors_of( const array<int,2>& addr ){
        // Return the Von Neumann neighborhood of `addr`
        vector<array<int,2>> rtnLst;
        rtnLst.push_back( { addr[0]  , addr[1]+1 } ); // X_POS
        rtnLst.push_back( { addr[0]  , addr[1]-1 } ); // X_NEG
        rtnLst.push_back( { addr[0]+1, addr[1]   } ); // Y_POS
        rtnLst.push_back( { addr[0]-1, addr[1]   } ); // Y_NEG
        return rtnLst;
    }

    Vector3 get_tile_origin( const array<int,2>& addr ){
        // Get the origin of the `addr` in 3D space
        return Vector3{
            1.0f*addr[0]*(1.0f*NcolsTile-1)*sclCell,
            1.0f*addr[1]*(1.0f*MrowsTile-1)*sclCell,
            0.0f 
        };
    }

    vector<Vector3> get_tile_corners( const array<int,2>& addr ){
        // Get the positions of the corners of the tile at the `addr`
        vector<Vector3> rtnLst;
        Vector3 /*---*/ origin = get_tile_origin( addr );
        rtnLst.push_back( origin );
        rtnLst.push_back( Vector3Add(  origin, Vector3{ (1.0f*NcolsTile-1)*sclCell,  0.0f                     , 0.0f }  ) );
        rtnLst.push_back( Vector3Add(  origin, Vector3{  0.0f                     , (1.0f*MrowsTile-1)*sclCell, 0.0f }  ) );
        rtnLst.push_back( Vector3Add(  origin, Vector3{ (1.0f*NcolsTile-1)*sclCell, (1.0f*MrowsTile-1)*sclCell, 0.0f }  ) );
        return rtnLst;
    }

    void populate_neighbors_of( const array<int,2>& addr ){
        // Create the Von Neumann neighborhood of `addr`
        TerrainTile* /*---*/ nuTile    = nullptr;
        vector<array<int,2>> neighbors = neighbors_of( addr ); // {X_POS, X_NEG, Y_POS, Y_NEG}
        vector<array<int,2>> nghbrNghbrs;
        int /*------------*/ i;
        
        for( array<int,2> nghbrAddr : neighbors ){
            if( !p_cell_occupied( nghbrAddr ) ){
                nuTile = new TerrainTile{ sclCell, MrowsTile, NcolsTile, get_tile_origin( nghbrAddr ) };
                tiles[ nghbrAddr ] = nuTile;
                nghbrNghbrs = neighbors_of( nghbrAddr ); // {X_POS, X_NEG, Y_POS, Y_NEG}
                i = 0;
                for( array<int,2> nghbrNghbr : nghbrNghbrs ){
                    if( p_cell_occupied( nghbrNghbr ) ){
                        switch (i){
                            case 0:
                                nuTile->stitch_Y_NEG_of( *tiles[ nghbrNghbr ] );
                                break;
                            case 1:
                                nuTile->stitch_Y_POS_of( *tiles[ nghbrNghbr ] );
                                break;
                            case 2:
                                nuTile->stitch_X_NEG_of( *tiles[ nghbrNghbr ] );
                                break;
                            case 3:
                                nuTile->stitch_X_POS_of( *tiles[ nghbrNghbr ] );
                                break;
                            default:
                                break;
                        }
                    }
                    i++;
                }
            }
        }
    }

    array<int,2> p_visible_and_oncoming( const FlightFollowThirdP_Camera& cam, const array<int,2>& addr ){
        // Return a two-part flag [ <Tile is close to the camera>, <Tile is in view or is in danger of coming into view> ]
        array<int,2>    rtnFlags = {0,0};
        vector<Vector3> corners = get_tile_corners( addr );
        float /*-----*/ dist, 
        /*-----------*/ xMin =  10000.0f, 
        /*-----------*/ xMax = -10000.0f, 
        /*-----------*/ yMin =  10000.0f, 
        /*-----------*/ yMax = -10000.0f;
        for( Vector3 corner : corners ){
            // Determine Visible
            if( Vector3Distance( cam.position, corner ) <= (1.5f*cam.dDrawMax) )  rtnFlags[0] = 1;
            // Determine oncoming
            dist = cam.signed_distance_to_frustrum( corner );
            if( (!isnan( dist )) && (dist <= (1.5f*cam.dDrawMax)) )  rtnFlags[1] = 1;
            if( corner.x < xMin )  xMin = corner.x;
            if( corner.x > xMax )  xMax = corner.x;
            if( corner.y < yMin )  yMin = corner.y;
            if( corner.y > yMax )  yMax = corner.y;
        }
        if(  (cam.position.x >= xMin) && (cam.position.x <= xMax) && (cam.position.y >= yMin) && (cam.position.y <= yMax)  ){
            rtnFlags[0] = 1;
            rtnFlags[1] = 1;
        }
        return rtnFlags;
    }

    vector<array<int,2>> mark_visible_and_return_oncoming( const FlightFollowThirdP_Camera& cam ){
        // Mark tiles in/visible, Return a list of oncoming tiles that might need expansion
        array<int,2> /*---*/ currAddr;
        TerrainTile* /*---*/ currTile = nullptr;
        array<int,2> /*---*/ tileRslt;
        vector<array<int,2>> rtnLst;

        // For every tile
        for( pair<array<int,2>,TerrainTile*> elem : tiles ){
            currAddr = elem.first;
            currTile = elem.second;
            tileRslt = p_visible_and_oncoming( cam, currAddr );
            if( tileRslt[0] ){  currTile->visible = true;  }else{  currTile->visible = false;  }
            if( tileRslt[1] ){
                rtnLst.push_back( currAddr );
            }
            if( (currTile->visible) && (!(currTile->loaded)) ){
                currTile->load_geo();
                // currTile->loaded = true;
            }
        }
        return rtnLst;
    }

    void mark_visible_and_expand_border( const FlightFollowThirdP_Camera& cam ){
        // Generate tiles as we approach them and maintain their visibility
        vector<array<int,2>> borderTiles = mark_visible_and_return_oncoming( cam );
        for( array<int,2> borderAddr : borderTiles ){
            populate_neighbors_of( borderAddr );
        }
    }

    void load_geo(){
        // Load geometry for all existing tiles
        for( pair<array<int,2>,TerrainTile*> elem : tiles ){  elem.second->load_geo();  }
    }

    void draw(){
        // Load geometry for all existing tiles
        for( pair<array<int,2>,TerrainTile*> elem : tiles ){  
            if( (elem.second->visible) && (elem.second->loaded) )  elem.second->draw();  
        }
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