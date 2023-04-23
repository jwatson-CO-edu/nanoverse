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

    TerrainTile( float scale = 10.0f, ulong Mrows = 10, ulong Ncols = 10 ) : TriModel( (Mrows-1)*(Ncols-1)*2 ){
        // Generate points and load triangles
        
        cout << "Basic `TerrainTile` constructor!" << endl;

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

    void stitch_X_POS_of( const TerrainTile& OtherPlate ){
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

    void stitch_X_NEG_of( const TerrainTile& OtherPlate ){
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

    void stitch_Y_POS_of( const TerrainTile& OtherPlate ){
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

    void stitch_Y_NEG_of( const TerrainTile& OtherPlate ){
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

    Vector3 get_neighbor_center( const NEIGHBORS& direction ){
        // Get a prospective center of a tile in this direction from this tile
        switch( direction ){
            case X_POS:
                return Vector3Add( posn1, Vector3{   1.0f*(N-1)*scl,  0.0f          , 0.0f   } );
            case X_NEG:
                return Vector3Add( posn1, Vector3{  -1.0f*(N-1)*scl,  0.0f          , 0.0f   } );
            case Y_POS:
                return Vector3Add( posn1, Vector3{   0.0f          ,  1.0f*(M-1)*scl, 0.0f   } );
            case Y_NEG:
                return Vector3Add( posn1, Vector3{   0.0f          , -1.0f*(M-1)*scl, 0.0f   } );
            default:
                return Vector3{ nanf(""), nanf(""), nanf("") };
        }
    }

    TerrainTile( const TerrainTile& OtherPlate, NEIGHBORS placement ) : TriModel( (OtherPlate.M-1)*(OtherPlate.N-1)*2 ){
        // Generate points and load triangles

        cout << "Offset `TerrainTile` constructor!" << endl;

        // 0. Inherit params from neighbor
        M /**/ = OtherPlate.M;
        N /**/ = OtherPlate.N;
        scl    = OtherPlate.scl;
        offset = OtherPlate.offset;
        gndClr = OtherPlate.gndClr;
        linClr = OtherPlate.linClr;
        pScale = OtherPlate.pScale;

        // cout << gndClr << ", " << linClr << endl;
        // Vector3 temp;
        
        // Placement Offset
        switch( placement ){

            case X_POS:
                posn1 = Vector3Add( OtherPlate.posn1, Vector3{  1.0f*(N-1)*scl,  0.0f , 0.0f   } );
                // temp = OtherPlate.get_neighbor_center( X_POS );
                // 1. Generate points
                gen_heightmap_perlin( pScale );
                break;

            case X_NEG:
                posn1 = Vector3Add( OtherPlate.posn1, Vector3{  -1.0f*(N-1)*scl,  0.0f , 0.0f   } );
                // 1. Generate points
                gen_heightmap_perlin( pScale );
                break;

            case Y_POS:
                posn1 = Vector3Add( OtherPlate.posn1, Vector3{  0.0f ,  1.0f*(M-1)*scl, 0.0f   } );
                // 1. Generate points
                gen_heightmap_perlin( pScale );
                break;

            case Y_NEG:
                posn1 = Vector3Add( OtherPlate.posn1, Vector3{  0.0f , -1.0f*(M-1)*scl, 0.0f   } );
                // 1. Generate points
                gen_heightmap_perlin( pScale );
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

    vector<Vector3> get_corners() const{
        // Get the positions of the corners of the tile
        vector<Vector3> corners;
        corners.push_back(  pts[0  ][0  ]  );
        corners.push_back(  pts[M-1][0  ]  );
        corners.push_back(  pts[M-1][N-1]  );
        corners.push_back(  pts[0  ][N-1]  );
        return corners;
    }

    void stitch_neighbors( const vector< TerrainTile* >& tiles ){
        // Find an stitch to existing neighbors
        float radius = max( 1.0f*(M-1)*scl, 1.0f*(N-1)*scl ) * 1.25f;
        float dist   = 10000.0f;
        for( TerrainTile* tile : tiles ){
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

// set<NEIGHBORS> get_occupied_neighbors( const TerrainTile& queryTile, const vector<TerrainTile*>& tiles ){
//     // Return a list of directions from `tile` that are occupied by existing tiles
//     // NOTE: This program should maintain the invariant that each tile has only one neighbor in each direction,
//     //       but it won't break anything if the invariant is not maintained
//     float /*----*/ radius = max( 1.0f*(queryTile.M-1)*queryTile.scl, 1.0f*(queryTile.N-1)*queryTile.scl ) * 1.25f;
//     set<NEIGHBORS> rtnSet;
//     for( TerrainTile* tile : tiles ){
//         // Compare X_POS
//         if((queryTile.x - tile->posn1.x) > ( 0.5f * radius))  rtnSet.insert( X_POS );
//         // Compare X_NEG
//         if((queryTile.x - tile->posn1.x) < (-0.5f * radius))  rtnSet.insert( X_NEG );
//         // Compare Y_POS
//         if((queryTile.y - tile->posn1.y) > ( 0.5f * radius))  rtnSet.insert( Y_POS );
//         // Compare Y_NEG
//         if((queryTile.y - tile->posn1.y) < (-0.5f * radius))  rtnSet.insert( Y_NEG );
//     }
//     return rtnSet;
// }

// 

// bool error_vec3( const Vector3& vec ){
//     // Return true if any of the elements of the vector are NaN
//     return isnanf( vec.x ) || isnanf( vec.y ) || isnanf( vec.z );
// }

bool p_in_view( const FlightFollowThirdP_Camera& cam, const TerrainTile& tile ){
    // Return true if `tile` needs its neighbors to be instantiated in order to look infinite from `cam` view
    vector<Vector3> corners;
    bool /*------*/ oncoming = false;
    float dist, 
          xMin =  10000.0f, 
          xMax = -10000.0f, 
          yMin =  10000.0f, 
          yMax = -10000.0f;
    // 1. Get the neighbors of `tile` that *should* be expanded
    // Is the tile in the frustrum cone?
    corners  = tile.get_corners();
    for( Vector3 corner : corners ){
        if( corner.x < xMin )  xMin = corner.x;
        if( corner.x > xMax )  xMax = corner.x;
        if( corner.y < yMin )  yMin = corner.y;
        if( corner.y > yMax )  yMax = corner.y;
        if( cam.inside_FOV( corner ) ){
            oncoming = true;
            break;
        }
    }
    if(  (cam.position.x >= xMin) && (cam.position.x <= xMax) && (cam.position.y >= yMin) && (cam.position.y <= yMax)  ){
        return true;
    }
    if( oncoming ){
        for( Vector3 corner : corners ){
            dist = cam.signed_distance_to_frustrum( corner );
            if( (!isnanf( dist )) && (dist < 0.0f) ){
                return true;
            }
        }
    }
    return false;
}



void expand_visible_and_hide_trailing_tiles( FlightFollowThirdP_Camera& cam, vector<TerrainTile*>& tiles ){
    // Add tiles ahead of visible tiles, Mark oncoming tiles visible, Mark far trailing tiles invisible

    set<NEIGHBORS>  occupied;
    vector<Vector3> corners;
    bool /*------*/ queryVisible;
    bool /*------*/ addTile;
    float /*-----*/ dist;
    float /*-----*/ dMin;
    float /*-----*/ radius = max( 1.0f*(tiles[0]->M-1)*tiles[0]->scl, 1.0f*(tiles[0]->N-1)*tiles[0]->scl ) * 1.25f;
    
    for( TerrainTile* queryTile : tiles ){

        queryVisible = p_in_view( cam, *queryTile );
        addTile /**/ = false;
        
        if( queryVisible )  queryTile->visible = true;
        occupied.clear();

        for( TerrainTile* otherTile : tiles ){
            
            if( queryVisible && (queryTile != otherTile) ){
                // Compare X_POS
                if((queryTile->posn1.x - otherTile->posn1.x) > ( 0.5f * radius)){
                    occupied.insert( X_POS );
                    addTile = true;
                }  
                // Compare X_NEG
                if((queryTile->posn1.x - otherTile->posn1.x) < (-0.5f * radius)){
                    occupied.insert( X_NEG );
                    addTile = true;
                }  
                // Compare Y_POS
                if((queryTile->posn1.y - otherTile->posn1.y) > ( 0.5f * radius)){
                    occupied.insert( Y_POS );
                    addTile = true;
                }  
                // Compare Y_NEG
                if((queryTile->posn1.y - otherTile->posn1.y) < (-0.5f * radius)){
                    occupied.insert( Y_NEG );
                    addTile = true;
                }  
            }
        }

        if( addTile ){
            for( NEIGHBORS neighbor : NEIGHBORHOOD ){
                if( !occupied.count( neighbor ) ){
                    tiles.push_back( new TerrainTile{ *queryTile, neighbor } );
                    tiles.back()->stitch_neighbors( tiles );
                }
            }
        }

        queryVisible = false;
        corners /**/ = queryTile->get_corners();
        for( Vector3 corner : corners ){
            dist = cam.signed_distance_to_frustrum( corner );
            if( !isnanf( dist ) && (dist < 2.0f*cam.dDrawMax) ){
                queryTile->visible = true;
                queryVisible /*-*/ = true;
            }  
        }

        dMin = 10000.0f;
        if( !queryVisible ){
            for( Vector3 corner : corners ){
                dist = Vector3Distance( cam.position, corner );
                if( dist < dMin )  dMin = dist;
            }
            if( dMin > cam.dDrawMax )  
                queryTile->visible = false;
            else
                queryTile->visible = true;
        }
    }
}


////////// MAIN ////////////////////////////////////////////////////////////////////////////////////

int main(){

    rand_seed();

    const Vector2 res = { 1200, 600 };
    ulong Mrows = 50,
    /*-*/ Nrows = 50;
    float tCellScale = 10.0f,
    /*-*/ wingspan   = 10.0f;

    

    /// Scene Init: Pre-Window ///
    vector< TerrainTile* > terrainTiles;
    cout << "Create tile 1 ..." << endl;
    terrainTiles.push_back( new TerrainTile{ tCellScale, Mrows, Nrows } );
    cout << "Create tile 2 ..." << endl;
    terrainTiles.push_back( new TerrainTile{ *(terrainTiles.back()), X_POS } );  terrainTiles.back()->stitch_neighbors( terrainTiles );
    terrainTiles.push_back( new TerrainTile{ *(terrainTiles.back()), Y_POS } );  terrainTiles.back()->stitch_neighbors( terrainTiles );
    terrainTiles.push_back( new TerrainTile{ *(terrainTiles.back()), X_NEG } );  terrainTiles.back()->stitch_neighbors( terrainTiles );
    terrainTiles.push_back( new TerrainTile{ *(terrainTiles.back()), X_NEG } );  terrainTiles.back()->stitch_neighbors( terrainTiles );
    terrainTiles.push_back( new TerrainTile{ *(terrainTiles.back()), X_NEG } );  terrainTiles.back()->stitch_neighbors( terrainTiles );
    terrainTiles.push_back( new TerrainTile{ *(terrainTiles.back()), Y_NEG } );  terrainTiles.back()->stitch_neighbors( terrainTiles );
    terrainTiles.push_back( new TerrainTile{ *(terrainTiles.back()), X_POS } );  terrainTiles.back()->stitch_neighbors( terrainTiles );

    // TerrainTile terrain{ tCellScale, Mrows, Nrows };
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
    for( TerrainTile* plate : terrainTiles ){
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
    // for( TerrainTile* plate : terrainTiles ){  plate->model.materials[0].shader = fog;  }

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

        // glider.z_thrust( frameThrust );


        ///// DRAW PHASE /////////////////////////
        
        BeginTextureMode( target );       // Enable drawing to texture
            ClearBackground( BLACK );  // Clear texture background
            BeginMode3D( camera );        // Begin 3d mode drawing
                for( TerrainTile* plate : terrainTiles ){  plate->draw();  }
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
    for( TerrainTile* plate : terrainTiles ){  
        UnloadModel( plate->model );  
        delete plate;
    }
    
    UnloadModel( glider.model  );

    CloseWindow(); // Close window and OpenGL context

    return 0;
}