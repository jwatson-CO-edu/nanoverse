// g++ 17_shiny-icos.cpp -std=c++17 -lraylib
// Re-implement Boid Ribbons **without** `Model`s!
// 2023-08-14: Do not break into smaller files until everything works

// FIXME: PUT FRACTURED CUBE IN A VECTOR OF POINTERS

////////// INIT ////////////////////////////////////////////////////////////////////////////////////

///// Imports ////////////////////////////////////

/// Standard ///
#include <iostream>
using std::cout, std::endl;
#include <stdlib.h>  // srand, rand
#include <time.h>
#include <array>
using std::array;
#include <vector>
using std::vector;
#include <memory>
using std::shared_ptr;

/// Raylib ///
#include <raylib.h>
#include <raymath.h>
#include <rlgl.h>

/// Local ///
#define RLIGHTS_IMPLEMENTATION
#include "rlights.h"


///// Aliases ////////////////////////////////////
typedef array<Vector3,3> triPnts; // Vector info for One Triangle (Vertices,nrms) 
typedef array<Color,3>   triClrs; // Color  info for One Triangle
typedef vector<Vector3>  vvec3; // - Vector of 3D vectors
#define VERTEX_BUFFER_IDX 0 // Vertex coord VBO
#define NORMAL_BUFFER_IDX 2 // Normal vector VBO
#define COLORS_BUFFER_IDX 3 // Vertex color VBO
#define INDEXF_BUFFER_IDX 6 // Indices of facet vertices VBO



////////// RANDOM NUMBERS //////////////////////////////////////////////////////////////////////////

float randf(){
    // Return a pseudo-random number between 0.0 and 1.0
    return  1.0f * rand() / RAND_MAX;
}

float randf( float lo, float hi ){
    // Return a pseudo-random number between `lo` and `hi`
    // NOTE: This function assumes `hi > lo`
    float span = hi - lo;
    return lo + span * randf();
}

void rand_seed(){  srand( time(NULL) );  } // Seed RNG with unpredictable time-based seed



////////// VECTOR OPERATIONS ///////////////////////////////////////////////////////////////////////


Vector3 normal_of_tiangle( const triPnts& tri ){
    // Get the normal of the triangle assuming CCW / Right-Hand ordering
    return Vector3Normalize( Vector3CrossProduct(
        Vector3Subtract( tri[2], tri[1] ),
        Vector3Subtract( tri[0], tri[1] )
    ));
}

Vector3 uniform_vector_noise( const Vector3& vec, float halfMag ){
    // Return a perturbed `vec` moved up to `halfMag` in each direction in each dimension
    return Vector3{
        randf( vec.x - halfMag, vec.x + halfMag ),
        randf( vec.y - halfMag, vec.y + halfMag ),
        randf( vec.z - halfMag, vec.z + halfMag )
    };
}

Vector3 uniform_vector_noise( float halfMag ){
    // Return a perturbed `vec` moved up to `halfMag` in each direction in each dimension
    return Vector3{
        randf( -halfMag, halfMag ),
        randf( -halfMag, halfMag ),
        randf( -halfMag, halfMag )
    };
}



////////// TOYS ////////////////////////////////////////////////////////////////////////////////////


class DynaMesh{ public:
    // Straightforward container for a `Mesh` with changing geometry
    // 2023-08-14: This class assumes that the mesh will have a constant size for each buffer, though their values might change
    // 2023-08-15: This class REQUIRES that vertices, indices, normals, and colors ALL be defined

    /// Members ///
    vector<triPnts> tris; // Dynamic geometry, each array is a facet of 3x vertices
    vector<triPnts> nrms; // Normal vector for each facet
    vector<triClrs> clrs; // Vertex colors for each facet
    Mesh /*------*/ mesh; // Raylib mesh geometry
    bool /*------*/ upld; // Has the mesh been uploaded?
    Matrix /*----*/ xfrm; // Pose in the parent frame
    Model /*-----*/ modl; // USE MODEL I GUESS
    Shader /*----*/ shdr; // USE MODEL I GUESS
    
    /// Memory Methods ///

    void init_mesh_memory( uint Ntri ){
        // Create the `Mesh` struct and allocate memory there

        uint Npts = Ntri*3;

        // 1. Init mesh
        upld = false;
        mesh = Mesh{};
        mesh.triangleCount = Ntri;
        mesh.vertexCount   = Npts;

        // 2. Init material
        // matl = LoadMaterialDefault();
        // matl = Material{};
        
        cout << "GO!: init_mesh_memory( " << mesh.triangleCount << " )" << endl;

        // 3. Init memory
        mesh.vertices = (float* ) MemAlloc(Npts*3 * sizeof( float  )); // 3 vertices, 3 coordinates each (x, y, z)
        mesh.indices  = (ushort*) MemAlloc(Npts   * sizeof( ushort ));
        mesh.normals  = (float* ) MemAlloc(Npts*3 * sizeof( float  )); // 3 vertices, 3 coordinates each (x, y, z)
        mesh.colors   = (u_char*) MemAlloc(Npts*4 * sizeof( u_char )); // 3 vertices, 4 coordinates each (r, g, b, a)
    }

    void load_mesh_buffers( bool loadGeo = true, bool loadColor = false ){
        // Load geometry (and color) info into `mesh` buffers        
        // 2023-08-14: For now assume that the facet indices do NOT change!

        cout << "GO!: " << mesh.triangleCount << ", load_mesh_buffers( " << loadGeo <<", "<< loadColor << " )" << endl;

        // 0. Init
        ulong k = 0; // Vertex _ counter
        ulong l = 0; // Index __ counter
        ulong m = 0; // Color __ counter

        // 1. Load CPU-side
        for( uint i = 0; i < mesh.triangleCount; ++i ){
            for( u_char j = 0; j < 3; j++ ){
                if( loadGeo ){
                    mesh.normals[k]  = nrms[i][j].x;
                    mesh.vertices[k] = tris[i][j].x;  k++;
                    mesh.normals[k]  = nrms[i][j].y;
                    mesh.vertices[k] = tris[i][j].y;  k++;
                    mesh.normals[k]  = nrms[i][j].z;
                    mesh.vertices[k] = tris[i][j].z;  k++;
                    mesh.indices[l]  = l; /*-------*/ l++; 
                }
                if( loadColor ){
                    mesh.colors[m] = clrs[i][j].r;  m++;
                    mesh.colors[m] = clrs[i][j].g;  m++;
                    mesh.colors[m] = clrs[i][j].b;  m++;
                    mesh.colors[m] = clrs[i][j].a;  m++;
                }
            }
        }

        // If this mesh is not present on the GPU, then send it
        if( !upld ){
            // 3. Initial load to GPU
            UploadMesh( &mesh, true );
            upld = true;
        // Else mesh is on GPU, update the mesh buffers there
        }else{
            // 2. Send GPU-side
            if( loadGeo ){
                UpdateMeshBuffer( 
                    mesh, // -------------------------------- `Mesh` object
                    VERTEX_BUFFER_IDX, // ------------------- VBO Index
                    mesh.vertices, // ----------------------- Array of data 
                    mesh.vertexCount*3 * sizeof( float  ), // Total array size
                    0 // ------------------------------------ Starting index in array
                );
                UpdateMeshBuffer( 
                    mesh, // -------------------------------- `Mesh` object
                    NORMAL_BUFFER_IDX, // ------------------- VBO Index
                    mesh.normals, // ------------------------ Array of data 
                    mesh.vertexCount*3 * sizeof( float  ), // Total array size
                    0 // ------------------------------------ Starting index in array
                );
                // 2023-08-14: For now assume that the facet indices do NOT change!
            }
            if( loadColor ){
                UpdateMeshBuffer( 
                    mesh, // -------------------------------- `Mesh` object
                    COLORS_BUFFER_IDX, // ------------------- VBO Index
                    mesh.colors, // ------------------------- Array of data 
                    mesh.vertexCount*4 * sizeof( u_char ), // Total array size
                    0 // ------------------------------------ Starting index in array
                );
            }
        }
    }

    void remodel(){
        modl = LoadModelFromMesh( mesh );
        modl.materials[0] = LoadMaterialDefault();
        modl.materials[0].shader = shdr;
    }

    /// Constructors ///

    DynaMesh( uint Ntri ){
        // Allocate memory and set default pose
        init_mesh_memory( Ntri );
        xfrm = MatrixIdentity();
    }

    /// Geometry Methods ///

    void push_triangle_w_norms( triPnts tri ){
        // Add one triangle
        Vector3 norm = normal_of_tiangle( tri );  
        tris.push_back( tri );
        nrms.push_back({ norm, norm, norm });
    }
    
    void wipe_geo( bool wipeTris = true, bool wipeColors = false ){
        // Remove all built geometry
        if( wipeTris ){
            tris.clear();
            nrms.clear();
        }
        if( wipeColors ){
            clrs.clear();
        }
    }

    /// Pose Math ///
    /* xfrm = 
    m0 m4 m8  m12
    m1 m5 m9  m13
    m2 m6 m10 m14
    m3 m7 m11 m15
    */

    Vector3 get_posn(){
        // Get the position components of the homogeneous coordinates as a vector
        return Vector3{ xfrm.m12, xfrm.m13, xfrm.m14 };
    }

    void set_posn( const Vector3& posn ){
        // Set the position components of the homogeneous coordinates
        xfrm.m12 = posn.x;
        xfrm.m13 = posn.y;
        xfrm.m14 = posn.z;
    }

    void translate( const Vector3& delta ){
        // Increment the position components of the homogeneous coordinates by the associated `delta` components
        xfrm.m12 += delta.x;
        xfrm.m13 += delta.y;
        xfrm.m14 += delta.z;
    }

    void rotate_RPY( float r_, float p_, float y_ ){
		// Increment the world Roll, Pitch, Yaw of the model
        xfrm = MatrixMultiply( 
            MatrixMultiply( MatrixMultiply( MatrixRotateY( y_ ), MatrixRotateX( p_ ) ), MatrixRotateZ( r_ ) ), 
            xfrm 
        );
	}

    ///// Drawing Context Methods ////////////////
    // NOTE: Method beyond this point require a drawing context being instantiated before calling!

    /// Rendering ///

    void set_shader( Shader shader ){ shdr = shader; }

    void draw(){
        // Render the mesh
        modl.transform = xfrm;
        DrawModel( modl, get_posn(), 1.0f, WHITE );
    }

};

class FractureCube : public DynaMesh{ public:
    // A cube that shakes apart

    vvec3   V;

    /// Constructors ///

    FractureCube( float sideLen ) : DynaMesh( 12 ){
        // 0. Init
        triPnts pushTri;
        triClrs pushClr;
        Vector3 norm;
        
        float   halfLen = sideLen/2.0;
        Color   R{ 255,   0,   0, 255 };
        Color   G{   0, 255,   0, 255 };
        Color   B{   0,   0, 255, 255 };

        // 1. Establish vertices
        V.push_back( Vector3{ -halfLen, -halfLen, -halfLen } );
        V.push_back( Vector3{ -halfLen, -halfLen,  halfLen } );
        V.push_back( Vector3{ -halfLen,  halfLen, -halfLen } );
        V.push_back( Vector3{ -halfLen,  halfLen,  halfLen } );
        V.push_back( Vector3{  halfLen, -halfLen, -halfLen } );
        V.push_back( Vector3{  halfLen, -halfLen,  halfLen } );
        V.push_back( Vector3{  halfLen,  halfLen, -halfLen } );
        V.push_back( Vector3{  halfLen,  halfLen,  halfLen } );

        // 2. Build tris
        push_triangle_w_norms( { V[0], V[3], V[2] } );
        push_triangle_w_norms( { V[0], V[1], V[3] } );
        push_triangle_w_norms( { V[6], V[4], V[0] } );
        push_triangle_w_norms( { V[6], V[0], V[2] } );
        push_triangle_w_norms( { V[0], V[4], V[5] } );
        push_triangle_w_norms( { V[0], V[5], V[1] } );
        push_triangle_w_norms( { V[7], V[6], V[2] } );
        push_triangle_w_norms( { V[7], V[2], V[3] } );
        push_triangle_w_norms( { V[4], V[6], V[7] } );
        push_triangle_w_norms( { V[4], V[7], V[5] } ); 
        push_triangle_w_norms( { V[1], V[5], V[7] } );
        push_triangle_w_norms( { V[1], V[7], V[3] } );

        // 3. Build Colors
        pushClr = { R, G, B };  clrs.push_back( pushClr );
        pushClr = { B, R, G };  clrs.push_back( pushClr );
        pushClr = { G, B, R };  clrs.push_back( pushClr );
        pushClr = { R, G, B };  clrs.push_back( pushClr );
        pushClr = { B, R, G };  clrs.push_back( pushClr );
        pushClr = { G, B, R };  clrs.push_back( pushClr );
        pushClr = { R, G, B };  clrs.push_back( pushClr );
        pushClr = { B, R, G };  clrs.push_back( pushClr );
        pushClr = { G, B, R };  clrs.push_back( pushClr );
        pushClr = { R, G, B };  clrs.push_back( pushClr );
        pushClr = { B, R, G };  clrs.push_back( pushClr );
        pushClr = { G, B, R };  clrs.push_back( pushClr );

        // 4. Load buffers
        load_mesh_buffers( true, true );
    }

    void update(){
        // translate( uniform_vector_noise( 0.125 ) );

        wipe_geo( true, true );

        // 2. Build tris
        push_triangle_w_norms( { V[0], V[3], V[2] } );
        push_triangle_w_norms( { V[0], V[1], V[3] } );
        push_triangle_w_norms( { V[6], V[4], V[0] } );
        push_triangle_w_norms( { V[6], V[0], V[2] } );
        push_triangle_w_norms( { V[0], V[4], V[5] } );
        push_triangle_w_norms( { V[0], V[5], V[1] } );
        push_triangle_w_norms( { V[7], V[6], V[2] } );
        push_triangle_w_norms( { V[7], V[2], V[3] } );
        push_triangle_w_norms( { V[4], V[6], V[7] } );
        push_triangle_w_norms( { V[4], V[7], V[5] } ); 
        push_triangle_w_norms( { V[1], V[5], V[7] } );
        push_triangle_w_norms( { V[1], V[7], V[3] } );

        // 3. Build Colors
        clrs.push_back( { RED, GREEN, BLUE } );
        clrs.push_back( { RED, GREEN, BLUE } );
        clrs.push_back( { RED, GREEN, BLUE } );
        clrs.push_back( { RED, GREEN, BLUE } );
        clrs.push_back( { RED, GREEN, BLUE } );
        clrs.push_back( { RED, GREEN, BLUE } );
        clrs.push_back( { RED, GREEN, BLUE } );
        clrs.push_back( { RED, GREEN, BLUE } );
        clrs.push_back( { RED, GREEN, BLUE } );
        clrs.push_back( { RED, GREEN, BLUE } );
        clrs.push_back( { RED, GREEN, BLUE } );
        clrs.push_back( { RED, GREEN, BLUE } );

        if( randf() < 0.50 ){
            for( triPnts& tri : tris ){
                tri[0] = uniform_vector_noise( tri[0], 0.125 );
                tri[1] = uniform_vector_noise( tri[1], 0.125 );
                tri[2] = uniform_vector_noise( tri[2], 0.125 );
                // 2023-08-13: Purposely avoiding a normal update until a shader is in use
            }
        }
        load_mesh_buffers( true, false );
        // remodel();
    }
};



////////// MAIN ////////////////////////////////////////////////////////////////////////////////////

int main(){
    rand_seed();

    /// Window Init ///
    InitWindow( 900, 900, "Dynamic Box!" );
    SetTargetFPS( 60 );
    // rlEnableSmoothLines();
    // rlDisableBackfaceCulling();

    float halfBoxLen = 100.0/10.0;

    /// Init Objects ///
    vector<shared_ptr<FractureCube>> cubes;
    shared_ptr<FractureCube> nuCube;
    for( uint i = 0; i < 3; ++i ){
        nuCube = shared_ptr<FractureCube>( new FractureCube{ 5.0 } );
        nuCube->set_posn( uniform_vector_noise( 3.0 ) );
        cubes.push_back( nuCube );
    }
    
    // FractureCube dc{ 5.0 };
    // Icosahedron_r ic{ 5.0, Vector3Zero(), true, BLUE };

    // Camera
    Camera camera = Camera{
        Vector3{  15.0,  15.0,  15.0 }, // Position
        Vector3{   0.0,   0.0,   0.0 }, // Target
        Vector3{   0.0,   0.0,   1.0 }, // Up
        45.0, // ---------------------- FOV_y
        0 // -------------------------- Projection mode
    };

    // Load basic lighting shader
    Shader shader = LoadShader(TextFormat("shaders/lighting.vs"),
                               TextFormat("shaders/lighting.fs"));
    // Get some required shader locations
    shader.locs[SHADER_LOC_VECTOR_VIEW] = GetShaderLocation(shader, "viewPos");
    // NOTE: "matModel" location name is automatically assigned on shader loading, 
    // no need to get the location again if using that uniform name
    //shader.locs[SHADER_LOC_MATRIX_MODEL] = GetShaderLocation(shader, "matModel");
    
    // Ambient light level (some basic lighting)
    // float ambientColor[4] = { 0.1f, 0.1f, 0.1f, 1.0f };
    float ambientColor[4] = { 0.25f, 0.25f, 0.25f, 0.50f };
    int ambientLoc = GetShaderLocation(shader, "ambient");
    SetShaderValue(shader, ambientLoc, ambientColor, SHADER_UNIFORM_VEC4);

    // Using just 1 point lights
    Light light = CreateLight(
        LIGHT_POINT, 
        Vector3{ 20.0, 20.0, 20.0 }, 
        Vector3Zero(), 
        // WHITE, 
        Color{ 255, 255, 255, 125 }, 
        shader
    );

    for( shared_ptr<FractureCube>& cube : cubes ){
        cube->set_shader( shader );
    }

    // ic.set_shader( shader );
    // dc.set_shader( shader );

    ////////// RENDER LOOP /////////////////////////////////////////////////////////////////////////

    while( !WindowShouldClose() ){

        /// Begin Drawing ///
        BeginDrawing();
        BeginMode3D( camera );
        ClearBackground( BLACK );
        // BeginShaderMode( shader );

        // ic.translate( uniform_vector_noise( 0.125 ) );
        // ic.remodel();
        
        for( shared_ptr<FractureCube>& cube : cubes ){
            cube->update();
        }
        // dc.remodel();

        UpdateLightValues( shader, light );

        SetShaderValue(shader, shader.locs[SHADER_LOC_VECTOR_VIEW], &camera.position.x, SHADER_UNIFORM_VEC3);

        ///// DRAW LOOP ///////////////////////////////////////////////////
        // 
        // dc.draw();
        for( shared_ptr<FractureCube>& cube : cubes ){
            cube->draw();
        }
        
        // ic.draw();

        /// End Drawing ///
        // EndShaderMode();
        EndMode3D();

        DrawFPS( 30, 30 );

        EndDrawing();
    }

    ////////// CLEANUP /////////////////////////////////////////////////////////////////////////////
    return 0;
}