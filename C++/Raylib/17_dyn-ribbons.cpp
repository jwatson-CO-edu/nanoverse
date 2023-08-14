// g++ 17_dyn-ribbons.cpp -std=c++17 -lraylib
// Re-implement Boid Ribbons **without** `Model`s!
// 2023-08-14: Do not break into smaller files until everything works

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

/// Raylib ///
#include "raylib.h"
#include "raymath.h"
#include <rlgl.h>


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
        Vector3Subtract( tri[0], tri[1] ),
        Vector3Subtract( tri[2], tri[1] )
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



////////// TOYS ////////////////////////////////////////////////////////////////////////////////////


class DynaMesh{ public:
    // Straightforward container for a `Mesh` with changing geometry
    // 2023-08-14: This class assumes that the mesh will have a constant size for each buffer, though their values might change

    /// Members ///
    vector<triPnts> tris; // Dynamic geometry, each array is a facet of 3x vertices
    vector<triPnts> nrms; // Normal vector for each facet
    vector<triClrs> clrs; // Vertex colors for each facet
    Mesh /*------*/ mesh; // Raylib mesh geometry

    
    /// Memory Methods ///

    void init_mesh_memory( uint Ntri ){
        // Create the `Mesh` struct and allocate memory there
        uint Npts = Ntri*3;

        // 1. Init mesh
        mesh = Mesh{};
        mesh.triangleCount = Ntri;
        mesh.vertexCount   = Npts;

        // 2. Init memory
        mesh.vertices = (float* ) MemAlloc(Npts*3 * sizeof( float  )); // 3 vertices, 3 coordinates each (x, y, z)
        mesh.indices  = (ushort*) MemAlloc(Npts   * sizeof( ushort ));
        mesh.normals  = (float* ) MemAlloc(Npts*3 * sizeof( float  )); // 3 vertices, 3 coordinates each (x, y, z)
        mesh.colors   = (u_char*) MemAlloc(Npts*4 * sizeof( u_char )); // 3 vertices, 4 coordinates each (r, g, b, a)

        // 3. Dummy load to GPU
        UploadMesh( &mesh, true );
    }

    void load_mesh_buffers( bool loadGeo = true, bool loadColor = false ){
        // Load geometry (and color) info into `mesh` buffers
        // 2023-08-14: For now assume that the facet indices do NOT change!

        // 0. Init
        ulong k    = 0; // Vertex _ counter
        ulong l    = 0; // Index __ counter
        ulong m    = 0; // Color __ counter

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

    /// Constructors ///

    DynaMesh( uint Ntri ){

    }

    /// Geometry Methods ///

    void push_triangle_w_norms( triPnts tri ){
        // Add one triangle
        Vector3 norm = normal_of_tiangle( tri );  
        tris.push_back( tri );
        nrms.push_back({ norm, norm, norm });
    }

    ///// Drawing Context Methods ////////////////
    // NOTE: Method beyond this point require a drawing context being instantiated before calling!

    /// Rendering ///

    void draw(){
        // Render the mesh
        // 2023-08-13: Let's stop thinking about `Model`s unless they are absolutely necessary!
        DrawMesh( mesh, LoadMaterialDefault(), MatrixIdentity() ); 
    }

};


struct DynCube{
    // A dynamic cube

    /// Members ///
    vector<triPnts> tris; // Dynamic geometry, each array is a facet of 3x vertices
    vector<triPnts> nrms; // Normal vector for each facet
    vector<triClrs> clrs; // Vertex colors for each facet
    Mesh /*------*/ mesh; // Raylib mesh geometry
    bool /*------*/ dynG; // Flag for whether geometry is dynamic

    /// Constructors ///

    void push_triangle_w_norms( triPnts tri ){
        // Add one triangle
        Vector3 norm = normal_of_tiangle( tri );  
        tris.push_back( tri );
        nrms.push_back({ norm, norm, norm });
    }

    void init_mesh_memory( uint Ntri ){
        // Create the `Mesh` struct and allocate memory there
        uint Npts = Ntri*3;

        // 1. Init mesh
        mesh = Mesh{};
        mesh.triangleCount = Ntri;
        mesh.vertexCount   = Npts;

        // 2. Init memory
        mesh.vertices = (float* ) MemAlloc(Npts*3 * sizeof( float  )); // 3 vertices, 3 coordinates each (x, y, z)
        mesh.indices  = (ushort*) MemAlloc(Npts   * sizeof( ushort ));
        mesh.normals  = (float* ) MemAlloc(Npts*3 * sizeof( float  )); // 3 vertices, 3 coordinates each (x, y, z)
        mesh.colors   = (u_char*) MemAlloc(Npts*4 * sizeof( u_char )); // 3 vertices, 4 coordinates each (r, g, b, a)
    }

    void load_mesh_buffers( bool loadGeo = true, bool loadColor = false ){
        // Load geometry (and color) info into `mesh` buffers
        ulong k    = 0; // Vertex _ counter
        ulong l    = 0; // Index __ counter
        ulong m    = 0; // Color __ counter
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
    }

    DynCube( float sideLen ){
        // Create tris that make up the cube

        // 0. Init
        triPnts pushTri;
        triClrs pushClr;
        Vector3 norm;
        vvec3   V;
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

        // 4. Init mesh
        dynG = true;
        uint Ntri = 12;
        init_mesh_memory( Ntri );

        // 5. Load buffers
        load_mesh_buffers( true, true );

        // 5. Send geometry to GPU
        UploadMesh( &mesh, dynG );
    }

    void update(){
        for( triPnts& tri : tris ){
            tri[0] = uniform_vector_noise( tri[0], 0.125 );
            tri[1] = uniform_vector_noise( tri[1], 0.125 );
            tri[2] = uniform_vector_noise( tri[2], 0.125 );
            // 2023-08-13: Purposely avoiding a normal update until a shader is in use
        }
        load_mesh_buffers( true, false );
        UpdateMeshBuffer( 
            mesh, // -------------------------------- `Mesh` object
            VERTEX_BUFFER_IDX, // ------------------- VBO Index
            mesh.vertices, // ----------------------- Array of data 
            mesh.vertexCount*3 * sizeof( float  ), // Total array size
            0 // ------------------------------------ Starting index in array
        );
    }

    void draw(){
        // Render the mesh
        // 2023-08-13: Let's stop thinking about `Model`s unless they are absolutely necessary!
        DrawMesh( mesh, LoadMaterialDefault(), MatrixIdentity() ); 
    }

};



////////// MAIN ////////////////////////////////////////////////////////////////////////////////////

int main(){
    rand_seed();

    /// Window Init ///
    InitWindow( 900, 900, "Dynamic Box!" );
    SetTargetFPS( 60 );
    // rlEnableSmoothLines();
    rlDisableBackfaceCulling();

    float halfBoxLen = 100.0/10.0;

    /// Init Objects ///
    DynCube dc{ 5.0 };

    // Camera
    Camera camera = Camera{
        Vector3{ 300.0/10.0, 150.0/10.0, 200.0/10.0 }, // Position
        Vector3{   0.0,   0.0,   0.0 }, // Target
        Vector3{   0.0,   0.0,   1.0 }, // Up
        45.0, // ---------------------- FOV_y
        0 // -------------------------- Projection mode
    };

    ////////// RENDER LOOP /////////////////////////////////////////////////////////////////////////

    while( !WindowShouldClose() ){

        /// Begin Drawing ///
        BeginDrawing();
        BeginMode3D( camera );
        ClearBackground( BLACK );

        ///// DRAW LOOP ///////////////////////////////////////////////////
        dc.update();
        dc.draw();

        /// End Drawing ///
        EndMode3D();

        DrawFPS( 30, 30 );

        EndDrawing();
    }

    ////////// CLEANUP /////////////////////////////////////////////////////////////////////////////
    return 0;
}