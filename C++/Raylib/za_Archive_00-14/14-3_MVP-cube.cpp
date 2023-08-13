// g++ 14-3_MVP-cube.cpp -std=c++17 -lraylib

////////// INIT ////////////////////////////////////////////////////////////////////////////////////

///// Imports ////////////////////////////////////

/// Standard ///
#include <deque>
using std::deque;
#include <array>
using std::array;
#include <vector>
using std::vector;
#include <stdlib.h>  // srand, rand
#include <time.h>
#include <iostream>
using std::cout, std::endl;

/// Raylib ///
#include "raylib.h"
#include "raymath.h"
#include <rlgl.h>


////////// HELPER FUNCTIONS ////////////////////////////////////////////////////////////////////////

float randf(){
    // Return a pseudo-random number between 0.0 and 1.0
    return  1.0f * rand() / RAND_MAX;
}

void rand_seed(){  srand( time(NULL) );  } // Seed RNG with unpredictable time-based seed



////////// TOYS ////////////////////////////////////////////////////////////////////////////////////

struct DynaCube{
    // A simple cube with optionally-drawn facets

    /// Members ///
    deque<array<Vector3,3>> triangles; // - Dynamic geometry, each array is a facet of 3x vertices
    deque<array<Vector3,3>> normals; // --- Normal vector for each facet
    deque<array<Color,3>>   facetColors; // Vertex colors for each facet
    deque<bool> /*-------*/ activeTri; // - Flags for whether to draw each triangle
    Mesh /*--------------*/ mesh; // ------ Raylib mesh geometry
	Model /*-------------*/ model; // ----- Raylib drawable model
    size_t /*------------*/ Nfrms; // ----- Frame divisor for state change
    size_t /*------------*/ iFrms; // ----- Age of cube in frames
    uint /*--------------*/ Nact = 0; // -- Active triangle counter

    /// Constructor ///
    DynaCube( float sideLen ){
        // Create triangles that make up the cube
        // -1. Init state
        Nfrms = 40;
        iFrms =  0;
        // 0. Init
        array<Vector3,3> pushArr;
        array<Color,3>   pushClr;
        Vector3 /*----*/ norm;
        vector<Vector3>  V;
        float /*------*/ halfLen = sideLen/2.0;
        Color /*------*/ R{ 255,   0,   0, 255 };
        Color /*------*/ G{   0, 255,   0, 255 };
        Color /*------*/ B{   0,   0, 255, 255 };
        // 1. Establish vertices
        V.push_back( Vector3{ -halfLen, -halfLen, -halfLen } );
        V.push_back( Vector3{ -halfLen, -halfLen,  halfLen } );
        V.push_back( Vector3{ -halfLen,  halfLen, -halfLen } );
        V.push_back( Vector3{ -halfLen,  halfLen,  halfLen } );
        V.push_back( Vector3{  halfLen, -halfLen, -halfLen } );
        V.push_back( Vector3{  halfLen, -halfLen,  halfLen } );
        V.push_back( Vector3{  halfLen,  halfLen, -halfLen } );
        V.push_back( Vector3{  halfLen,  halfLen,  halfLen } );
        // 2. Build triangles
        pushArr = { V[0], V[3], V[2] };  triangles.push_back( pushArr );
        norm    = Vector3Normalize( Vector3CrossProduct(
            Vector3Subtract( V[2], V[3] ),
            Vector3Subtract( V[0], V[3] )
        ));  normals.push_back({ norm, norm, norm });
        pushArr = { V[0], V[1], V[3] };  triangles.push_back( pushArr );
        norm    = Vector3Normalize( Vector3CrossProduct(
            Vector3Subtract( V[3], V[1] ),
            Vector3Subtract( V[0], V[1] )
        ));  normals.push_back({ norm, norm, norm });
        pushArr = { V[6], V[4], V[0] };  triangles.push_back( pushArr );
        norm    = Vector3Normalize( Vector3CrossProduct(
            Vector3Subtract( V[0], V[4] ),
            Vector3Subtract( V[6], V[4] )
        ));  normals.push_back({ norm, norm, norm });
        pushArr = { V[6], V[0], V[2] };  triangles.push_back( pushArr );
        norm    = Vector3Normalize( Vector3CrossProduct(
            Vector3Subtract( V[2], V[0] ),
            Vector3Subtract( V[6], V[0] )
        ));  normals.push_back({ norm, norm, norm });
        pushArr = { V[0], V[4], V[5] };  triangles.push_back( pushArr );
        norm    = Vector3Normalize( Vector3CrossProduct(
            Vector3Subtract( V[5], V[4] ),
            Vector3Subtract( V[0], V[4] )
        ));  normals.push_back({ norm, norm, norm });
        pushArr = { V[0], V[5], V[1] };  triangles.push_back( pushArr );
        norm    = Vector3Normalize( Vector3CrossProduct(
            Vector3Subtract( V[1], V[5] ),
            Vector3Subtract( V[0], V[5] )
        ));  normals.push_back({ norm, norm, norm });
        pushArr = { V[7], V[6], V[2] };  triangles.push_back( pushArr );
        norm    = Vector3Normalize( Vector3CrossProduct(
            Vector3Subtract( V[2], V[6] ),
            Vector3Subtract( V[7], V[6] )
        ));  normals.push_back({ norm, norm, norm });
        pushArr = { V[7], V[2], V[3] };  triangles.push_back( pushArr );
        norm    = Vector3Normalize( Vector3CrossProduct(
            Vector3Subtract( V[3], V[2] ),
            Vector3Subtract( V[7], V[2] )
        ));  normals.push_back({ norm, norm, norm });
        pushArr = { V[4], V[6], V[7] };  triangles.push_back( pushArr );
        norm    = Vector3Normalize( Vector3CrossProduct(
            Vector3Subtract( V[7], V[6] ),
            Vector3Subtract( V[4], V[6] )
        ));  normals.push_back({ norm, norm, norm });
        pushArr = { V[4], V[7], V[5] };  triangles.push_back( pushArr );
        norm    = Vector3Normalize( Vector3CrossProduct(
            Vector3Subtract( V[5], V[7] ),
            Vector3Subtract( V[4], V[7] )
        ));  normals.push_back({ norm, norm, norm });
        pushArr = { V[1], V[5], V[7] };  triangles.push_back( pushArr );
        norm    = Vector3Normalize( Vector3CrossProduct(
            Vector3Subtract( V[7], V[5] ),
            Vector3Subtract( V[1], V[5] )
        ));  normals.push_back({ norm, norm, norm });
        pushArr = { V[1], V[7], V[3] };  triangles.push_back( pushArr );
        norm    = Vector3Normalize( Vector3CrossProduct(
            Vector3Subtract( V[3], V[7] ),
            Vector3Subtract( V[1], V[7] )
        ));  normals.push_back({ norm, norm, norm });
        // 3. Build Colors
        pushClr = { R, G, B };  facetColors.push_back( pushClr );
        pushClr = { B, R, G };  facetColors.push_back( pushClr );
        pushClr = { G, B, R };  facetColors.push_back( pushClr );
        pushClr = { R, G, B };  facetColors.push_back( pushClr );
        pushClr = { B, R, G };  facetColors.push_back( pushClr );
        pushClr = { G, B, R };  facetColors.push_back( pushClr );
        pushClr = { R, G, B };  facetColors.push_back( pushClr );
        pushClr = { B, R, G };  facetColors.push_back( pushClr );
        pushClr = { G, B, R };  facetColors.push_back( pushClr );
        pushClr = { R, G, B };  facetColors.push_back( pushClr );
        pushClr = { B, R, G };  facetColors.push_back( pushClr );
        pushClr = { G, B, R };  facetColors.push_back( pushClr );
        // 4. Set active triangles
        activeTri.push_back( true );  activeTri.push_back( true );  
        activeTri.push_back( true );  activeTri.push_back( true );  
        activeTri.push_back( true );  activeTri.push_back( true );  
        activeTri.push_back( true );  activeTri.push_back( true );  
        activeTri.push_back( true );  activeTri.push_back( true );  
        activeTri.push_back( true );  activeTri.push_back( true );  
        // 4. Init mesh
        mesh = Mesh{};
        uint Ntri = 12;
        uint Npts = Ntri*3;
        mesh.triangleCount = Ntri;
        mesh.vertexCount   = Npts;
        // 4. Init memory
        mesh.vertices = (float* ) MemAlloc(Npts*3 * sizeof( float  )); // 3 vertices, 3 coordinates each (x, y, z)
        mesh.indices  = (ushort*) MemAlloc(Npts   * sizeof( ushort ));
        mesh.normals  = (float* ) MemAlloc(Npts*3 * sizeof( float  )); // 3 vertices, 3 coordinates each (x, y, z)
        mesh.colors   = (u_char*) MemAlloc(Npts*4 * sizeof( u_char )); // 3 vertices, 4 coordinates each (r, g, b, a)
        UploadMesh( &mesh, true );
    }

    void update_and_write_triangles(){
        // Choose the active triangles and write their geometry to memory
        ++iFrms;
        
        if( iFrms%Nfrms == 0 ){
            uint  i    = 0; // Triangle counter
            ulong k    = 0; // Vertex _ counter
            ulong l    = 0; // Index __ counter
            ulong m    = 0; // Color __ counter
            Nact = 0;
            for( bool& p_active : activeTri ){
                p_active = (randf() < 0.50);
                if( p_active ){
                    ++Nact;
                    for( u_char j = 0; j < 3; j++ ){
                        mesh.normals[k]  = normals[i][j].x;
                        mesh.vertices[k] = triangles[i][j].x;    k++;
                        mesh.normals[k]  = normals[i][j].y;
                        mesh.vertices[k] = triangles[i][j].y;    k++;
                        mesh.normals[k]  = normals[i][j].z;
                        mesh.vertices[k] = triangles[i][j].z;    k++;
                        mesh.indices[l]  = l; /*--------------*/ l++; 
                        mesh.colors[m]   = facetColors[i][j].r;  m++;
                        mesh.colors[m]   = facetColors[i][j].g;  m++;
                        mesh.colors[m]   = facetColors[i][j].b;  m++;
                        mesh.colors[m]   = facetColors[i][j].a;  m++;
                    }
                }
                ++i;
            }
            mesh.triangleCount = Nact;
            mesh.vertexCount   = Nact*3;
            
            UploadMesh( &mesh, false );
            model = LoadModelFromMesh( mesh );
            model.transform = MatrixIdentity();
        }
        cout << iFrms << ", " << Nact << endl;
    }

    void draw(){
        if( Nact ){
            // DrawMesh( mesh, ,MatrixIdentity() )
            DrawModel( model, Vector3Zero(), 1.0, WHITE );
        }
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
    DynaCube dc{ 5.0 };

    // Camera
    Camera camera = Camera{
        Vector3{ 200.0/10.0, 200.0/10.0, 200.0/10.0 }, // Position
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

        // Activate our custom shader to be applied on next shapes/textures drawings
        // BeginShaderMode( shader );

        // UpdateLightValues( shader, lights[0] );

        

        ///// DRAW LOOP ///////////////////////////////////////////////////
        dc.update_and_write_triangles();
        dc.draw();
        

        // Activate our default shader for next drawings
        // EndShaderMode();

        /// End Drawing ///
        EndMode3D();

        DrawFPS( 30, 30 );

        EndDrawing();
    }

    ////////// CLEANUP /////////////////////////////////////////////////////////////////////////////
    return 0;
}
