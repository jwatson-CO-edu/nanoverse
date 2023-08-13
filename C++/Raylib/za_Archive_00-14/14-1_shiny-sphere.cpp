// g++ 14-1_shiny-sphere.cpp -std=c++17 -lraylib

////////// INIT ////////////////////////////////////////////////////////////////////////////////////

///// Imports ////////////////////////////////////

/// Standard ///
#include <algorithm>
using std::clamp;

/// Raylib ///
#include "raylib.h"
#include "raymath.h"

/// Local ///
#include "utils.hpp"
#include "rl_toybox.hpp"
#define RLIGHTS_IMPLEMENTATION
#include "rlights.h"



////////// TOYS ////////////////////////////////////////////////////////////////////////////////////

struct Sphere{
    // Container struct for an obstacle to avoid
    
    /// Members ///
    Vector3 center;
    double  radius;
    Model   model;
    Color   color;

    /// Constructors ///
    
    Sphere(){
        center = Vector3Zero();
        radius = 1.0;
        color  = GRAY;
        model  = LoadModelFromMesh( GenMeshSphere( radius, 32, 32 ) );
    }

    Sphere( const Vector3& cntr, double rad ){
        center = cntr;
        radius = rad;
        color  = GRAY;
        model  = LoadModelFromMesh( GenMeshSphere( radius, 32, 32 ) );
    }

    /// Methods ///

    void draw(){
        // DrawSphere( center, radius, GRAY ); 
        DrawModel( model, center, 1.0f, color );
    }

    Sphere copy() const {  return Sphere{ center, radius };  }
};

////////// MAIN ////////////////////////////////////////////////////////////////////////////////////

int main(){
    rand_seed();

    /// Window Init ///
    InitWindow( 900, 900, "Boid-like Ribbons!" );
    SetTargetFPS( 60 );
    // rlEnableSmoothLines();
    // rlDisableBackfaceCulling();

    float halfBoxLen = 100.0/10.0;

    /// Init Objects ///
    vector<Sphere> sphereList;
    for( uint i = 0; i < 16; i++ ){
        sphereList.push_back( Sphere{ 
            Vector3{
                randf( -halfBoxLen*1.25, halfBoxLen*1.25 ), 
                randf( -halfBoxLen*1.25, halfBoxLen*1.25 ), 
                randf( -halfBoxLen*1.25, halfBoxLen*1.25 ) 
            }, 
            randf( 10/10.0, 40/10.0 )
        } );
    }

    // Camera
    Camera camera = Camera{
        Vector3{ 200.0/10.0, 200.0/10.0, 200.0/10.0 }, // Position
        Vector3{   0.0,   0.0,   0.0 }, // Target
        Vector3{   0.0,   0.0,   1.0 }, // Up
        45.0, // ---------------------- FOV_y
        0 // -------------------------- Projection mode
    };

    // Load basic lighting shader
    // Shader shader = LoadShader( "shaders/lighting.vs", "shaders/lighting.fs" );
    Shader shader = LoadShader( "shaders/fogLight.vs", "shaders/fogLight.fs" );
    shader.locs[SHADER_LOC_MATRIX_MODEL] = GetShaderLocation(shader, "matModel");
    shader.locs[SHADER_LOC_VECTOR_VIEW] = GetShaderLocation(shader, "viewPos");

    // Ambient light level
    int ambientLoc = GetShaderLocation(shader, "ambient");
    float ambientCol[4] = { 0.2f, 0.2f, 0.2f, 1.0f };
    SetShaderValue(shader, ambientLoc, ambientCol, SHADER_UNIFORM_VEC4);

    int fColorLoc = GetShaderLocation(shader, "fogColor");
    float fogColor[4] = { 0.2f, 0.2f, 0.2f, 1.0f };
    SetShaderValue(shader, ambientLoc, fogColor, SHADER_UNIFORM_VEC4);

    float fogDensity = 0.015f;
    int fogDensityLoc = GetShaderLocation(shader, "fogDensity");
    SetShaderValue(shader, fogDensityLoc, &fogDensity, SHADER_UNIFORM_FLOAT);

    // Using just 1 point lights
    CreateLight(LIGHT_POINT, (Vector3){ 100/10.0, 100/10.0, 100/10.0 }, Vector3Zero(), WHITE, shader);

    for( Sphere& sphere : sphereList ){
        sphere.model.materials[0].shader = shader;
    }

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
        
        for( Sphere& sphere : sphereList ){
            sphere.draw();
        }

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
