// g++ 14_boid-ribbons.cpp -std=c++17 -O3 -lraylib

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


uint Nboids = 0;

class BoidRibbon{ public:

    /// Members ///
    Color sldClr; // Boid color
    uint  ID; // --- Identifier
    
    /// Pose ///
    Basis headingB; // Where the boid is actually pointed

    /// Way-Finding ///
    float   ur; // ----- Update rate
    float   dNear; // -- Radius of hemisphere for flocking consideration
    uint    Nnear; // -- How many neighbors are there?
    float   scale; // -- Habitat scale
    Vector3 home; // --- Don't get too far from this point
    Basis   flocking; // Flocking instinct
    Basis   homeSeek; // Home seeking instinct
    Basis   freeWill; // Drunken walk
    Basis   avoidSph; // Sphere avoidance instinct
    Sphere  fearSphr; // The sphere to fear

    /// Rendering ///
    uint /*--------------*/ Npairs; // -- Number of coordinate pairs allowed
    float /*-------------*/ headAlpha; // Beginning opacity
    float /*-------------*/ tailAlpha; // Ending    opacity
    deque<array<Vector3,2>> coords; // -- Ribbon data, listed from head to tail
    double /*------------*/ width; // --- Width of the ribbon 

    /// Constructor ///

    BoidRibbon( uint N, float Ah, float At ){
        // Build the geometry of the boid
        Npairs    = N;
        headAlpha = Ah;
        tailAlpha = At;
        width     =   6.0/10.0;
        dNear     =  12.5/10.0;
        scale     = 150.0/10.0;
        ur /*--*/ =   0.10;
        home /**/ = Vector3{0,0,0};
        sldClr    = Color{
            (ubyte) randi( 0, 255 ),
            (ubyte) randi( 0, 255 ),
            (ubyte) randi( 0, 255 ),
            255
        };
        headingB = Basis::random_Basis(); // Where the boid is actually pointed
        flocking = headingB.copy(); // ----- Flocking instinct
        homeSeek = headingB.copy(); // ----- Home seeking instinct
        freeWill = headingB.copy(); // ----- Drunken walk
        avoidSph = headingB.copy(); // ----- Sphere avoidance instinct

        Nboids++;
        ID = Nboids;
    }

    /// Methods ///

    Basis get_Basis(){
        // Get pose info for this boid
        return headingB.copy();
    }

    void push_coord_pair( const Vector3& c1, const Vector3& c2 ){
        // Add coordinates to the head of the plume
        if( coords.size() >= Npairs )  coords.pop_back(); // If queue is full, drop the tail element
        coords.push_front(  array<Vector3,2>{ c1, c2 }  );
    }

    /// Navigation ///

    Basis consider_neighbors( const vector<Basis>& nghbrPoses ){
        // Flocking instinct: Adjust bearing according to the closest neighbors in front of the boid
        Vector3 Xmean;
        Vector3 Ymean;
        Vector3 Zmean;
        Vector3 Zfly , diffVec;
        Basis   rtnMsg;
        uint    relevant = 0;
        float   dist, dotFront;
        Xmean = Vector3{0.0f, 0.0f, 0.0f};
        Ymean = Vector3{0.0f, 0.0f, 0.0f};
        Zmean = Vector3{0.0f, 0.0f, 0.0f};
        for( const Basis& msg : nghbrPoses ){
            diffVec  = Vector3Subtract( msg.Pt, headingB.Pt );
            dist     = Vector3Length( diffVec );
            if( dist > 0.0 ){
                Zfly     = headingB.Zb;
                dotFront = Vector3DotProduct( Zfly, diffVec );
                if( (dist <= dNear) || (dotFront >= 0.0f) ){
                    Xmean  = Vector3Add( Xmean, msg.Xb );
                    Ymean  = Vector3Add( Ymean, msg.Yb );
                    Zmean  = Vector3Add( Zmean, msg.Zb );
                    relevant++;
                }
            }
        }
        Nnear = relevant;
        if( relevant ){
            rtnMsg.Xb = Xmean;
            rtnMsg.Yb = Ymean;
            rtnMsg.Zb = Zmean;
            rtnMsg.orthonormalize();
        }else{
            rtnMsg.Xb = Vector3{0.0f, 0.0f, 0.0f};
            rtnMsg.Yb = Vector3{0.0f, 0.0f, 0.0f};
            rtnMsg.Zb = Vector3{0.0f, 0.0f, 0.0f};
        }
        return rtnMsg;
    }

    Basis consider_home(){
        // Home seeking instinct: Take `N_samples` and choose the one that points closest to `home`
        Basis   rtnMsg;
        Vector3 hVec = Vector3Subtract( home, headingB.Pt );
        rtnMsg.Zb = Vector3Normalize( hVec );
        rtnMsg.Xb = Vector3Normalize( Vector3CrossProduct( headingB.Yb, rtnMsg.Zb ) );
        rtnMsg.Yb = Vector3Normalize( Vector3CrossProduct( rtnMsg.Zb  , rtnMsg.Xb ) );
        return rtnMsg;
    }

    Basis consider_free_will(){
        // Drunken walk: Choose a random direction in which to nudge free will
        Basis   rtnMsg;
        Vector3 vWil = Vector3{
            randf( -1.0,  1.0 ),
            randf( -1.0,  1.0 ),
            randf( -1.0,  1.0 )
        };
        rtnMsg.Zb = Vector3Normalize( vWil );
        rtnMsg.Xb = Vector3Normalize( Vector3CrossProduct( headingB.Yb, rtnMsg.Zb ) );
        rtnMsg.Yb = Vector3Normalize( Vector3CrossProduct( rtnMsg.Zb  , rtnMsg.Xb ) );
        return rtnMsg;
    }

    Basis consider_spheres( const vector<Sphere>& sphereList ){
        // Sphere avoidance instinct
        float nearDist = 1e9;
        float sphrDist = 0.0;
        uint  i /*--*/ = 0;
        // 1. Find the closest sphere
        for( const Sphere& sphere : sphereList ){
            sphrDist = Vector3Distance( sphere.center, headingB.Pt );
            if( sphrDist < nearDist ){
                nearDist = sphrDist;
                fearSphr = sphere.copy();
            }
            i++;
        }
        // 2. Generate an avoiding heading
        Vector3 toSphr = Vector3Subtract( fearSphr.center, headingB.Pt );
        if( nearDist < fearSphr.radius ){
            Basis rtnBasis;
            rtnBasis.Zb = Vector3Scale( toSphr, -1.0f );
            rtnBasis.Yb = Vector3CrossProduct( rtnBasis.Zb, headingB.Xb );
            rtnBasis.orthonormalize();
            return rtnBasis;
        }else if( Vector3DotProduct( headingB.Zb, toSphr ) > 0.0 ){
            Basis rtnBasis;
            rtnBasis.Yb = Vector3Scale( toSphr, -1.0 );
            rtnBasis.Xb = Vector3CrossProduct( rtnBasis.Yb, headingB.Zb );
            rtnBasis.Zb = Vector3CrossProduct( rtnBasis.Xb, rtnBasis.Yb );
            rtnBasis.orthonormalize();
            return rtnBasis;
        }else{
            return headingB.copy();
        }
    }

    double update_instincts_and_heading( const vector<Basis>& flockPoses, const vector<Sphere>& spheres ){
        // Main navigation function
        // 1. Update flocking instinct
        Basis flockDrive = consider_neighbors( flockPoses );
        flocking.blend_orientations_with_factor( flockDrive, ur );
        // 2. Update home seeking instinct
        float dist /*-*/ = Vector3Distance( home, headingB.Pt );
        Basis centrDrive = consider_home();
        homeSeek.blend_orientations_with_factor( centrDrive, ur );
        // 3. Update drunken walk
        if( randf() < 0.25 ){
            Basis freewDrive = consider_free_will();
            freeWill.blend_orientations_with_factor( freewDrive, ur );
        }
        // 4. Update sphere avoidance instinct
        Basis fearDrive = consider_spheres( spheres );
        avoidSph.blend_orientations_with_factor( fearDrive, ur );
        float fearDist = Vector3Distance( fearSphr.center, headingB.Pt );
        // 4. Blend intincts
        Basis total = flocking.get_scaled_orientation( 0.45f * Nnear ) + 
                      homeSeek.get_scaled_orientation( dist/scale*5.0f ) + 
                      freeWill.get_scaled_orientation( 10.0 ) + 
                      avoidSph.get_scaled_orientation( fearSphr.radius / fearDist * 15 );
        // 5. Limit turn and set heading
        double updateTurn = Vector3Angle( total.Zb, headingB.Zb );
        double turnMax    = PI/32;
        double factor     = updateTurn/turnMax;

        headingB.blend_orientations_with_factor( total, ur/factor );
        return 0.0;
    }

    void update_position( float zThrust ){
        // Move forward and push a segment to the ribbon
        // 1. Z Thrust
        headingB.Pt = Vector3Add( headingB.Pt, Vector3Scale( headingB.Zb, zThrust ) );
        push_coord_pair(
            Vector3Add( headingB.Pt, Vector3Scale( headingB.Xb,  width/2.0f ) ),
            Vector3Add( headingB.Pt, Vector3Scale( headingB.Xb, -width/2.0f ) )
        );
    }

    ///// Rendering //////////////////////////////
    // WARNING: Requires window init to call!

    void draw(){
        // Render plume as a batch job
        uint    Nsize = coords.size()-1;        
        float   R     = sldClr.r/255.0f;
        float   G     = sldClr.g/255.0f;
        float   B     = sldClr.b/255.0f;
        float   Aspan = headAlpha - tailAlpha;
        Vector3 c1, c2, c3, c4, n1, n2;
        float   A_i;
        float   A_ip1;
        
        // // Begin triangle batch job
        // rlBegin( RL_TRIANGLES );

        // Begin triangle batch job
        rlBegin( RL_QUADS );

        for( uint i = 0; i < Nsize; i++){
            c1    = coords[i  ][0];
            c2    = coords[i  ][1];
            c3    = coords[i+1][0];
            c4    = coords[i+1][1];
            A_i   = tailAlpha + Aspan*(Nsize-(i  ))/(1.0f*Nsize);
            A_ip1 = tailAlpha + Aspan*(Nsize-(i+1))/(1.0f*Nsize);

            // DRAW NORMALS?: https://gist.github.com/ChrisDill/09de7c818bc8618e07d8d41174704fee

            /// Quad 1: c2, c1, c3, c4 ///
            n1 = Vector3Normalize( Vector3CrossProduct(
                Vector3Subtract( c3, c1 ),
                Vector3Subtract( c2, c1 )
            ) );
            rlNormal3f( n1.x, n1.y, n1.z );
            // t1.p1 //
            rlColor4f(R, G, B, A_i);
            rlVertex3f(c2.x, c2.y, c2.z);
            // t1.p2 //
            rlVertex3f(c1.x, c1.y, c1.z);
            // t1.p3 //
            rlColor4f(R, G, B, A_ip1);
            rlVertex3f(c3.x, c3.y, c3.z);
            // t2.p2 //
            rlVertex3f(c4.x, c4.y, c4.z);

            /// Quad 2: c1, c2, c4, c3 ///
            rlNormal3f( -n1.x, -n1.y, -n1.z );
            // t1.p1 //
            rlColor4f(R, G, B, A_i);
            rlVertex3f(c1.x, c1.y, c1.z);
            rlVertex3f(c2.x, c2.y, c2.z);
            // t1.p2 //
            
            // t1.p3 //
            rlColor4f(R, G, B, A_ip1);
            rlVertex3f(c4.x, c4.y, c4.z);
            rlVertex3f(c3.x, c3.y, c3.z);
            // t2.p2 //
            
        }
        // End quad batch job
        rlEnd();
    }

};
typedef shared_ptr<BoidRibbon> rbbnPtr;



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
    vector<rbbnPtr> flock;
    for( uint i = 0; i < 100; i++ ){
        rbbnPtr nuBirb = rbbnPtr( new BoidRibbon{ 200, 1.0f, 0.25f } );
        nuBirb->headingB.Pt = Vector3{ 
            randf( -halfBoxLen, halfBoxLen ), 
            randf( -halfBoxLen, halfBoxLen ), 
            randf( -halfBoxLen, halfBoxLen ) 
        };
        flock.push_back( nuBirb );
    }
    vector<Basis>  flockPoses;
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

        flockPoses.clear();
        for( rbbnPtr birb : flock ){  flockPoses.push_back( birb->get_Basis() );  }
        for( rbbnPtr birb : flock ){  
            birb->update_instincts_and_heading( flockPoses, sphereList );
            birb->update_position( 0.50/10.0 );
            birb->draw();  
        }

        

        // Activate our default shader for next drawings
        // EndShaderMode();

        /// End Drawing ///
        EndMode3D();

        DrawFPS( 30, 30 );

        EndDrawing();
    }

    ////////// CLEANUP /////////////////////////////////////////////////////////////////////////////
    flock.clear();
    return 0;
}
