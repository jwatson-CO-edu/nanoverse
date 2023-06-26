// g++ 13_boids.cpp -std=c++17 -lraylib

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
        width     =   6.0f;
        dNear     =  25.0f;
        scale     = 150.0f;
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
        for( Basis msg : nghbrPoses ){
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

    double update_instincts_and_heading( const vector<Basis>& flockPoses ){
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
        // 4. Blend intincts
        Basis total = flocking.get_scaled_orientation( 0.45f * Nnear ) + 
                      homeSeek.get_scaled_orientation( dist/scale*5.0f ) + 
                      freeWill.get_scaled_orientation( 10.0 );
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
        
        // Begin triangle batch job
        rlBegin( RL_TRIANGLES );

        for( uint i = 0; i < Nsize; i++){
            c1    = coords[i  ][0];
            c2    = coords[i  ][1];
            c3    = coords[i+1][0];
            c4    = coords[i+1][1];
            A_i   = tailAlpha + Aspan*(Nsize-(i  ))/(1.0f*Nsize);
            A_ip1 = tailAlpha + Aspan*(Nsize-(i+1))/(1.0f*Nsize);

            // DRAW NORMALS?: https://gist.github.com/ChrisDill/09de7c818bc8618e07d8d41174704fee

            if( i%2==0 ){
                /// Triangle 1: c2, c1, c3 ///
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

                /// Triangle 2: c3, c4, c2 ///
                n2 = Vector3Normalize( Vector3CrossProduct(
                    Vector3Subtract( c2, c4 ),
                    Vector3Subtract( c3, c4 )
                ) );
                rlNormal3f( n2.x, n2.y, n2.z );
                // t2.p1 //
                rlVertex3f(c3.x, c3.y, c3.z);
                // t2.p2 //
                rlVertex3f(c4.x, c4.y, c4.z);
                // t2.p3 //
                rlColor4f(R, G, B, A_i);
                rlVertex3f(c2.x, c2.y, c2.z);
            }else{
                /// Triangle 1: c1, c3, c4 ///
                n1 = Vector3Normalize( Vector3CrossProduct(
                    Vector3Subtract( c4, c3 ),
                    Vector3Subtract( c1, c3 )
                ) );
                rlNormal3f( n1.x, n1.y, n1.z );
                // t1.p1 //
                rlColor4f(R, G, B, A_i);
                rlVertex3f(c1.x, c1.y, c1.z);
                // t1.p2 //
                rlColor4f(R, G, B, A_ip1);
                rlVertex3f(c3.x, c3.y, c3.z);
                // t1.p3 //
                rlVertex3f(c4.x, c4.y, c4.z);

                /// Triangle 2: c4, c2, c1 ///
                n2 = Vector3Normalize( Vector3CrossProduct(
                    Vector3Subtract( c1, c2 ),
                    Vector3Subtract( c4, c2 )
                ) );
                rlNormal3f( n2.x, n2.y, n2.z );
                // t2.p1 //
                rlVertex3f(c4.x, c4.y, c4.z);
                // t2.p2 //
                rlColor4f(R, G, B, A_i);
                rlVertex3f(c2.x, c2.y, c2.z);
                // t2.p3 //
                rlVertex3f(c1.x, c1.y, c1.z);
            }
        }
        // End triangle batch job
        rlEnd();
    }

};
typedef shared_ptr<BoidRibbon> rbbnPtr;



////////// MAIN ////////////////////////////////////////////////////////////////////////////////////

int main(){
    rand_seed();

    /// Window Init ///
    InitWindow( 900, 900, "Boids!" );
    SetTargetFPS( 60 );
    rlEnableSmoothLines();
    rlDisableBackfaceCulling();

    /// Init Objects ///
    vector<rbbnPtr> flock;
    for( uint i = 0; i < 300; i++ ){
        rbbnPtr nuBirb = rbbnPtr( new BoidRibbon{ 200, 1.0f, 0.0f } );
        nuBirb->headingB.Pt = Vector3{ randf( -75.0f, 75.0f ), randf( -75.0f, 75.0f ), randf( -75.0f, 75.0f ) };
        flock.push_back( nuBirb );
    }
    vector<Basis> flockPoses;

    // Camera
    Camera camera = Camera{
        Vector3{ 200.0, 200.0, 200.0 }, // Position
        Vector3{   0.0,   0.0,   0.0 }, // Target
        Vector3{   0.0,   0.0,   1.0 }, // Up
        45.0, // ---------------------- FOV_y
        0 // -------------------------- Projection mode
    };

    // Load basic lighting shader
    Shader shader = LoadShader( "shaders/lighting.vs", "shaders/lighting.fs" );
    shader.locs[ SHADER_LOC_VECTOR_VIEW ] = GetShaderLocation( shader, "viewPos" );

    // Ambient light level (some basic lighting)
    int ambientLoc = GetShaderLocation( shader, "ambient" );
    float ambColor[4] = { 0.1f, 0.1f, 0.1f, 1.0f };
    SetShaderValue( shader, ambientLoc, ambColor, SHADER_UNIFORM_VEC4 );

    // Create lights
    Light lights[ 1 ] = { 0 };
    lights[0] = CreateLight( LIGHT_POINT, (Vector3){50, 50, 50}, Vector3Zero(), RAYWHITE, shader );

    // Update the shader with the camera view vector (points towards { 0.0f, 0.0f, 0.0f })
    float cameraPos[3] = { camera.position.x, camera.position.y, camera.position.z };
    SetShaderValue(shader, shader.locs[SHADER_LOC_VECTOR_VIEW], cameraPos, SHADER_UNIFORM_VEC3);

    ////////// RENDER LOOP /////////////////////////////////////////////////////////////////////////

    while( !WindowShouldClose() ){

        /// Begin Drawing ///
        BeginDrawing();
        BeginMode3D( camera );
        ClearBackground( BLACK );

        UpdateLightValues( shader, lights[0] );

        // Activate our custom shader to be applied on next shapes/textures drawings
        BeginShaderMode( shader );

        ///// DRAW LOOP ///////////////////////////////////////////////////
        flockPoses.clear();
        for( rbbnPtr birb : flock ){  flockPoses.push_back( birb->get_Basis() );  }
        for( rbbnPtr birb : flock ){  
            birb->update_instincts_and_heading( flockPoses );
            birb->update_position( 0.50f );
            birb->draw();  
        }

        // Activate our default shader for next drawings
        EndShaderMode();

        /// End Drawing ///
        EndMode3D();

        DrawFPS( 30, 30 );

        EndDrawing();
    }

    ////////// CLEANUP /////////////////////////////////////////////////////////////////////////////
    flock.clear();
    return 0;
}
