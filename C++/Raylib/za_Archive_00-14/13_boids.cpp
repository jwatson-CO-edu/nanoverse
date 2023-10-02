// g++ 13_boids.cpp -std=c++17 -O3 -lraylib -o boids.out

////////// INIT ////////////////////////////////////////////////////////////////////////////////////

///// Imports ////////////////////////////////////

/// Standard ///
#include <algorithm>
using std::clamp;

/// Local ///
#include "utils.hpp"
#include "rl_toybox.hpp"



////////// TOYS ////////////////////////////////////////////////////////////////////////////////////
uint Nboids = 0;

class Boid : public TriModel{ public:
    // A flocking entity like a bird

    /// Members ///
    float length; // - Length of the boid
    float width; // -- Width of the boid
    Color sldClr; // - Boid color
    uint  ID; // ----- Identifier
    
    /// Way-Finding ///
    float   ur; // ----- Update rate
    float   dNear; // -- Radius of hemisphere for flocking consideration
    uint    Nnear; // -- How many neighbors are there?
    float   scale; // -- Habitat scale
    Vector3 home; // --- Don't get too far from this point
    Basis   flocking; // Flocking instinct
    Basis   homeSeek; // Home seeking instinct
    Basis   freeWill; // Drunken walk
    Basis   headingB; // Where the boid is actually pointed

    /// Constructor ///

    Boid( float len, float wdt ) : TriModel( 2 ){
        // Build the geometry of the boid
        length = len;
        width  = wdt;
        dNear  = length * 5.0f;
        scale  = 150.0f;
        ur     =   0.10;
        home   = Vector3{0,0,0};
        sldClr = Color{
            (ubyte) randi( 0, 255 ),
            (ubyte) randi( 0, 255 ),
            (ubyte) randi( 0, 255 ),
            255
        };
        headingB = Basis::random_Basis(); // Where the boid is actually pointed
        flocking = headingB.copy(); // ----- Flocking instinct
        homeSeek = headingB.copy(); // ----- Home seeking instinct
        freeWill = headingB.copy(); // ----- Drunken walk

        load_tri( // Horizontal Plane
            Vector3{  width/2, 0.0f, 0.0f   }, 
            Vector3{ -width/2, 0.0f, 0.0f   }, 
            Vector3{  0.0f   , 0.0f, length }
        );
        load_tri( // Vertical Plane
            Vector3{  0.0f,  width/2, 0.0f   }, 
            Vector3{  0.0f, -width/2, 0.0f   }, 
            Vector3{  0.0f, 0.0f    , length }
        );

        Nboids++;
        ID = Nboids;
    }

    Basis get_Basis(){
        // Get pose info for this boid
        return basis_from_transform_and_point( T, XYZ );
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
            diffVec  = Vector3Subtract( msg.Pt, XYZ );
            dist     = Vector3Length( diffVec );
            if( dist > 0.0 ){
                Zfly     = Vector3Transform( Vector3{0.0f, 0.0f, 1.0f}, T );
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

    void load_heading(){
        // Transfer the heading basis to the orientation matrix
        // 2023-06-24: Assume that `T` is column major
        T.m0  = headingB.Xb.x;
        T.m1  = headingB.Xb.y;
        T.m2  = headingB.Xb.z;
        T.m4  = headingB.Yb.x;
        T.m5  = headingB.Yb.y;
        T.m6  = headingB.Yb.z;
        T.m8  = headingB.Zb.x;
        T.m9  = headingB.Zb.y;
        T.m10 = headingB.Zb.z;
        model.transform = T;
    }

    Basis consider_home(){
        // Home seeking instinct: Take `N_samples` and choose the one that points closest to `home`
        Basis   rtnMsg;
        Vector3 hVec = Vector3Subtract( home, XYZ );
        rtnMsg.Zb = Vector3Normalize( hVec );
        rtnMsg.Xb = Vector3Normalize( Vector3CrossProduct( Vector3Transform( Vector3{0.0, 1.0, 0.0}, T ) , rtnMsg.Zb ) );
        rtnMsg.Yb = Vector3Normalize( Vector3CrossProduct( rtnMsg.Zb, rtnMsg.Xb ) );
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
        rtnMsg.Xb = Vector3Normalize( Vector3CrossProduct( Vector3Transform( Vector3{0.0, 1.0, 0.0}, T ) , rtnMsg.Zb ) );
        rtnMsg.Yb = Vector3Normalize( Vector3CrossProduct( rtnMsg.Zb, rtnMsg.Xb ) );
        return rtnMsg;
    }

    double update_instincts_and_heading( const vector<Basis>& flockPoses ){
        // Main navigation function
        // 1. Update flocking instinct
        Basis flockDrive = consider_neighbors( flockPoses );
        flocking.blend_orientations_with_factor( flockDrive, ur );
        // 2. Update home seeking instinct
        float dist /*-*/ = Vector3Distance( home, XYZ );
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
        load_heading();
        return 0.0;
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
        // Draw the model
        DrawModel( model, XYZ, 1.00, sldClr );  
    }

};
typedef shared_ptr<Boid> boidPtr;



////////// MAIN ////////////////////////////////////////////////////////////////////////////////////

int main(){
    rand_seed();

    /// Window Init ///
    InitWindow( 700, 700, "Boids!" );
    SetTargetFPS( 60 );
    rlEnableSmoothLines();
    rlDisableBackfaceCulling();

    /// Init Objects ///
    vector<boidPtr> flock;
    for( uint i = 0; i < 100; i++ ){
        boidPtr nuBirb = boidPtr( new Boid{ 10.0f*1.25f, 7.0f*1.25f } );
        nuBirb->XYZ = Vector3{ randf( -75.0f, 75.0f ), randf( -75.0f, 75.0f ), randf( -75.0f, 75.0f ) };
        nuBirb->load_geo();
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

    ////////// RENDER LOOP /////////////////////////////////////////////////////////////////////////

    while( !WindowShouldClose() ){

        /// Begin Drawing ///
        BeginDrawing();
        BeginMode3D( camera );
        ClearBackground( BLACK );

        ///// DRAW LOOP ///////////////////////////////////////////////////
        flockPoses.clear();
        for( boidPtr birb : flock ){  flockPoses.push_back( birb->get_Basis() );  }
        for( boidPtr birb : flock ){  
            birb->update_instincts_and_heading( flockPoses );
            birb->z_thrust( 0.50f );
            birb->draw();  
        }

        /// End Drawing ///
        EndMode3D();

        // DrawFPS( 30, 30 );

        EndDrawing();
    }

    ////////// CLEANUP /////////////////////////////////////////////////////////////////////////////
    flock.clear();
    return 0;
}
