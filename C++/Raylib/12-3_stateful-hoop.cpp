// g++ 12-3_stateful-hoop.cpp -std=c++17 -lraylib

////////// INIT ////////////////////////////////////////////////////////////////////////////////////

///// Imports ////////////////////////////////////
/// Standard ///
#include <iostream>
using std::cout, std::endl, std::ostream;
#include <algorithm> 
using std::min, std::max;
#include <deque>
using std::deque;

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
typedef unsigned short  ushort;



////////// CLASSES /////////////////////////////////////////////////////////////////////////////////

class HoopTarget{ public:
    // A hoop for the player to fly through

    /// Members ///
    Vector3 center;
    Vector3 normal;
    float   radInner;
    float   radOuter;
    Color   clrSolidInit;
    Color   clrSolidWin;
    Color   clrBorder;
    uint    Nseg;
    bool    scored;

    HoopTarget( Vector3 origin, Vector3 faceDirection, float ID, float OD ){
        center /*-*/ = origin;
        normal /**/  = Vector3Normalize( faceDirection ); // Enforce normal vector
        radInner     = ID/2.0f;
        radOuter     = OD/2.0f;
        clrSolidInit = SKYBLUE;
        clrSolidWin  = SKYBLUE;
        clrBorder    = RAYWHITE;
        Nseg /*---*/ = 40;
        scored /*-*/ = false;
        // Make the hoop semi-transparent until it is scored
        clrSolidInit.a = 125;
    }

    void draw(){
        // Render hoop as a batch job

        Vector3 radRay = Vector3Normalize( Vector3CrossProduct( 
            normal, 
            Vector3Normalize(  Vector3{ randf(-1.0, 1.0), randf(-1.0, 1.0), randf(-1.0, 1.0) }  )    
        ) );
        Vector3 bgnRay = radRay;
        float   incr   = 2.0f * PI / (1.0f * Nseg);
        float   margin = 0.02 * radOuter;
        Vector3 p1, p2, p3, p4;
        p1 = Vector3Add( center, Vector3Scale( radRay, radInner ) );
        p2 = Vector3Add( center, Vector3Scale( radRay, radOuter ) );

        // Begin triangle batch job
        rlBegin( RL_TRIANGLES );

        if( scored ) rlColor4f( clrSolidWin.r/255.0f , clrSolidWin.g/255.0f , clrSolidWin.b/255.0f , clrSolidWin.a/255.0f  );
        else /*---*/ rlColor4f( clrSolidInit.r/255.0f, clrSolidInit.g/255.0f, clrSolidInit.b/255.0f, clrSolidInit.a/255.0f );

        for( uint i = 0; i < Nseg; i++ ){

            radRay = Vector3RotateByAxisAngle( radRay, normal, incr );
            p3     = Vector3Add( center, Vector3Scale( radRay, radInner ) );
            p4     = Vector3Add( center, Vector3Scale( radRay, radOuter ) );

            /// Triangle 1: p2, p1, p3 ///
            rlVertex3f( p2.x, p2.y, p2.z );
            rlVertex3f( p1.x, p1.y, p1.z );
            rlVertex3f( p3.x, p3.y, p3.z );

            /// Triangle 2: c3, c4, c2 ///
            rlVertex3f( p3.x, p3.y, p3.z );
            rlVertex3f( p4.x, p4.y, p4.z );
            rlVertex3f( p2.x, p2.y, p2.z );

            p1 = p3;
            p2 = p4;
        }

        // End triangle batch job
        rlEnd();
        
        // Begin border batch job
        radRay = bgnRay;
        p1 = Vector3Add( center, Vector3Scale( radRay, radInner-margin ) );
        p2 = Vector3Add( center, Vector3Scale( radRay, radOuter+margin ) );
        
        rlBegin( RL_LINES );

        rlColor4f( clrBorder.r/255.0f, clrBorder.g/255.0f, clrBorder.b/255.0f, clrBorder.a/255.0f );

        for( uint i = 0; i < Nseg; i++ ){

            radRay = Vector3RotateByAxisAngle( radRay, normal, incr );
            p3     = Vector3Add( center, Vector3Scale( radRay, radInner-margin ) );
            p4     = Vector3Add( center, Vector3Scale( radRay, radOuter+margin ) );

            /// Segment 1: p1-to-p3 ///
            rlVertex3f( p1.x, p1.y, p1.z );
            rlVertex3f( p3.x, p3.y, p3.z );

            /// Segment 2: p2-to-p4 ///
            rlVertex3f( p2.x, p2.y, p2.z );
            rlVertex3f( p4.x, p4.y, p4.z );

            p1 = p3;
            p2 = p4;
        }

        // End border batch job
        rlEnd();
    }
};


////////// MAIN ////////////////////////////////////////////////////////////////////////////////////

int main(){

    rand_seed();

    /// Window Init ///
    InitWindow( 600, 600, "Hoop!" );
    SetTargetFPS( 60 );
    rlEnableSmoothLines();
    rlDisableBackfaceCulling();

    // Camera
    Camera camera = Camera{
        Vector3{ 15.0, 15.0, 50.0 }, // Position
        Vector3{ 15.0, 15.0,  0.0 }, // Target
        Vector3{  0.0,  1.0,  0.0 }, // Up
        45.0, // ---------------------- FOV_y
        0 // -------------------------- Projection mode
    };

    HoopTarget hoop{ Vector3{15.0, 15.0, 0.0}, Vector3{0.0, 0.0, 1.0}, 20.0f, 30.0f };

    while( !WindowShouldClose() ){
        
        /// Begin Drawing ///
        BeginDrawing();
        BeginMode3D( camera );
        ClearBackground( BLACK );

        ///// DRAW LOOP //////////////////////////
        hoop.draw();

        /// End Drawing ///
        EndMode3D();
        EndDrawing();
    }

    return 0;
}