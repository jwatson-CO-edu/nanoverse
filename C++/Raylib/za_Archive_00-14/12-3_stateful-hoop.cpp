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
#include <array>
using std::array;
#include <memory>
using std::shared_ptr;

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
    // A hoop for the player to fly through, Reminiscent of an Identity Disk from Tron

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

    void test_segment( const Vector3& Q, const Vector3& R ){
        // Test if a line segment QR passes thru the hoop, If so then set `scored` to true
        // Author: saxbophone, https://math.stackexchange.com/a/4657621
        bool    win   = false;
        Vector3 RmQ   = Vector3Subtract( R, Q );
        float   QRlen = Vector3Length( RmQ );
        float   num   = Vector3DotProduct( RmQ, Vector3Subtract( Q, center ) );
        float   den   = Vector3DotProduct( RmQ, RmQ );
        float   tHt, dotQR;
        Vector3 G, GmQ;
        // Test for nonzero segment length
        if( den > 0.0f ){
            tHt = num / den;
            // G is closest to the hoop center on QR, BUT may NOT be between Q and R!
            G     = Vector3Subtract( Q, Vector3Scale( RmQ, tHt ) );
            GmQ   = Vector3Subtract( G, Q );
            dotQR = Vector3DotProduct( GmQ, RmQ );
            // If dot prod negative, then center is entirely "behind" segment, otherwise test within segment
            // `G` must lie within the circle
            if( dotQR > 0.0f ){  
                win = (Vector3Length( GmQ ) < QRlen) && (Vector3Distance(G,center) <= radOuter);  
            }
        }
        // If we won, then latch in the `scored` state, Let the client code unlatch if needed
        if( win )  scored = true;
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
        Vector3 p1, p2, p3, p4, p5, p6;
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
        p5 = Vector3Add( center, Vector3Scale( radRay, radOuter+2.0*margin ) );
        
        rlBegin( RL_LINES );

        rlColor4f( clrBorder.r/255.0f, clrBorder.g/255.0f, clrBorder.b/255.0f, clrBorder.a/255.0f );

        for( uint i = 0; i < Nseg; i++ ){

            radRay = Vector3RotateByAxisAngle( radRay, normal, incr );
            p3     = Vector3Add( center, Vector3Scale( radRay, radInner-margin ) );
            p4     = Vector3Add( center, Vector3Scale( radRay, radOuter+margin ) );
            p6     = Vector3Add( center, Vector3Scale( radRay, radOuter+2.0*margin ) );

            /// Segment 1: p1-to-p3 ///
            rlVertex3f( p1.x, p1.y, p1.z );
            rlVertex3f( p3.x, p3.y, p3.z );

            /// Segment 2: p2-to-p4 ///
            rlVertex3f( p2.x, p2.y, p2.z );
            rlVertex3f( p4.x, p4.y, p4.z );

            /// Segment 3: p5-to-p6 ///
            rlVertex3f( p5.x, p5.y, p5.z );
            rlVertex3f( p6.x, p6.y, p6.z );

            p1 = p3;
            p2 = p4;
            p5 = p6;
        }

        // End border batch job
        rlEnd();
    }
};

class HoopTester{ public:
    // Test harness for `HoopTarget`
    Vector3 /*----------*/ minC;
    Vector3 /*----------*/ maxC;
    shared_ptr<HoopTarget> hoop;
    bool /*-------------*/ drwg;
    array<Vector3,2> /*-*/ tSeg;
    ulong /*------------*/ fSeq;
    ulong /*------------*/ fTrn;
    Color /*------------*/ lClr;

    HoopTester( const Vector3& minCorner, const Vector3& maxCorner, HoopTarget* target ){
        // Set the test bounds the the hoop to be tested
        minC = minCorner;
        maxC = maxCorner;
        hoop = shared_ptr<HoopTarget>( target );
        drwg = false;
        fSeq = 0;
        fTrn = 30;
        lClr = RAYWHITE;
    }

    Vector3 sample_point(){
        // Sample within the bounding box
        return Vector3{
            randf( minC.x, maxC.x ),
            randf( minC.y, maxC.y ),
            randf( minC.z, maxC.z ),
        };
    }

    array<Vector3,2> sample_segment(){
        // Return a line segment that begins and ends within the specfied bounds
        return array<Vector3,2>{ sample_point(), sample_point() };
    }
    
    void update(){
        // State transitions on multiples of `fTrn` number of frames
        if( (fSeq % fTrn) == 0 ){
            // Toggle test off, Unlatch hoop
            if( drwg ){
                hoop->scored = false;
                drwg = false;
            // Toggle test on, Query hoop
            }else{
                tSeg = sample_segment();
                hoop->test_segment( tSeg[0], tSeg[1] );
                drwg = true;
            }
        }
        fSeq++;
    }

    void draw(){
        // If we are in the testing state, draw the query segment.  Otherwise do nothing
        update();
        if( drwg ){  DrawLine3D( tSeg[0], tSeg[1], lClr);  }
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
        Vector3{ 50.0, 30.0, 50.0 }, // Position
        Vector3{ 15.0, 15.0,  0.0 }, // Target
        Vector3{  0.0,  1.0,  0.0 }, // Up
        45.0, // ---------------------- FOV_y
        0 // -------------------------- Projection mode
    };

    HoopTarget hoop{ Vector3{15.0, 15.0, 0.0}, Vector3{0.0, 0.0, 1.0}, 20.0f, 30.0f };
    HoopTester tstr{ Vector3{-10.0, -10.0, -10.0}, Vector3{40.0, 40.0, 40.0}, &hoop };

    while( !WindowShouldClose() ){
        
        /// Begin Drawing ///
        BeginDrawing();
        BeginMode3D( camera );
        ClearBackground( BLACK );

        ///// DRAW LOOP //////////////////////////
        hoop.draw();
        tstr.draw();

        /// End Drawing ///
        EndMode3D();
        EndDrawing();
    }

    return 0;
}