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

        // FIXME, START HERE: DRAW THE HOOP IN EITHER INIT OR SCORED STATE

    }

};