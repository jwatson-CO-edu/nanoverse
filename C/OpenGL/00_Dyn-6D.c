////////// INIT ////////////////////////////////////////////////////////////////////////////////////

#include "toolbox.h"

////////// PROGRAM STRUCTS /////////////////////////////////////////////////////////////////////////

typedef struct{
    union{ float x;  float xDot; };
    union{ float y;  float yDot; };
    union{ float z;  float zDot; };
    union{ float v;  float vDot; };
    union{ float u;  float uDot; };
    union{ float o;  float oDot; };
}L6DStatef;

L6DStatef step_state( const L6DStatef state, const L6DStatef veloc, float timeStep ){
    L6DStatef rtnState = {
        state.x + veloc.xDot * timeStep,
        state.y + veloc.yDot * timeStep,
        state.z + veloc.zDot * timeStep,
        state.v + veloc.vDot * timeStep,
        state.u + veloc.uDot * timeStep,
        state.o + veloc.oDot * timeStep
    };
    return rtnState;
}

L6DStatef calc_rates_of_change( const L6DStatef s, float sigma, float r, float b ){
    // Get the state rate of change for a 6D Lorenz Attractor, given the present state and the system params
    /* Felicio, Carolini C., and Paulo C. Rech. "On the dynamics of five-and six-dimensional Lorenz models." 
       Journal of Physics Communications 2, no. 2 (2018): 025028.*/
    float b2p1 = 1.0f + 2.0f*b;
    L6DStatef rtnState = { // Section 2.2, Page 8
        sigma*(s.y - s.x),
        r*s.x - s.y - s.x*s.z + s.z*s.v - 2.0f*s.v*s.o,
        s.x*s.y - b*s.z - s.x*s.u -s.y*s.v,
        -1.0f*b2p1*sigma*s.v + sigma*s.u/(b2p1) ,
        s.x*s.z - 2.0f*s.x*s.o + r*s.v - b2p1*s.u,
        2.0f*s.x*s.u + 2.0f*s.y*s.v -4.0f*b*s.o
    };
    return rtnState;
}
