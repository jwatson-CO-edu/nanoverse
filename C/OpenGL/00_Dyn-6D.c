////////// INIT ////////////////////////////////////////////////////////////////////////////////////

#include "toolbox.h"

////////// PROGRAM STRUCTS /////////////////////////////////////////////////////////////////////////

///// 6D Lorenz Attractor Dynamics ////////////////////////////////////////

typedef struct{
    // State -or- Rate of Change
    union{ float x;  float xDot; };
    union{ float y;  float yDot; };
    union{ float z;  float zDot; };
    union{ float v;  float vDot; };
    union{ float u;  float uDot; };
    union{ float o;  float oDot; };
}L6DStatef;


L6DStatef step_state( const L6DStatef state, const L6DStatef veloc, float timeStep ){
    // Rectangular integration of state
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
        s.x*s.y - b*s.z - s.x*s.u - s.y*s.v,
        -1.0f*b2p1*sigma*s.v + sigma*s.u/(b2p1) ,
        s.x*s.z - 2.0f*s.x*s.o + r*s.v - b2p1*s.u,
        2.0f*s.x*s.u + 2.0f*s.y*s.v - 4.0f*b*s.o
    };
    return rtnState;
}

vec4f get_state_half( const L6DStatef s, ubyte half ){
    // Return either the first or second half of `s` as a scaled 3D vector
    vec4f rtnHalf = make_0_vec4f();
    if( half ){
        rtnHalf.x = s.v;
        rtnHalf.y = s.u;
        rtnHalf.z = s.o;
    }else{
        rtnHalf.x = s.x;
        rtnHalf.y = s.y;
        rtnHalf.z = s.z;
    }
    return rtnHalf;
}



///// 6D Lorenz Attractor /////////////////////////////////////////////////

typedef struct{
    // 6D Lorenz Attractor, See above ref
    float     sigma;
    float     r;
    float     b;
    L6DStatef state;
    float     tsSec;
    uint /**/ Nstat;
    uint /**/ bgnDx;
    uint /**/ endDx;
    vec4f*    edge0;
    vec4f*    edge1;
    vec4f     color;
    float     h1scl;
}Attractor6D;


Attractor6D* make_6D_attractor( float sigma_, float r_, float b_, L6DStatef initState, 
                                float tsSec_, uint Nstates, vec4f color_, float h1scl_ ){
    // Setup the attractor with simulation params and history space
    Attractor6D* rtnStruct = (Attractor6D*) malloc( sizeof( Attractor6D ) );
    // Set Simulation Params //
    rtnStruct->sigma = sigma_;
    rtnStruct->r     = r_;
    rtnStruct->b     = b_;
    rtnStruct->state = initState;
    rtnStruct->tsSec = tsSec_;
    // Set Bookkeeping //
    rtnStruct->Nstat = Nstates;
    rtnStruct->bgnDx = 0;
    rtnStruct->endDx = 1;
    // Alloc Mem //
    rtnStruct->edge0 = (vec4f*) malloc( Nstates * sizeof( vec4f ) );
    rtnStruct->edge1 = (vec4f*) malloc( Nstates * sizeof( vec4f ) );
    // Set Display Params //
    rtnStruct->color = color_;
    rtnStruct->h1scl = h1scl_;
    // Set Init State Display //
    rtnStruct->edge0[0] = get_state_half( initState, 0 );
    rtnStruct->edge1[0] = get_state_half( initState, 1 );
    // Return //
    return rtnStruct;
}


void delete_6D_attractor( Attractor6D* attractor ){
    // Free attractor memory
    free( attractor->edge0 );
    free( attractor->edge1 );
    free( attractor );
}


void step_6D_attractor( Attractor6D* attractor ){
    // Run one timestep and append to state history
    vec4f h0, h1;
    uint end = attractor->endDx;
    attractor->state = step_state( 
        attractor->state, 
        calc_rates_of_change( attractor->state, attractor->sigma, attractor->r, attractor->b ), 
        attractor->tsSec
    );
    h0 = get_state_half( attractor->state, 0 );
    h1 = get_state_half( attractor->state, 1 );
    attractor->edge0[ end ] = h0;
    attractor->edge1[ end ] = add_vec4f( h0, scale_vec4f( h1, attractor->h1scl ) );
    attractor->endDx = (end+1)%(attractor->Nstat);
    if( attractor->bgnDx == end ){  attractor->bgnDx = attractor->endDx;  }
}


void draw_6D_attractor( Attractor6D* attractor ){
    // Render the state history as a triangle strip with receding
    uint  Ntot, ndx, ldx; 
    uint  bgn = attractor->bgnDx;
    uint  len = attractor->Nstat;
    float alpha;
    if( bgn == attractor->endDx ){  Ntot = attractor->Nstat;  }else{  Ntot = attractor->endDx;  }
    for( uint i = 0; i < Ntot; ++i ){
        ndx   = (bgn + i) % len;
        alpha = (Ntot - i)*1.0f/Ntot;
        glColor4f( attractor->color.r , attractor->color.g , attractor->color.b, alpha );

        // FIXME, START HERE: CALC THE NORMAL FOR THIS PART OF THE RIBBON
        
        glVtx4f( attractor->edge0[ ndx ] );  glVtx4f( attractor->edge1[ ndx ] );
    }
}