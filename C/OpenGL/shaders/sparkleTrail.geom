/*
sparkleTrail.geom
Randomly populate points on the surfaces of a polyhedron, but draw the poly normally, points --to-> triangle
James Watson, 2024-05

Resources:
* https://learnopengl.com/Advanced-OpenGL/Geometry-Shader
* https://github.com/jwatson-CO-edu/nanoverse/blob/main/C/OpenGL/shaders/prtclDyn.comp#L12
*/

////////// INIT ////////////////////////////////////////////////////////////////////////////////////

#version 330 // "#version 450" changes many things about shaders!  Book uses 330!

layout( points ) in;
layout( triangle_strip, max_vertices = 3 ) out;

// in vData
// {
//     vec3 normal;
//     vec4 vertexColor;
// }vertices[];

// out fData
// {
//     vec3 normal;
//     vec4 color;
// }frag;   


uniform int kill;
uniform float shdrTime;
uniform int   Nprt;
uniform float clrThresh;
uniform vec4  brightClr;



// layout( binding = 4 ) buffer posbuf { vec4  posnArr[]; };
// layout( binding = 5 ) buffer clrbuf { vec4  colrArr[]; };

////////// HELPER FUNCTIONS ////////////////////////////////////////////////////////////////////////
float accum;

uint hash( uint x ) {
    // a single iteration of Bob Jenkins' OAT algorithm
    // Author: Lee C, http://amindforeverprogramming.blogspot.com/2013/07/random-floats-in-glsl-330.html
    x += ( x << 10u );
    x ^= ( x >>  6u );
    x += ( x <<  3u );
    x ^= ( x >> 11u );
    x += ( x << 15u );
    return x;
}


float random( float f ){
    // Mutate the bits of `f`
    // Author: Lee C, http://amindforeverprogramming.blogspot.com/2013/07/random-floats-in-glsl-330.html
    const uint mantissaMask = 0x007FFFFFu;
    const uint one /*----*/ = 0x3F800000u;
    
    uint h = hash( floatBitsToUint( f ) );
    h &= mantissaMask;
    h |= one;
    
    float  r2 = uintBitsToFloat( h );
    return r2 - 1.0;
}


float randf( void ){
    // Accumulate seed value, then use it to gen `random` number
    accum += random( shdrTime );
    return random( accum );
}

// void reset_particles( void ){
//     // Set all particles to dead state
//     for( int i = 0; i < Nprt; ++i ){
//         colrArr[i].a = 0.0;
//     }
// }



////////// MAIN ////////////////////////////////////////////////////////////////////////////////////

/// Update Params ///
const float _MTD /*--*/ = 5.0;
const float _DEATH_PROB = 1.0 / _MTD;
const float _DECAY_RATE = 0.99;
const float _DROP_RATE  = 0.01;

void main(){    

    /// Init ///

    accum = shdrTime;

    /// Sparkle Gen & Update ///
    vec4  nuPosn;
    vec4  nuColr;
    float w0, w1, w2, tot;
    // for( int i = 0; i < Nprt; ++i ){

    //     // Drop and Decay the particle
    //     colrArr[i]   *= _DECAY_RATE
    //     posnArr[i].z -= _DROP_RATE;

    //     // If particle has died, then regen
    //     if( (posnArr[i].z < 0.0) || (colrArr[i].a < clrThresh) || (randf() < _DEATH_PROB) ){
    //         w0  = randf();
    //         w1  = randf();
    //         w2  = randf();
    //         tot = w0 + w1 + w2;
    //         w0  /= tot;
    //         w1  /= tot;
    //         w2  /= tot;
    //         posnArr[i] = gl_in[0].gl_Position*w0 + gl_in[1].gl_Position*w1 + gl_in[1].gl_Position*w1;
    //         colrArr[i] = brightClr;
    //     }
    // }

    /// Unmodified Triangle ///
    gl_Position = gl_in[0].gl_Position;  EmitVertex();
    gl_Position = gl_in[1].gl_Position;  EmitVertex();
    gl_Position = gl_in[2].gl_Position;  EmitVertex();
    EndPrimitive();
}  