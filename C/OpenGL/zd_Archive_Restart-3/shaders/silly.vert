// Silly vertex shader
// Modified from source by: Willem A. (Vlakkies) Schreüder, https://www.prinmath.com/
#version 120

uniform float time;
uniform float scale;
uniform uint  bgn;
varying float tLst;
varying vec4  color;
varying vec2  tex;



////////// HELPER FUNCTIONS ////////////////////////////////////////////////////////////////////////
float accum;

uint hash( uint x ) {
    // A single iteration of Bob Jenkins' OAT algorithm
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

vec2 rand_unit_vec2( void ){
   // Return a random unit 2D vector
   vec2 dir = { randf(), randf() };
   return normalize( dir );
}



////////// MAIN ////////////////////////////////////////////////////////////////////////////////////


void main(){
   //  Use color unchanged
   color = gl_Color;
   //  Scroll texture coordinates
   tex  = gl_MultiTexCoord0.xy + vec2( 0.5*time,0.0);
   //  Set vertex coordinates
   vec4 pos = gl_Vertex;
   //  Scale in XY plane
   pos.xy *= 1.0+0.3*cos(4.0*time);
   //  Transform
   gl_Position = gl_ModelViewProjectionMatrix * pos;
}
