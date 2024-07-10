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

// Geometry Shader I/O
layout( points ) in;
layout( triangle_strip, max_vertices = 3 ) out;

// Particle Buffer
layout( binding = 0 ) uniform samplerBuffer buf;
uniform int /*---------------------------*/ Nprt;

// Particle Settings
uniform int   active; // -- Particle system flag
uniform float shdrTime; //- Used for randomization
uniform float clrThresh; // Opacity that denotes a live particle
uniform vec4  brightClr; // Particle base color



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



////////// MAIN ////////////////////////////////////////////////////////////////////////////////////

/// Update Params ///
const float _MTD /*--*/ = 5.0;
const float _DEATH_PROB = 1.0 / _MTD;
const float _DECAY_RATE = 0.99;
const float _DROP_RATE  = 0.01;

void main(){    

    /// Init ///

    accum = shdrTime;

    ///// Particle System /////////////////////////////////////////////////

    /// Sparkle Gen & Update ///
    vec4  posn_i;
    vec4  colr_i;
    float w0, w1, w2, tot;
    int  live_i;

    // For each possible particle
    for( int i = 0; i < Nprt; ++i ){

        // Fetch particle
        posn_i = texelFetch( buf, i      );
        colr_i = texelFetch( buf, i+Nprt );

        // Determine if the particle is active
        if( (posn_i.z > 0.0) || (colr_i.a > clrThresh) ){
            
            live_i = 1;

            // Drop and Decay the particle
            colr_i   *= _DECAY_RATE
            posn_i.z += _DROP_RATE; // FIXME: CHANGE FROM RISE TO DROP AFTER SHADER TEST
            
            if( randf() < _DEATH_PROB ){
                posn_i.z = 0.0;
                colr_i.a = 0.0;
                live_i   = 0;
            }
        }

        // If the particle is dead, roll to resurrect
        if( (!live_i) && (randf() > _DEATH_PROB) ){
            w0  = randf();
            w1  = randf();
            w2  = randf();
            tot = w0 + w1 + w2;
            w0  /= tot;
            w1  /= tot;
            w2  /= tot;
            posn_i = gl_in[0].gl_Position*w0 + gl_in[1].gl_Position*w1 + gl_in[1].gl_Position*w1;
            colr_i = brightClr;
        }

        // Write transformed particle back to the buffer texture

    }


    
    ///// Geometry Pass-Thru //////////////////////////////////////////////

    /// Unmodified Triangle ///
    gl_Position = gl_in[0].gl_Position;  EmitVertex(); // Produces a new, independent point
    gl_Position = gl_in[1].gl_Position;  EmitVertex();
    gl_Position = gl_in[2].gl_Position;  EmitVertex();
    EndPrimitive(); // Produce a new vertex at the output of the geometry shader
    // (When the shader exits, the current primitive is ended implicitly.)
}  