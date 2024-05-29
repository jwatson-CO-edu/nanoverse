/*
sparkleTrail.geom
Randomly populate points on the surfaces of a polyhedron, but draw the poly normally, points --to-> triangle
James Watson, 2024-05

Resources:
* https://learnopengl.com/Advanced-OpenGL/Geometry-Shader
* https://github.com/jwatson-CO-edu/nanoverse/blob/main/C/OpenGL/shaders/prtclDyn.comp#L12
*/

////////// INIT ////////////////////////////////////////////////////////////////////////////////////

#version 450

layout( points ) in;
layout( triangle_strip, max_vertices = 3 ) out;

layout( binding = 4 ) buffer posbuf { vec4  posnArr[]; };
layout( binding = 5 ) buffer clrbuf { vec4  colrArr[]; };

uniform float time;


////////// HELPER FUNCTIONS ////////////////////////////////////////////////////////////////////////

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

////////// MAIN ////////////////////////////////////////////////////////////////////////////////////

void main(){    

    /// Sparkle ///

    /// Unmodified Triangle ///
    gl_Position = gl_in[0].gl_Position;  EmitVertex();
    gl_Position = gl_in[1].gl_Position;  EmitVertex();
    gl_Position = gl_in[2].gl_Position;  EmitVertex();
    EndPrimitive();
}  