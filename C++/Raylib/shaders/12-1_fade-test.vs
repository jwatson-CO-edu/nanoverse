#version 330

/* 
12-1_fade-test.vs
James Watson, 2023-05
First attempt at a vertex shader
*/

// Input vertex attributes
in vec3 vertexPosition;
in vec4 vertexColor;

// Output vertex attributes (to fragment shader)
out vec4 fragColor;
out vec3 fragPosition;

float alphaMax  =  1.0f;
float alphaMin  =  0.1f
float alphaSpan = alphaMax - alphaMin;
float Zmin      = -5.0f;
float Zmax      =  5.0f;
float zSpan     = Zmax - Zmin;
float zVal, aVal;

void main(){

    zVal = fragPosition[2];
    aVal = (zVal - Zmin)*zSpan * alphaSpan + alphaMin;
    fragColor[0] = vertexColor[0];
    fragColor[1] = vertexColor[1];
    fragColor[2] = vertexColor[2];
    fragColor[3] = aVal;
}