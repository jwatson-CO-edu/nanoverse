// Source: https://shader-tutorial.dev/basics/vertex-shader/


////////// INIT ////////////////////////////////////////////////////////////////////////////////////

#version 330

// in  vec4 vertexPosition;
// in  vec4 color;
layout (location = 0) in vec4 vertexPosition;
layout (location = 1) in vec4 color;
out vec4 vertexColor;

uniform mat4 modelMatrix;
uniform mat4 viewMatrix;
uniform mat4 projectionMatrix;



////////// MAIN ////////////////////////////////////////////////////////////////////////////////////

void main(){
    gl_Position = projectionMatrix * viewMatrix * modelMatrix * vertexPosition;
    // gl_Position = vertexPosition;
    vertexColor = color;
}