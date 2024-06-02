// Source: https://shader-tutorial.dev/basics/vertex-shader/


////////// INIT ////////////////////////////////////////////////////////////////////////////////////

#version 450

in vec4 vertexPosition;

uniform mat4 modelMatrix;
uniform mat4 viewMatrix;
uniform mat4 projectionMatrix;



////////// MAIN ////////////////////////////////////////////////////////////////////////////////////

void main(){
    gl_Position = projectionMatrix * viewMatrix * modelMatrix * vertexPosition;
}