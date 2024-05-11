// Fixed color GL4
#version 400 core

//  Transformation matrices
uniform mat4 ModelViewMatrix;
uniform mat4 ProjectionMatrix;

//  Vertex attributes (input)
in vec4 Vertex;

void main()
{
   //  Set transformed vertex location
   gl_Position =  ProjectionMatrix * ModelViewMatrix * Vertex;
}
