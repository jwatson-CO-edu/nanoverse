// Per Pixel Lighting GL4
#version 400 core

//  Transformation matrices
uniform mat4 ModelViewMatrix;
uniform mat4 ViewMatrix;
uniform mat4 ProjectionMatrix;
uniform mat3 NormalMatrix;
//  Light properties
uniform vec4 Position;

//  Vertex attributes
in vec4 Vertex;
in vec3 Normal;
in vec4 Color;
in vec2 Texture;

//  Output to next shader
out vec3 View;
out vec3 Light;
out vec3 Norm;
out vec2 TexCoord;
out vec4 Kd;

void main()
{
   //  Vertex location in modelview coordinates
   vec4 P = ModelViewMatrix * Vertex;
   //  Light direction
   Light = vec3(ViewMatrix * Position - P);
   //  Normal vector
   Norm = NormalMatrix * Normal;
   //  Eye position
   View  = -P.xyz;
   //  Set diffuse to Color
   Kd = Color;
   //  Texture
   TexCoord = Texture;
   //  Set transformed vertex location
   gl_Position =  ProjectionMatrix * ModelViewMatrix * Vertex;
}
