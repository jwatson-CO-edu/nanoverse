// Font texture GL4
#version 400 core

//  Convert point to quad
layout(points) in;
layout(triangle_strip,max_vertices=4) out;

//  Transformation matrices
uniform mat4 ProjectionMatrix;
uniform mat4 ModelViewMatrix;

//  Vertex attributes (input)
in vec2 gTexture[1];
//  Texture out
out vec2 fTexture;

//  Character Dimensions in world coordinates
uniform float dX;
uniform float dY;
//  Character Dimensions in texture coordinates
uniform float sX;
uniform float sY;

void billboard(float x,float y,float s,float t)
{
   //  Set texture coordinates
   fTexture = gTexture[0]+vec2(s,t);

   //  Determine position
   vec2 delta = vec2(x,y);
   vec4 p = vec4(0,0,0,1);
   p.x += dot(delta,ModelViewMatrix[0].xy);
   p.y += dot(delta,ModelViewMatrix[1].xy);
   p.z += dot(delta,ModelViewMatrix[2].xy);
   gl_Position = ProjectionMatrix * ModelViewMatrix * p;
   //  Emit new vertex
   EmitVertex();
}

//  Draw 4 vertexes
void main()
{
   billboard( 0, 0 , 0, 0);
   billboard(dX, 0 ,sX, 0);
   billboard( 0,dY , 0,sY);
   billboard(dX,dY ,sX,sY);
   EndPrimitive();
}
