// Font texture GL4
#version 400 core

//  Texture attributes
in  vec2 Texture;
out vec2 gTexture;

//  The geometry shader does all the work
void main()
{
   gTexture = Texture;
}
