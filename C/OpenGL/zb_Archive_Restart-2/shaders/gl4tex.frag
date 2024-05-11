// Font texture GL4
#version 400 core

//  Input Color
uniform vec4  Kd;

//  Texture coordinates
in vec2 fTexture;
//  Font texture
uniform sampler2D font;

//  Fragment color
layout (location=0) out vec4 FragColor;

void main()
{
   vec4 color =  Kd*texture(font,fTexture);
   //  Discard black
   if (color.r+color.g+color.b<0.1)
     discard;
   //  Show font color
   else
     FragColor = color;
}
