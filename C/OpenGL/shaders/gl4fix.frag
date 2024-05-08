// Fixed color GL4
#version 400 core

//  Input Color
uniform vec4  Kd;
//  Fragment color
layout (location=0) out vec4 FragColor;

void main()
{
   FragColor = Kd;
}
