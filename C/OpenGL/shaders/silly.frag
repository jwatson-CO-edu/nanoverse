// Silly fragment shader
// Modified from source by: Willem A. (Vlakkies) Schreüder, https://www.prinmath.com/
#version 120

uniform float time;
varying vec4 color;
varying vec2 tex;
uniform sampler2D img;

void main()
{
   //  Reverse color every three seconds
   vec4 col = (fract(time/6.0)>0.5) ? color : 1.0-color;
   //  Set color
   gl_FragColor = col*texture2D(img,tex);
}
