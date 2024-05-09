// Per Pixel Lighting GL4
#version 400 core

//  Material properties (assume Ka=Kd)
in      vec4  Kd;  // Diffuse
uniform vec4  Ks;  // Specular
uniform vec4  Ke;  // Emissions
uniform float Shinyness;

//  Light vectors
in vec3 View;
in vec3 Light;
in vec3 Norm;

//  Light colors
uniform vec4 Ambient;
uniform vec4 Diffuse;
uniform vec4 Specular;

//  Texture coordinates
in vec2 TexCoord;
//  Texture
uniform sampler2D tex;

//  Fragment color
out vec4 FragColor;

void main()
{
   //  N is the object normal
   vec3 N = normalize(Norm);
   //  L is the light vector
   vec3 L = normalize(Light);
   //  R is the reflected light vector R = 2(L.N)N - L
   vec3 R = reflect(-L,N);
   //  V is the view vector (eye vector)
   vec3 V = normalize(View);

   //  Diffuse light is cosine of light and normal vectors
   float Id = max(dot(L,N) , 0.0);
   //  Specular is cosine of reflected and view vectors
   float Is = (Id>0.0) ? pow(max(dot(R,V),0.0) , Shinyness) : 0.0;

   //  Sum color types
   vec4 color = Ke + Kd*Ambient + Id*Kd*Diffuse + Is*Ks*Specular;

   //  Apply texture
   FragColor = color * texture(tex,TexCoord);
}
