// https://bedroomcoders.co.uk/advanced-shader-debugging-with-raylib/

in vec2 vertexTexCoord;            
in vec4 vertexColor;

out vec2 fragTexCoord;             
out vec4 fragColor;                

uniform mat4 mvp;      

/* The vertex shader is very simple: all its really doing is taking the colour and vertex information 
   from a vertex buffer and translating the vertex coordinates into the view space 
   (mvp is an abbreviation for Model View Perspective, its a combination of three different matrices – 
   you could send the three separately to the shader and multiply, 
   but then that’s an expensive operation per vertex.)  */

void main(){             
    fragTexCoord = vertexTexCoord; 
    fragColor    = vertexColor;       
    gl_Position  = mvp*vec4( vertexPosition, 1.0 ); 
}