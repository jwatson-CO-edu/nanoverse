// https://bedroomcoders.co.uk/advanced-shader-debugging-with-raylib/

in vec2 fragTexCoord;           
in vec4 fragColor;       

out vec4 finalColor;            

uniform sampler2D texture0;     
uniform vec4 colDiffuse;

/* Were taking colour from three different sources, the vertices (not that commonly used to be honest) 
   The diffuse colour (an overall colour when seen in bright white light) and finally a colour value 
   looked up from the texture coordinates that usually accompany each vertex.  */

void main(){                               
    vec4 texelColor = texture( texture0, fragTexCoord );
    finalColor      = texelColor*colDiffuse*fragColor;  
}