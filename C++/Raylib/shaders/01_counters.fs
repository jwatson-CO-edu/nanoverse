#version 430

// https://bedroomcoders.co.uk/advanced-shader-debugging-with-raylib/

in vec2 fragTexCoord;           
in vec4 fragColor;       

out vec4 finalColor;            

uniform sampler2D texture0;     
uniform vec4 colDiffuse;

layout( binding = 0, offset = 0 ) uniform atomic_uint debug_frags;
layout( binding = 0, offset = 4 ) uniform atomic_uint debug_verts;

layout ( std430, binding = 0 ) buffer debug_data {
    vec4 debug_values[1024];
    uint debug_message_id[1024];
    uint debug_id[1024];
    uint debug_index;
};

uint fragCounter;

void UpdateFragCounter(){
    fragCounter = atomicCounterIncrement( debug_frags );
}

void LogValue( uint messageID, vec4 value ){
    if( debug_index < 1024 ){
        uint di = atomicAdd(debug_index, 1);
        debug_values[di]     = value;
        debug_message_id[di] = messageID;
        debug_id[di]         = fragCounter;
    }
}

/* Were taking colour from three different sources, the vertices (not that commonly used to be honest) 
   The diffuse colour (an overall colour when seen in bright white light) and finally a colour value 
   looked up from the texture coordinates that usually accompany each vertex.  */

void main(){                               
    vec4 texelColor = texture( texture0, fragTexCoord );
    finalColor      = texelColor * colDiffuse * fragColor;  
    
    UpdateFragCounter();
  
    if( fragCounter / 10000.0 == fragCounter / 10000 ){

        LogValue( 3, texelColor );
            
        if( fragCounter != 10000 ){
            LogValue( 4, finalColor );
        }          

    } 
 
}