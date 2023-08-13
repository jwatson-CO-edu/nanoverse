#version 430

// https://bedroomcoders.co.uk/advanced-shader-debugging-with-raylib/

in vec3 vertexPosition;            
in vec2 vertexTexCoord;            
in vec4 vertexColor;
              
out vec2 fragTexCoord;             
out vec4 fragColor;                

uniform mat4 mvp;          

uint vertCounter;

layout( binding = 0, offset = 0 ) uniform atomic_uint debug_frags;
layout( binding = 0, offset = 4 ) uniform atomic_uint debug_verts;

layout ( std430, binding = 0 ) buffer debug_data{
    vec4 debug_values[1024];
    uint debug_message_id[1024];
    uint debug_id[1024];
    uint debug_index;
};

void UpdateVertCounter(){
    vertCounter = atomicCounterIncrement( debug_verts );
}

void LogValue( uint messageID, vec4 value ){
    if( debug_index < 1024 ){
        uint di = atomicAdd( debug_index, 1 );
        debug_values[di]     = value;
        debug_message_id[di] = messageID;
        debug_id[di]         = vertCounter;
    }
}
   

/*  Each time a shader is executed a counter is incremented. 
    This keeps track of just which vertex or fragment is being processed.  
    
    Since OpenGL 4.3 there has been a special type of buffer called Shader Storage Buffer Objects (SSBO), 
    these are ideal for logging data and probably many more uses as not only can GLSL read and write to SSBO’s 
    but so can your application. They can also be much larger than other types of shader buffers, 
    the spec guarantees some 128MB of space and aparently many implementations will let you use 
    as much GPU memory as is currently available.  
    
    There are some wrinkles with implementing this, so the shader needs a few helper functions, 
    which are embedded in your target shader just before you compile it. 
    
    There are two slightly different versions of LogValue depending if the fragment or vertex shader is being executed. 
    Similarly UpdateFragCounter() is only available in the fragment shader. For convenience updating a counter 
    also updates a copy the counter, this is intended to allow you to filter what you log, and writing to this variable 
    (`fragCounter` or `vertCounter`) will do nothing but mess things up!*/

void main(){     
            
    fragTexCoord = vertexTexCoord; 
    fragColor    = vertexColor;       
    gl_Position  = mvp*vec4(vertexPosition, 1.0); 

    UpdateVertCounter();

    if( (vertCounter >= 4) && (vertCounter <= 8) ){
        LogValue( 2, gl_Position );
        LogValue( 1, vec4( vertexPosition, 0.0 ) );
    }

    if( vertCounter == 16 )  LogValue( 2, gl_Position );
}