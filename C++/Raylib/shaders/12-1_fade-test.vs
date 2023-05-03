#version 330

/* 
12-1_fade-test.vs
James Watson, 2023-05
First attempt at a vertex shader
*/

// Input vertex attributes
in vec3 vertexPosition;
in vec2 vertexTexCoord;
in vec3 vertexNormal;
in vec4 vertexColor;

// Input uniform values
uniform mat4 mvp;
uniform mat4 matModel;

// Output vertex attributes (to fragment shader)
out vec2 fragTexCoord;
out vec4 fragColor;
out vec3 fragPosition;
out vec3 fragNormal;
out vec3 modNorm;

const float alphaMax =  1.0;
const float alphaMin =  0.1;
const float Zmin     = -5.0;
const float Zmax     =  5.0;

void main(){
    // Send vertex attributes to fragment shader
    fragTexCoord = vertexTexCoord;
    fragPosition = vec3(matModel*vec4(vertexPosition, 1.0f));
    fragColor    = vertexColor;
    fragColor.a  = 1.0*(Zmax - fragPosition.z)*(Zmax - Zmin) * (alphaMax - alphaMin) + alphaMin;
    
    mat3 normalMatrix = transpose(inverse(mat3(matModel)));
    fragNormal = normalize(normalMatrix*vertexNormal);
    modNorm    = vertexNormal;

    // Calculate final vertex position
    gl_Position = mvp*vec4(vertexPosition, 1.0);
}