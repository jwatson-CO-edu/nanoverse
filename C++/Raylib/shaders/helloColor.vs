#version 330

// layout (location = 0) in vec3 aPos;

// Input vertex attributes
in vec3 vertexPosition;
in vec2 vertexTexCoord;
in vec2 vertexTexCoord2;
in vec3 vertexNormal;
in vec3 vertexTangent;
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

void main(){
   fragPosition = vec3(matModel*vec4(vertexPosition, 1.0f));
   // Calculate final vertex position
   gl_Position = mvp*vec4(vertexPosition, 1.0);
}