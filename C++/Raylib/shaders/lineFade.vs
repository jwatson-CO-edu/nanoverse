#version 330

// Input vertex attributes
in vec3 vertexPosition;
in vec2 vertexTexCoord;
in vec3 vertexNormal;
in vec4 vertexColor;

// Input uniform values
// uniform float u_linewidth;
uniform mat4  mvp;               // conmbined Model View Projection matrix
uniform mat4  matModel;          // Model matrix
uniform mat4  matView;           // View matrix
uniform mat4  matProjection;     // Projection matrix

// Output vertex attributes (to fragment shader)
out vec2 fragTexCoord;
out vec4 fragColor;
out float u_linewidth;

// NOTE: Add here your custom variables

void main()
{
    // Send vertex attributes to fragment shader
    fragTexCoord = vertexTexCoord;
    fragColor = vertexColor;

    // Calculate final vertex position
    gl_Position = mvp*vec4(vertexPosition, 1.0);
    fragColor.a   = 1.0/gl_Position.z;
}