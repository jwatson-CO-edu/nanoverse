#version 140 // https://bugs.freedesktop.org/show_bug.cgi?id=70261

precision mediump float;

// Input vertex attributes (from vertex shader)
varying vec2 fragTexCoord;
varying vec4 fragColor;

// Input uniform values
uniform sampler2D texture0;
uniform vec4 colDiffuse;

// NOTE: Add here your custom variables

const vec2 size = vec2(600, 600);   // render size
const float samples = 4.25; // 4.25; // 4.5; // 7.5; // 10.0; // 5.0; // pixels per axis; higher = bigger glow, worse performance
const float quality = 3.5; //2.5;  // lower = smaller glow, better quality

void main(){
    vec4 sum = vec4(0);
    vec2 sizeFactor = vec2(1)/size*quality;

    // Texel color fetching from texture sampler
    vec4 source = texture2D(texture0, fragTexCoord);

    const int range = 2;            // should be = (samples - 1)/2;

    for (int x = -range; x <= range; x++)
    {
        for (int y = -range; y <= range; y++)
        {
            sum += texture2D(texture0, fragTexCoord + vec2(x, y)*sizeFactor);
        }
    }

    // Calculate final fragment color
    gl_FragColor = ((sum/(samples*samples)) + source)*colDiffuse;
}