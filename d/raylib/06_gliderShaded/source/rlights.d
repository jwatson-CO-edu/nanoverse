module rlights;
// FIXME: TEST PORT
// https://github.com/raysan5/raylib/blob/387c06000618ef0aa3b15c5e46d1c525ba194c50/examples/shaders/rlights.h

/**********************************************************************************************
*
*   rlights.d - Some useful functions to deal with lights data
*
*   LICENSE: zlib/libpng
*
*   D port by James Watson, 2022-11
*
*   Original Copyright (c) 2017-2022 Victor Fisac (@victorfisac) and Ramon Santamaria (@raysan5)
*
*   This software is provided "as-is", without any express or implied warranty. In no event
*   will the authors be held liable for any damages arising from the use of this software.
*
*   Permission is granted to anyone to use this software for any purpose, including commercial
*   applications, and to alter it and redistribute it freely, subject to the following restrictions:
*
*     1. The origin of this software must not be misrepresented; you must not claim that you
*     wrote the original software. If you use this software in a product, an acknowledgment
*     in the product documentation would be appreciated but is not required.
*
*     2. Altered source versions must be plainly marked as such, and must not be misrepresented
*     as being the original software.
*
*     3. This notice may not be removed or altered from any source distribution.
*
**********************************************************************************************/

import raylib;

/***********************************************************************************
*
*   RLIGHTS ENUMS & STRUCTS
*
************************************************************************************/

enum LIGHT_CONST{
    MAX_LIGHTS = 4 
}

enum LightType{
    LIGHT_DIRECTIONAL = 0,
    LIGHT_POINT
}

struct Light{
    // Basic attributes
    int     type;
    bool    enabled;
    Vector3 position;
    Vector3 target;
    Color   color;
    float   attenuation;
    // Shader locations
    int enabledLoc;
    int typeLoc;
    int positionLoc;
    int targetLoc;
    int colorLoc;
    int attenuationLoc;
}

/***********************************************************************************
*
*   RLIGHTS IMPLEMENTATION
*
************************************************************************************/

//----------------------------------------------------------------------------------
// Global Variables Definition
//----------------------------------------------------------------------------------
static int lightsCount = 0;    // Current amount of created lights

//----------------------------------------------------------------------------------
// Module Functions Definition
//----------------------------------------------------------------------------------


void UpdateLightValues( Shader shader, Light light ){
    // Send light properties to shader
    // NOTE: Light shader locations should be available 

    // Send to shader light enabled state and type
    SetShaderValue( shader, light.enabledLoc, &light.enabled, ShaderUniformDataType.SHADER_UNIFORM_INT );
    SetShaderValue( shader, light.typeLoc   , &light.type   , ShaderUniformDataType.SHADER_UNIFORM_INT );

    // Send to shader light position values
    float[3] position = [ light.position.x, light.position.y, light.position.z ];
    SetShaderValue( shader, light.positionLoc, position.ptr, ShaderUniformDataType.SHADER_UNIFORM_VEC3 );

    // Send to shader light target position values
    float[3] target = [light.target.x, light.target.y, light.target.z];
    SetShaderValue( shader, light.targetLoc, target.ptr, ShaderUniformDataType.SHADER_UNIFORM_VEC3 );

    // Send to shader light color values
    float[4] color = [ light.color.r/255.0f, light.color.g/255.0f, 
                       light.color.b/255.0f, light.color.a/255.0f ];
    SetShaderValue(shader, light.colorLoc, color.ptr, ShaderUniformDataType.SHADER_UNIFORM_VEC4);
}


Light CreateLight( int type, Vector3 position, Vector3 target, Color color, Shader shader ){
    // Create a light and get shader locations

    Light light;

    if( lightsCount < LIGHT_CONST.MAX_LIGHTS ){
        light.enabled  = true;
        light.type     = type;
        light.position = position;
        light.target   = target;
        light.color    = color;

        // NOTE: Lighting shader naming must be the provided ones
        light.enabledLoc  = GetShaderLocation(shader, TextFormat("lights[%i].enabled", lightsCount));
        light.typeLoc     = GetShaderLocation(shader, TextFormat("lights[%i].type", lightsCount));
        light.positionLoc = GetShaderLocation(shader, TextFormat("lights[%i].position", lightsCount));
        light.targetLoc   = GetShaderLocation(shader, TextFormat("lights[%i].target", lightsCount));
        light.colorLoc    = GetShaderLocation(shader, TextFormat("lights[%i].color", lightsCount));

        UpdateLightValues(shader, light);
        
        lightsCount++;
    }

    return light;
}