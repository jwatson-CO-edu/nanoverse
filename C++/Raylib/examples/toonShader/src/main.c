/*
 * Copyright (c) 2019 Chris Camacho (codifies -  http://bedroomcoders.co.uk/)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 */

#include <stddef.h>

#include "raylib.h"
#include "raymath.h"

#define RLIGHTS_IMPLEMENTATION
#include "rlights.h"

extern Model LoadObj(const char* filename); 

Model model = { 0 };
Model edMod = { 0 };
Model tank = { 0 };

const Vector2 res = { 1280, 720 };

// positions and rotation for each snowman in the scene
const Vector3 xza[] = {
    {0,0,0},
    {-4,-2,-.5},
    {-3,3,-1},
    {-2,-3,.5},
    {2,-4,2},
    {2,2,1},
    {4.5,0.5,.2}
};

// set the models shader for each material
void setModelShader(Model* m, Shader* s)
{
    for (int i = 0; i < m->materialCount; i++) {
        m->materials[i].shader = *s;
    }    
}

// each snowman has a position on the ground plane with
// z component used for Y axis rotation.
void drawScene() {
    DrawModel(edMod, (Vector3){ -1, 0, 1 }, .5, WHITE);
    DrawModel(tank, (Vector3){ 2, 0, -1 }, 2, WHITE);
    for (int i=0; i<7; i++) {
        model.transform = MatrixRotateY(xza[i].z);
        DrawModel(model, (Vector3){xza[i].x,0,xza[i].y}, 0.5, WHITE);
    }
}

int main(void)
{
    // Initialization
    //--------------------------------------------------------------------------------------
    SetConfigFlags(FLAG_MSAA_4X_HINT | FLAG_VSYNC_HINT);
    InitWindow(res.x, res.y, "raylib - test");

    // Define the camera to look into our 3d world
    Camera camera = { 0 };
    camera.position = (Vector3){ 0.0f, 4.0f, 8.0f };
    camera.target = (Vector3){ 0.0f, 1.0f, 0.0f };
    camera.up = (Vector3){ 0.0f, 1.0f, 0.0f };
    camera.fovy = 45.0f;

    model = LoadObj("data/snowman.obj");
    edMod = LoadObj("data/ed.obj");
    edMod.transform = MatrixRotateY(-135*DEG2RAD);
    tank = LoadModel("data/tank.gltf");
    
    // normal shader
    Shader normShader = LoadShader("data/norm.vs", "data/norm.fs");
    normShader.locs[SHADER_LOC_MATRIX_MODEL] = GetShaderLocation(normShader, "matModel");
    
    // outline shader
    Shader outline = LoadShader(NULL, "data/outline.fs");

    // lighting shader
    Shader shader = LoadShader("data/toon.vs", "data/toon.fs");
    shader.locs[SHADER_LOC_MATRIX_MODEL] = GetShaderLocation(shader, "matModel");

    // make a light (max 4 but we're only using 1)
    Light light = CreateLight(LIGHT_POINT, (Vector3){ 2,4,4 }, Vector3Zero(), WHITE, shader);
    
    // camera orbit angle
    float ca = 0;
    
    RenderTexture2D target = LoadRenderTexture(res.x, res.y);
    //--------------------------------------------------------------------------------------

    // Main game loop
    while (!WindowShouldClose())    // Detect window close button or ESC key
    {
        // Update
        //----------------------------------------------------------------------------------

        // manually orbiting the camera but slower than the
        // raylib camera does.
        ca += 0.025;
        camera.position.x = cosf(-ca/11)*8.0;
        camera.position.z = sinf(-ca/11)*8.0;

        UpdateCamera(&camera);

        // you can move the light around if you want
        light.position.x = 12.0 * cosf(ca);
        light.position.z = 12.0 * sinf(ca);

        // update the light shader with the camera view position
        SetShaderValue(shader, shader.locs[SHADER_LOC_VECTOR_VIEW], &camera.position.x, SHADER_UNIFORM_VEC3);
        SetShaderValue(normShader, shader.locs[SHADER_LOC_VECTOR_VIEW], &camera.position.x, SHADER_UNIFORM_VEC3);
        UpdateLightValues(shader, light);
        UpdateLightValues(normShader, light);

        // Draw
        //----------------------------------------------------------------------------------
        BeginDrawing();

            // render first to the normals texture for outlining
            // to a texture
            BeginTextureMode(target); 
                ClearBackground((Color){255,255,255,255});
                BeginMode3D(camera);
                    setModelShader(&model, &normShader);
                    setModelShader(&edMod, &normShader);
                    setModelShader(&tank, &normShader);
                    drawScene();
                EndMode3D();
            EndTextureMode();
            
            // draw the scene but with banded light effect
            ClearBackground((Color){32,64,255,255});
            BeginMode3D(camera);   
                setModelShader(&model, &shader);
                setModelShader(&edMod, &shader);
                setModelShader(&tank, &shader);
                drawScene();
                DrawGrid(10, 1.0f);        // Draw a grid
            EndMode3D();
            
            // show the modified normals texture
            DrawTexturePro(target.texture,
                    (Rectangle){ 0, 0, target.texture.width, -target.texture.height },
                    (Rectangle){ 0, 0, target.texture.width/4.0, target.texture.height/4.0 },
                    (Vector2){0,0}, 0, WHITE);    
                    
            // outline shader uses the normal texture to overlay outlines
            BeginShaderMode(outline);
                DrawTexturePro(target.texture,
                        (Rectangle){ 0, 0, target.texture.width, -target.texture.height },
                        (Rectangle){ 0, 0, target.texture.width, target.texture.height },
                        (Vector2){0,0}, 0, WHITE);
            EndShaderMode();

            DrawFPS(10, 10);

        EndDrawing();
        //----------------------------------------------------------------------------------
    }

    // De-Initialization
    //--------------------------------------------------------------------------------------


    UnloadShader(shader);
    UnloadShader(outline);
    UnloadShader(normShader);
    UnloadModel(model);
    UnloadModel(edMod);
    UnloadRenderTexture(target);

    CloseWindow();        // Close window and OpenGL context
    //--------------------------------------------------------------------------------------

    return 0;
}


