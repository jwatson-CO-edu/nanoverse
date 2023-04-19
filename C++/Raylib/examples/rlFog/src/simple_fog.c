/*******************************************************************************************
*
*   raylib example - simple fog lighting material
*
*   This example has been created using raylib 2.5 (www.raylib.com)
*   raylib is licensed under an unmodified zlib/libpng license (View raylib.h for details)
*
*   This example Copyright (c) 2018 Chris Camacho (codifies) http://bedroomcoders.co.uk/captcha/
*
* THIS EXAMPLE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
* EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
* MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
* IN NO EVENT SHALL THE AUTHORS BE LIABLE FOR ANY CLAIM, DAMAGES OR
* OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
* ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
* OTHER DEALINGS IN THE SOFTWARE.
*
* This example may be freely redistributed.
*
********************************************************************************************/

#include "raylib.h"
#include "raymath.h"

#include <stdio.h>

#define RLIGHTS_IMPLEMENTATION
#include "rlights.h"

/*
 *
 * I'm no expert at light calculation in shaders, so really probably a fudge!
 *
 */


int main(void)
{
    // Initialization
    //--------------------------------------------------------------------------------------
    const int screenWidth = 1280;
    const int screenHeight = 720;

    SetConfigFlags(FLAG_MSAA_4X_HINT);  // Enable Multi Sampling Anti Aliasing 4x (if available)
    InitWindow(screenWidth, screenHeight, "raylib example - simple fog material");

    // Define the camera to look into our 3d world
    Camera camera = {(Vector3){ 2.0f, 2.0f, 6.0f }, (Vector3){ 0.0f, 0.5f, 0.0f },
                        (Vector3){ 0.0f, 1.0f, 0.0f }, 45.0f, CAMERA_PERSPECTIVE};

    // Load models
    Model model = LoadModelFromMesh(GenMeshTorus(.4,1,16,32));
    Model model2 = LoadModelFromMesh(GenMeshCube(1,1,1));
    Model model3 = LoadModelFromMesh(GenMeshSphere(.5,32,32));

    // texture the models
    Texture texture = LoadTexture("data/test.png");

    model.materials[0].maps[MAP_DIFFUSE].texture = texture;
    model2.materials[0].maps[MAP_DIFFUSE].texture = texture;
    model3.materials[0].maps[MAP_DIFFUSE].texture = texture;

    Shader shader = LoadShader("data/fogLight.vs", "data/fogLight.fs");
    // load a shader and set up some uniforms
    shader.locs[LOC_MATRIX_MODEL] = GetShaderLocation(shader, "matModel");
    shader.locs[LOC_VECTOR_VIEW] = GetShaderLocation(shader, "viewPos");

    // ambient light level
    int amb = GetShaderLocation(shader, "ambient");
    SetShaderValue(shader, amb, (float[4]){0.2, 0.2, 0.2, 1.0}, UNIFORM_VEC4);

    int fogC = GetShaderLocation(shader, "fogColor");
    SetShaderValue(shader, fogC, (float[4]){0.6, 0.2, 0.2, 1.0}, UNIFORM_VEC4);

    int fogD = GetShaderLocation(shader, "FogDensity");
    float fogDensity = 0.15f;
    SetShaderValue(shader, fogD, &fogDensity, UNIFORM_FLOAT);

    // models 2 & 3 share the first models shader
    model.materials[0].shader = shader;
    model2.materials[0].shader = shader;
    model3.materials[0].shader = shader;

    // using just 1 point light
    CreateLight(LIGHT_POINT, (Vector3){ 0,2,6 }, Vector3Zero(), WHITE, shader);

    SetCameraMode(camera, CAMERA_ORBITAL);  // Set an orbital camera mode

    SetTargetFPS(60);                       // Set our game to run at 60 frames-per-second
    //--------------------------------------------------------------------------------------

    // Main game loop
    while (!WindowShouldClose())            // Detect window close button or ESC key
    {
        // Update
        //----------------------------------------------------------------------------------

        UpdateCamera(&camera);              // Update camera

        if (IsKeyDown(KEY_UP)) {
            fogDensity+=0.001;
            if (fogDensity>1.0) {
                fogDensity = 1.0;
            }
        }
        if (IsKeyDown(KEY_DOWN)) {
            fogDensity-=0.001;
            if (fogDensity<0.0) {
                fogDensity = 0.0;
            }
        }
        SetShaderValue(shader, fogD, &fogDensity, UNIFORM_FLOAT);

        // rotate the torus
        model.transform = MatrixMultiply(model.transform, MatrixRotateX(-0.025));
        model.transform = MatrixMultiply(model.transform, MatrixRotateZ(0.012));

        // update the light shader with the camera view position
        SetShaderValue(shader, shader.locs[LOC_VECTOR_VIEW], &camera.position.x, UNIFORM_VEC3);
        //----------------------------------------------------------------------------------

        // Draw
        //----------------------------------------------------------------------------------
        BeginDrawing();

            ClearBackground((Color){0.6*255,0.2*255,0.2*255,255});

            BeginMode3D(camera);

                // draw the three models
                DrawModel(model, Vector3Zero(), 1.0f, WHITE);
                DrawModel(model2, (Vector3){-2.6,0,0}, 1.0f, WHITE);
                DrawModel(model3, (Vector3){ 2.6,0,0}, 1.0f, WHITE);

                // draw the line of the tori
                for (int i=-20;i<20;i=i+2) {
                    DrawModel(model,(Vector3){i,0,2}, 1.0f, WHITE);
                }

            EndMode3D();

            DrawFPS(10, 10);
            DrawText(FormatText("up/down to change fog density: %f", fogDensity), 10, 30, 20, WHITE);

        EndDrawing();
        //----------------------------------------------------------------------------------
    }

    // De-Initialization
    //--------------------------------------------------------------------------------------
    UnloadModel(model);         // Unload the model
    UnloadModel(model2);
    UnloadModel(model3);
    UnloadTexture(texture);     // Unload the texture
    UnloadShader(shader);

    CloseWindow();              // Close window and OpenGL context
    //--------------------------------------------------------------------------------------

    return 0;
}

