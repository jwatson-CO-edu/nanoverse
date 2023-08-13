// https://www.reddit.com/r/raylib/comments/v2su1s/help_with_dynamic_mesh_creation_in_raylib/

#include <raylib.h>
#include <rlgl.h>

// Utility function to prepare a mesh made of n triangles
// only fills vertices and indices. rest of the member are not used.
void makeTriangleMesh(Mesh &mesh, int n)
{
  mesh.triangleCount = n;
  mesh.vertexCount = n * 3;
  mesh.vertices = new float[mesh.vertexCount * 3];
  mesh.indices = new unsigned short[mesh.vertexCount];

  for (int i = 0; i < n; ++i)
  {
    float triangle[9] = {0.f, 0.f, static_cast<float>(i), 1.f, 0.f, static_cast<float>(i), 0.f, 1.f, static_cast<float>(i)};
    for (int j = 0; j < 9; ++j)
      mesh.vertices[i * 9 + j] = triangle[j];
  }
  for (unsigned short ix = 0; ix < n * 3; ++ix)
    mesh.indices[ix] = ix;
}

int main()
{
  Mesh mesh{};
  Model model;
  int numTriangles = 1;

  InitWindow(800, 600, "Dynamic Mesh test");
  SetTargetFPS(60);
  rlDisableBackfaceCulling();
  rlEnableSmoothLines();
  Camera camera = {{5.0f, 5.0f, 5.0f}, {0.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f}, 45.0f, 0};

  makeTriangleMesh(mesh, numTriangles); // start with 1 triangle
  UploadMesh(&mesh, true);
  model = LoadModelFromMesh(mesh);
 
  while (!WindowShouldClose()) // ESC
  {
    if (IsKeyPressed('Y')) // Method 2: not working :-(
    {
      Mesh &modelMesh = model.meshes[0];
      makeTriangleMesh(modelMesh, ++numTriangles);
      UpdateMeshBuffer(modelMesh, 0, modelMesh.vertices, sizeof(float) * modelMesh.vertexCount * 3, 0);
      // from UploadMesh: index of vbo for indices is 6
      UpdateMeshBuffer(modelMesh, 6, modelMesh.indices, sizeof(unsigned int) * modelMesh.vertexCount, 0);
    }
    if (IsKeyPressed('T')) // Method 1: working!
    {
      UnloadModel(model);
      mesh = {};
      makeTriangleMesh(mesh, ++numTriangles);
      UploadMesh(&mesh, true);
      model = LoadModelFromMesh(mesh);
    }

    BeginDrawing();
    ClearBackground(RAYWHITE);

    BeginMode3D(camera);
    DrawModelWires(model, {0.0f, 0.0f, 0.0f}, 1.0f, RED);
    DrawGrid(10, 1.0);
    EndMode3D();
    EndDrawing();
  }

  UnloadModel(model);
  CloseWindow();
  return 0;
}