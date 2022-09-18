// Adapted from: https://www.reddit.com/r/raylib/comments/v2su1s/help_with_dynamic_mesh_creation_in_raylib/

import core.stdc.stdlib; // `malloc`
import raylib; // --------- easy graphics

void main()
{
	// call this before using raylib
	validateRaylibBinding();
	
    // Window / Display Params
    InitWindow(800, 600, "Hello, Raylib-D!");
    SetTargetFPS(60);
    rlDisableBackfaceCulling();
    rlEnableSmoothLines();

    // Camera
    Camera camera = Camera(
        Vector3(5.0, 5.0, 5.0), 
        Vector3(0.0, 0.0, 0.0), 
        Vector3(0.0, 1.0, 0.0), 
        45.0, 
        0
    );

    // Entities
    int n = 1;
    Mesh mesh = Mesh();
    mesh.triangleCount = n;
    mesh.vertexCount   = n * 3;
    mesh.vertices      = cast(float*)malloc(float.sizeof * mesh.vertexCount * 3);
    mesh.indices       = cast(ushort*)malloc(ushort.sizeof * mesh.vertexCount);
    
    mesh.vertices[0]   = 0.0;
    mesh.vertices[1]   = 0.0;
    mesh.vertices[2]   = 0.0;
    
    mesh.vertices[3]   = 0.0;
    mesh.vertices[4]   = 0.0;
    mesh.vertices[5]   = 1.0;
    
    mesh.vertices[6]   = 0.0;
    mesh.vertices[7]   = 1.0;
    mesh.vertices[8]   = 0.0;
    
    mesh.indices[0]    = 0;
    mesh.indices[1]    = 1;
    mesh.indices[2]    = 2;

    UploadMesh(&mesh, true);
    Model model = LoadModelFromMesh(mesh);

    while (!WindowShouldClose())
	{
		BeginDrawing();
		ClearBackground(Colors.RAYWHITE);

        BeginMode3D(camera);
        DrawModelWires(model, Vector3(0.0, 0.0, 0.0), 1.00, Colors.RED);
        DrawGrid(10, 1.0);
        EndMode3D();

		DrawText("Hello, World!", 400, 300, 28, Colors.BLACK);
		EndDrawing();
	}
	CloseWindow();
}