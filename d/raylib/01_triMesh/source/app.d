// Adapted from: https://www.reddit.com/r/raylib/comments/v2su1s/help_with_dynamic_mesh_creation_in_raylib/

import raylib;

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
    Mesh mesh = Mesh();
    float[9] verts = [ 0.0, 0.0, 0.0 , 
                       0.0, 0.0, 1.0 ,
                       0.0, 1.0, 0.0 ];

    int[3]   indcs = [ 0, 1, 2 ];

    float* foo = new float[9];

    mesh.vertices = foo;

	while (!WindowShouldClose())
	{
		// BeginDrawing();
		// ClearBackground(Colors.RAYWHITE);
		// DrawText("Hello, World!", 400, 300, 28, Colors.BLACK);
		// EndDrawing();
	}
	CloseWindow();
}