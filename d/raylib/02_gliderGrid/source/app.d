// Adapted from: https://www.reddit.com/r/raylib/comments/v2su1s/help_with_dynamic_mesh_creation_in_raylib/

/*
***** DEV PLAN *****
[ ] Draw the entire glider
[ ] Rotate glider
[ ] Roll glider while flying in a circle
	* https://www.raylib.com/examples/models/loader.html?name=models_yaw_pitch_roll
	* https://bedroomcoders.co.uk/aligning-a-model-with-a-terrain-raylib/
*/

import core.stdc.stdlib; // `malloc`
import raylib; // --------- easy graphics

class TriMesh{
	/// Members ///
	int   N_tri; // Number of triangles 
	ulong vDex; //- Index offset for vertex coords
	ulong iDex; //- Index offset for face index
	Mesh  mesh; //- Raylib mesh geometry
	Model model; // Raylib drawable model

	/// Constructors ///
	this( int n ){
		// Init mesh
		vDex  = 0;
		iDex  = 0;
		N_tri = n;
		mesh  = Mesh();
		
		// Init geo memory
		mesh.triangleCount = n;
    	mesh.vertexCount   = n * 3;
		mesh.vertices      = cast(float*)malloc(float.sizeof * mesh.vertexCount * 3);
    	mesh.indices       = cast(ushort*)malloc(ushort.sizeof * mesh.vertexCount);
	}

	public bool push_vertex( float x, float y, float z ){
		if(vDex+3 > mesh.vertexCount*3){  return false;  }else{
			mesh.vertices[vDex + 0] = x;
			mesh.vertices[vDex + 1] = y;
			mesh.vertices[vDex + 2] = z;
			vDex += 3;
			return true;
		}
	}

	public bool push_triangle( ushort p1, ushort p2, ushort p3 ){
		if(iDex+3 > mesh.vertexCount){  return false;  }else{
			mesh.indices[iDex + 0] = p1;
			mesh.indices[iDex + 1] = p2;
			mesh.indices[iDex + 2] = p3;
			iDex += 3;
			return true;
		}
	}

	public void load_geo(){
		UploadMesh(&mesh, true);
    	model = LoadModelFromMesh(mesh);
	}
}

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

	TriMesh tmsh = new TriMesh(1);
	tmsh.push_vertex( 0.0, 0.0, 0.0 );
	tmsh.push_vertex( 0.0, 0.0, 1.0 );
	tmsh.push_vertex( 0.0, 1.0, 0.0 );
	tmsh.push_triangle( 0, 1, 2 );
	tmsh.load_geo();

	while (!WindowShouldClose())
	{
		BeginDrawing();
		ClearBackground(Colors.RAYWHITE);

        BeginMode3D(camera);
        // DrawModelWires(tmsh.model, Vector3(0.0, 0.0, 0.0), 1.00, Colors.RED);
        DrawModel(tmsh.model, Vector3(0.0, 0.0, 0.0), 1.00, Colors.RED);
        DrawGrid(10, 1.0);
        EndMode3D();

		DrawText("Hello, World!", 400, 300, 28, Colors.BLACK);
		EndDrawing();
	}
	CloseWindow();
}