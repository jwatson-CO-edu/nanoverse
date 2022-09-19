// Adapted from: https://www.reddit.com/r/raylib/comments/v2su1s/help_with_dynamic_mesh_creation_in_raylib/

/*
***** DEV PLAN *****
[Y] Draw the entire glider - 2022-09-19: Complete and correct, but with no shading
[Y] Rotate glider - 2022-09-19: Complete and correct, but with no shading
[Y] Roll glider while flying in a circle
	* https://www.raylib.com/examples/models/loader.html?name=models_yaw_pitch_roll
	* https://bedroomcoders.co.uk/aligning-a-model-with-a-terrain-raylib/
{ } Simple lighting / shading?
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
	float x; // --- World X pos
	float y; // --- World Y pos
	float z; // --- World Z pos
	Matrix R; // -- World rotation
	float r; // --- Local roll  angle
	float p; // --- Local pitch angle
	float w; // --- Local yaw   angle
	Matrix T; // -- World orientation

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

		// Init pose
		x = 0.0; // --- World X pos
		y = 0.0; // --- World Y pos
		z = 0.0; // --- World Z pos
		r = 0.0; // --- Local roll  angle
		p = 0.0; // --- Local pitch angle
		w = 0.0; // --- Local yaw   angle
	}

	/// Methods ///

	public bool push_vertex( float x, float y, float z ){
		// Add a vertex
		if(vDex+3 > mesh.vertexCount*3){  return false;  }else{
			mesh.vertices[vDex + 0] = x;
			mesh.vertices[vDex + 1] = y;
			mesh.vertices[vDex + 2] = z;
			vDex += 3;
			return true;
		}
	}

	public bool push_triangle( ushort p1, ushort p2, ushort p3 ){
		// Add a triangle in CCW order spanning 3 vertex indices
		if(iDex+3 > mesh.vertexCount){  return false;  }else{
			mesh.indices[iDex + 0] = p1;
			mesh.indices[iDex + 1] = p2;
			mesh.indices[iDex + 2] = p3;
			iDex += 3;
			return true;
		}
	}

	public void load_geo(){
		// Send triangle mesh geometry to RayLib, needed for drawing
		UploadMesh(&mesh, true);
    	model = LoadModelFromMesh(mesh);
	}

	public void set_XYZ( float x_, float y_, float z_ ){
		// Set the world XYZ position of the model
		x = x_;
		y = y_;
		z = z_;
	}

	public void set_RPY( float r_, float p_, float y_ ){
		// Set the world Roll, Pitch, Yaw of the model
		R = MatrixRotateXYZ( Vector3( p_, y_, r_ ) );
	}

	public void rotate_RPY( float r_, float p_, float y_ ){
		r += r_;
		p += p_;
		w += y_;
		T = MatrixMultiply( R, MatrixRotateXYZ( Vector3( p, w, r ) ) );
	}

	public void z_thrust( float d = 0.0 ){
		// Advance a plane model in the forward direction (local Z)
		Vector3 vec = Vector3Transform(Vector3( 0.0, 0.0, d ), T);
		x += vec.x;
		y += vec.y;
		z += vec.z;
	}

	public void draw(){
		// Draw the model at the current pose
		model.transform = T;
        DrawModelWires(model, Vector3(x, y, z), 1.0, Colors.RED);
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
        Vector3(10.0, 10.0, 10.0), 
        Vector3(5.0, 0.0, 0.0), 
        Vector3(0.0, 0.0, 1.0), 
        45.0, 
        0
    );

	// Build Delta Glider //
	float wingspan  = 10.0;
	float fusFrac   =    0.5;
	float sweptFrac =    0.75;
	float thickFrac =    0.25;

	TriMesh tmsh = new TriMesh( 8 ); // MUST match the total number of triangles to draw!
	// Verts
	tmsh.push_vertex(  0.0        ,  0.0                  , 0.0                 + wingspan*sweptFrac/2.0 ); // 0, Front
	tmsh.push_vertex(  0.0        , +wingspan*thickFrac/2 , -wingspan*fusFrac/2 + wingspan*sweptFrac/2.0 ); // 1, Top peak
	tmsh.push_vertex(  0.0        , -wingspan*thickFrac/2 , -wingspan*fusFrac/2 + wingspan*sweptFrac/2.0 ); // 2, Bottom peak
	tmsh.push_vertex(  0.0        ,  0.0                  , -wingspan*fusFrac   + wingspan*sweptFrac/2.0 ); // 3, Back
	tmsh.push_vertex( -wingspan/2 ,  0.0                  , -wingspan*sweptFrac + wingspan*sweptFrac/2.0 ); // 4, Left wingtip
	tmsh.push_vertex( +wingspan/2 ,  0.0                  , -wingspan*sweptFrac + wingspan*sweptFrac/2.0 ); // 5, Right wingtip 
	// Faces
	tmsh.push_triangle( 0,4,1 ); // Left  Top    Front
	tmsh.push_triangle( 1,4,3 ); // Left  Top    Back 
	tmsh.push_triangle( 5,0,1 ); // Right Top    Front
	tmsh.push_triangle( 5,1,3 ); // Right Top    Back 
	tmsh.push_triangle( 0,2,4 ); // Left  Bottom Front
	tmsh.push_triangle( 2,3,4 ); // Left  Bottom Back
	tmsh.push_triangle( 0,5,2 ); // Right Bottom Front
	tmsh.push_triangle( 2,5,3 ); // Right Bottom Back

	tmsh.load_geo();

	tmsh.set_RPY( 0.0, 0.0, 0.0 ); // 3.1416/2.0
	tmsh.rotate_RPY( 0.0, -3.1416/2.0, 0.0 );

	while (!WindowShouldClose())
	{
		BeginDrawing();
		ClearBackground(Colors.RAYWHITE);

        BeginMode3D(camera);
        
		
		tmsh.rotate_RPY( 0.0, 0.0, 2.0*3.1416/60.0/4.0 ); // Test local yaw
		tmsh.rotate_RPY( 2.0*3.1416/60.0/4.0, 0.0, 0.0 ); // Test local roll
		// tmsh.rotate_RPY( 0.0, 2.0*3.1416/60.0/4.0, 0.0 ); // Test local pitch

		tmsh.z_thrust( 10.0/60.0 );

		tmsh.draw();

        // DrawGrid(10, 1.0);
        EndMode3D();

		// DrawText("Hello, World!", 400, 300, 28, Colors.BLACK);
		EndDrawing();
	}
	CloseWindow();
}