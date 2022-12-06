////////// INIT ////////////////////////////////////////////////////////////////////////////////////


import raylib; // --------- easy graphics
import std.stdio; // ------ writeline
import std.range;
import core.time;
import core.math; // `sqrt`
import std.math.exponential; // `pow`
import std.random;

import rlights;
import toybox;


////////// CONTRAIL ////////////////////////////////////////////////////////////////////////////////

class Contrail{
	// Queue of triangles representing a plume that decays from back to front
	Vector3[2][] coords; // - 3D coordinates that define the trail, back of list is the head
	ushort /*-*/ N_seg; // -- Number of total segments the trail can contain
	ushort /*-*/ C_seg; // -- Count of current segments that the trail contains
	Color /*--*/ color; // -- Color of the trail
	float /*--*/ bgnAlpha; // Alpha value at the head of the trail
	float /*--*/ endAlpha; // Alpha value at the tail of the trail
	

	/// Constructors ///
	this( ushort n, Color c, float bgnA = 1.0f, float endA = 0.0f ){
		N_seg    = n;
		C_seg    = 0;
		color    = c;
		bgnAlpha = bgnA;
		endAlpha = endA;
	}

	ushort push_segment( Vector3 pnt1, Vector3 pnt2 ){
		// Vector3[2] leadingEdge = [ pnt1, pnt2 ];
		if( C_seg < N_seg ){
			C_seg++;
			// coords ~=  leadingEdge;
			coords ~= [ pnt1, pnt2 ];
		}else{
			coords.popFront();
			// coords ~= leadingEdge;
			coords ~= [ pnt1, pnt2 ];
		}
		return C_seg;
	}

	void draw_segments(){
		float divAlpha = (bgnAlpha - endAlpha) / N_seg;
		float curAlpha = bgnAlpha;
		if( C_seg > 1 ){
			for( ushort i = cast(ushort)(C_seg-1); i >= 1; i-- ){
				DrawTriangle3D(
					coords[i][1], coords[i][0], coords[i-1][0], 
					ColorAlpha( color, curAlpha )
				);
				DrawTriangle3D(
					coords[i-1][0], coords[i-1][1], coords[i][1], 
					ColorAlpha( color, curAlpha )
				);
				curAlpha -= divAlpha;
			}
		}
	}
}


////////// ACCESSORIES /////////////////////////////////////////////////////////////////////////////

class RectXY{
	// Filled, axis-aligned rectangle on the XY plane, +Z is UP
	Vector3    center;
	float /**/ xSide;
	float /**/ ySide;
	Vector3[4] corners;
	Color /**/ color;

	this( Vector3 cntr, float xLen, float yLen, Color clr ){
		center = cntr;
		xSide  = xLen;
		ySide  = yLen;
		color  = clr;
		/*   +X
		    1---0  
		 +Y | / |
		    2---3 */
		corners[0] = Vector3( cntr.x + xLen/2.0, cntr.y - yLen/2.0, cntr.z );
		corners[1] = Vector3( cntr.x + xLen/2.0, cntr.y + yLen/2.0, cntr.z );
		corners[2] = Vector3( cntr.x - xLen/2.0, cntr.y + yLen/2.0, cntr.z );
		corners[3] = Vector3( cntr.x - xLen/2.0, cntr.y - yLen/2.0, cntr.z );
	}

	void draw(){
		DrawTriangle3D( corners[0], corners[1], corners[2], color );
		DrawTriangle3D( corners[0], corners[2], corners[3], color );
	}
}

class Triangle{
	// A lone, filled triangle in 3D space
	Vector3[3] corners;
	Color /**/ color;

	this( Vector3 p1, Vector3 p2, Vector3 p3, Color clr ){
		corners[0] = p1;
		corners[1] = p2;
		corners[2] = p3;
	}

	void draw(){
		DrawTriangle3D( corners[0], corners[1], corners[2], color );
	}
}

////////// VECTOR MATH /////////////////////////////////////////////////////////////////////////////

float vec3_mag( Vector3 vec ){  return sqrt( pow( vec.x, 2 ) + pow( vec.y, 2 ) + pow( vec.z, 2 ) );  }

Vector3 vec3_divide( Vector3 vec, float div ){
	return Vector3(
		vec.x / div,
		vec.y / div,
		vec.z / div
	);
}

Vector3 vec3_unit( Vector3 vec ){
	float mag = vec3_mag( vec );
	if( mag == 0.0 )  
		return vec;
	else 
		return vec3_divide( vec, mag );
}

Vector3 vec3_mult( Vector3 vec, float factor ){
	return Vector3(
		vec.x * factor,
		vec.y * factor,
		vec.z * factor
	);
}


////////// CAMERA //////////////////////////////////////////////////////////////////////////////////

class FlightFollowThirdP_Camera{
	// Aircraft drags the camera like in games

	Camera  camera; // --- The actual RayLib camera
	Vector3 trgtCenter; // Position of the target
	// Matrix  trgtXform; //- Orientation of the target
	float   offset_d; // - Desired camera offset in meters

	this( float desiredOffset_m, Vector3 tCenter, Matrix tXform ){
		trgtCenter = tCenter; 
		// trgtXform  = tXform; 
		offset_d   = desiredOffset_m;

		camera = Camera(
			Vector3Add( tCenter, Vector3Transform( Vector3( 0.0, 0.0, -offset_d ), tXform) ), // Camera location, world frame
			tCenter, // ----------------------------------------------------- Camera target, world frame
			Vector3(0.0, 0.0, 1.0), // -------- Up vector
			45.0, // FOV
			0 // ???
		);
	}

	public void update_target_position( Vector3 tCenter ){  trgtCenter = tCenter;  }
	
	// public void update_target_orientation( Matrix tXform ){  trgtXform = tXform;  }

	public void advance_camera(){
		// Move the camera after all the target updates are in

		Vector3 dragVec = vec3_mult( 
			vec3_unit( Vector3Subtract( camera.position, trgtCenter ) ), 
			offset_d 
		);
		camera.position = Vector3Add( trgtCenter, dragVec );
		camera.target   = trgtCenter;
		// camera.up       =  Vector3Transform( Vector3(0.0, 0.0, 1.0), trgtXform);
		// camera.Update();
	}

	public void begin(){  BeginMode3D(camera);  }
	public void end(){  EndMode3D();  }

}

class FlightThirdP_Camera{
	// Camera control for a 3rd person view of an aircraft/spacecraft
	// 2022-09-22: Right now fixing the camera to the ship by a rigid "stick" 
	//             without obstacle avoidance or damping

	Camera  camera; // --- The actual RayLib camera
	Vector3 trgtCenter; // Position of the target
	Matrix  trgtXform; //- Orientation of the target
	Vector3 offset_t; // - Desired camera offset in the target's frame

	this( Vector3 desiredOffset_V3, Vector3 tCenter, Matrix tXform ){
		trgtCenter = tCenter; // Position of the target
		trgtXform  = tXform; // -------- Orientation of the target
		offset_t   = desiredOffset_V3; // --------- Desired camera offset in the target's frame
		camera = Camera(
			Vector3Add( tCenter, Vector3Transform( offset_t, trgtXform) ), // Camera location, world frame
			tCenter, // ----------------------------------------------------- Camera target, world frame
			Vector3Transform( Vector3(0.0, 0.0, 1.0), trgtXform), // -------- Up vector
			45.0, 
			0
		);
	}

	public void update_target_position( Vector3 tCenter ){  trgtCenter = tCenter;  }
	
	public void update_target_orientation( Matrix tXform ){  trgtXform = tXform;  }

	public void advance_camera(){
		// Move the camera after all the target updates are in
		camera.position = Vector3Add( trgtCenter, Vector3Transform( offset_t, trgtXform) );
		camera.target   = trgtCenter;
		camera.up       =  Vector3Transform( Vector3(0.0, 0.0, 1.0), trgtXform);
		// camera.Update();
	}

	public void begin(){  BeginMode3D(camera);  }
	public void end(){  EndMode3D();  }
	
}



////////// MAIN ////////////////////////////////////////////////////////////////////////////////////

auto clock = MonoTime();
float curr_time_s(){  return cast(float)(clock.currTime().ticks()) / cast(float)(clock.ticksPerSecond());  }

void main(){
	// call this before using raylib
	validateRaylibBinding();
	
    // Window / Display Params
    InitWindow(800, 600, "Hello, Raylib-D!");
    SetTargetFPS(60);
    rlDisableBackfaceCulling();
    rlEnableSmoothLines();

	// Timing
	float tBgn = curr_time_s();
	float tEnd;
	Vector3 posn;

	// Randomness
	auto rnd = Random( unpredictableSeed );

	// FrameAxes worldFrame = new FrameAxes();
	GridXY    worldGrid  = new GridXY( 
		Vector3(0.0, 0.0, 0.0), 
		  1.0, 
		500, // https://github.com/raysan5/raylib/issues/1051#issue-543279679
		Colors.GRAY, // https://robloach.github.io/raylib-cpp/classraylib_1_1_color.html
		false
	);
	
	RectXY[] panels;
	for( uint i = 0; i < 8000; i++ ){
		panels ~= new RectXY( 
			Vector3(
				uniform(-201, 199, rnd) + 0.5f,
				uniform(-201, 199, rnd) + 0.5f,
				0.0
			), 
			1.0, 1.0, Colors.GRAY );
	}

	Triangle[] tris;
	Vector3 cntr, p1, p2, p3;
	float maxRad = 20.0f;
	for( uint i = 0; i < 1000; i++ ){
		cntr = Vector3(
			uniform( -200.0f, 200.0f, rnd ),
			uniform( -200.0f, 200.0f, rnd ),
			uniform(    0.0f, 100.0f, rnd )
		);
		p1 = Vector3Add(
			cntr,
			Vector3(
				uniform(-maxRad, maxRad, rnd),
				uniform(-maxRad, maxRad, rnd),
				uniform(-maxRad, maxRad, rnd)
			)
		);
		p2 = Vector3Add(
			cntr,
			Vector3(
				uniform(-maxRad, maxRad, rnd),
				uniform(-maxRad, maxRad, rnd),
				uniform(-maxRad, maxRad, rnd)
			)
		);
		p3 = Vector3Add(
			cntr,
			Vector3(
				uniform(-maxRad, maxRad, rnd),
				uniform(-maxRad, maxRad, rnd),
				uniform(-maxRad, maxRad, rnd)
			)
		);
		tris ~= new Triangle( 
			p1, p2, p3, 
			ColorFromNormalized( Vector4(
				uniform( -1.0, 1.0, rnd),
				uniform( -1.0, 1.0, rnd),
				uniform( -1.0, 1.0, rnd),
				uniform( -1.0, 1.0, rnd)
			) )
		);
	}


	GridXY    skyGrid  = new GridXY( 
		Vector3(0.0, 0.0, 200.0), 
		  1.0, 
		500, // https://github.com/raysan5/raylib/issues/1051#issue-543279679
		Colors.BLUE, // https://robloach.github.io/raylib-cpp/classraylib_1_1_color.html
		false
	);

	DeltaShip glider = new DeltaShip( 2.0 );
	glider.rotate_RPY( 0.0, 3.1416/2.0, 0.0 );
	glider.set_XYZ(0.0, 0.0, 2.0);

	Contrail contrail = new Contrail( 20, Colors.GRAY, 1.0f, 0.0f );

	// glider.set_RPY( 0.0, 0.0, 0.0 ); // 3.1416/2.0

	// Shaders
	// https://www.raylib.com/examples/shaders/loader.html?name=shaders_basic_lighting
	// Load basic lighting shader
	Shader shader = LoadShader("shaders/lighting.vs", "shaders/lighting.fs");
	// Get some required shader locations
	shader.locs[ShaderLocationIndex.SHADER_LOC_VECTOR_VIEW] = GetShaderLocation(shader, "viewPos");
	// Ambient light level (some basic lighting)
    int ambientLoc = GetShaderLocation(shader, "ambient");
	float[4] arr = [0.1f, 0.1f, 0.1f, 1.0f];
	SetShaderValue( 
		shader, ambientLoc, arr.ptr, ShaderUniformDataType.SHADER_UNIFORM_VEC4
	); //                   ^^^^^^^---void* here
	// Assign our lighting shader to model
	glider.model.materials[0].shader = shader;
	// Create lights
    Light[LIGHT_CONST.MAX_LIGHTS] lights; // = { 0 };
    lights[0] = CreateLight( LightType.LIGHT_POINT, Vector3( -2, 1, -2 ), Vector3Zero(), Colors.YELLOW, shader);
    lights[1] = CreateLight( LightType.LIGHT_POINT, Vector3( 2, 1, 2 ), Vector3Zero(), Colors.RED, shader);
    lights[2] = CreateLight( LightType.LIGHT_POINT, Vector3( -2, 1, 2 ), Vector3Zero(), Colors.GREEN, shader);
    lights[3] = CreateLight( LightType.LIGHT_POINT, Vector3( 2, 1, -2 ), Vector3Zero(), Colors.BLUE, shader);

	FlightFollowThirdP_Camera camera = new FlightFollowThirdP_Camera(
		7.0,
		glider.get_XYZ(),
		glider.T
	);

	while (!WindowShouldClose()){

		/// Update ///
		if( IsKeyDown( KeyboardKey.KEY_LEFT  ) ){  glider.rotate_RPY( 0.0,  0.0,          3.1416/120.0 );  }
		if( IsKeyDown( KeyboardKey.KEY_RIGHT ) ){  glider.rotate_RPY( 0.0,  0.0,         -3.1416/120.0 );  }
		if( IsKeyDown( KeyboardKey.KEY_UP    ) ){  glider.rotate_RPY( 0.0,  3.1416/120.0, 0.0          );  }
		if( IsKeyDown( KeyboardKey.KEY_DOWN  ) ){  glider.rotate_RPY( 0.0, -3.1416/120.0, 0.0          );  }

		camera.update_target_position( glider.get_XYZ() );
		// camera.update_target_orientation( glider.T );
		camera.advance_camera();

		/// Rendering ///

		BeginDrawing();
		ClearBackground(Colors.RAYWHITE);

		
        camera.begin();
		

		glider.z_thrust( 2.0/60.0 );
		glider.draw();
		tEnd = curr_time_s();
		if( (tEnd - tBgn) > 0.10 ){
			posn = glider.get_XYZ();
			// writeln( posn );
			contrail.push_segment(
				Vector3Add( posn, Vector3Transform( Vector3( -0.125, 0.0, 0.0 ), glider.T) ),
				Vector3Add( posn, Vector3Transform( Vector3(  0.125, 0.0, 0.0 ), glider.T) )
			);
			tBgn = curr_time_s();
		}
		contrail.draw_segments();

		// worldFrame.draw();
		worldGrid.draw();
		foreach( RectXY panel; panels ){
			panel.draw();
		}
		// skyGrid.draw();
		foreach( Triangle tri; tris ){
			tri.draw();
		}

		// DrawGrid(10, 1.0);
        
		camera.end();

		// DrawText("Hello, World!", 400, 300, 28, Colors.BLACK);
		EndDrawing();
	}

	CloseWindow();
}