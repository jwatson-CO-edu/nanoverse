module toybox;

////////// INIT ////////////////////////////////////////////////////////////////////////////////////

import core.stdc.stdlib; // `malloc`
import core.math; // `sqrt`
import std.math.exponential; // `pow`
import std.range;
import raylib; // --------- easy graphics



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



////////// MODELS / MESHES /////////////////////////////////////////////////////////////////////////


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
	Matrix mx; // - Local pitch
	Matrix my; // - Local yaw
	Matrix mz; // - Local roll
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
		x = 0.0; // ------------ World X pos
		y = 0.0; // ------------ World Y pos
		z = 0.0; // ------------ World Z pos
		r = 0.0; // ------------ Local roll  angle
		p = 0.0; // ------------ Local pitch angle
		w = 0.0; // ------------ Local yaw   angle
		mx = MatrixIdentity(); // - Local pitch
		my = MatrixIdentity(); // - Local yaw
		mz = MatrixIdentity(); // - Local roll
		R  = MatrixIdentity(); // Init identity
		T  = MatrixIdentity(); // Init identity
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
    	model = LoadModelFromMesh( mesh );
		// writeln( "geo loaded" );
	}

	public Vector3 get_XYZ(){  return Vector3( x, y, z );  }

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
		mx = MatrixRotateX( p );
		my = MatrixRotateY( r );
		mz = MatrixRotateZ( w );
		T  = MatrixMultiply( mz, my );
		T  = MatrixMultiply( mx, T  );
		T  = MatrixMultiply( T, R );
		// writeln( T );
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
        // DrawModelWires(model, Vector3(x, y, z), 1.0, Colors.RED);
        DrawModel(model, Vector3(x, y, z), 1.0, Colors.RED);  // What happens when you call this with color data populated?
	}
}


class DeltaShip:TriMesh{
	// A cool and simple starship / glider 

	this( float wingspan = 10.0, float fusFrac = 0.5, float sweptFrac = 0.75, float thickFrac = 0.25 ){
		// TriMesh constructor
		super( 8 );
		// Verts
		super.push_vertex(  0.0        ,  0.0                  , 0.0                 + wingspan*sweptFrac/2.0 ); // 0, Front
		super.push_vertex(  0.0        , +wingspan*thickFrac/2 , -wingspan*fusFrac/2 + wingspan*sweptFrac/2.0 ); // 1, Top peak
		super.push_vertex(  0.0        , -wingspan*thickFrac/2 , -wingspan*fusFrac/2 + wingspan*sweptFrac/2.0 ); // 2, Bottom peak
		super.push_vertex(  0.0        ,  0.0                  , -wingspan*fusFrac   + wingspan*sweptFrac/2.0 ); // 3, Back
		super.push_vertex( -wingspan/2 ,  0.0                  , -wingspan*sweptFrac + wingspan*sweptFrac/2.0 ); // 4, Left wingtip
		super.push_vertex( +wingspan/2 ,  0.0                  , -wingspan*sweptFrac + wingspan*sweptFrac/2.0 ); // 5, Right wingtip
		// Faces
		super.push_triangle( 0,4,1 ); // Left  Top    Front
		super.push_triangle( 1,4,3 ); // Left  Top    Back 
		super.push_triangle( 5,0,1 ); // Right Top    Front
		super.push_triangle( 5,1,3 ); // Right Top    Back 
		super.push_triangle( 0,2,4 ); // Left  Bottom Front
		super.push_triangle( 2,3,4 ); // Left  Bottom Back
		super.push_triangle( 0,5,2 ); // Right Bottom Front
		super.push_triangle( 2,5,3 ); // Right Bottom Back
		// load
		super.load_geo();
	}
}


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


////////// LINE SEGMENTS ///////////////////////////////////////////////////////////////////////////

class FrameAxes{
	// Classic XYX-RGB axes
	float   len; // -- Length of basis vectors
	Vector3 origin; // World position
	Matrix  T; // ---- World orientation

	this(){
		len    = 1.0;
		origin = Vector3( 0.0, 0.0, 0.0 );
		T      = MatrixIdentity();
	}

	this( float unitLen, Vector3 orig, Matrix transform ){
		len    = unitLen;
		origin = orig;
		T      = transform;
	}

	this( float unitLen, Vector3 orig ){
		len    = unitLen;
		origin = orig;
		T      = MatrixIdentity();
	}

	public void draw(){
		// Render classic coordinate axes
		DrawLine3D( // X basis
			origin, 
			Vector3Add( origin, Vector3Transform( Vector3( len, 0.0, 0.0 ), T)  ),
			Colors.RED
		);
		DrawLine3D( // Y basis
			origin, 
			Vector3Add( origin, Vector3Transform( Vector3( 0.0, len, 0.0 ), T)  ),
			Colors.GREEN
		);
		DrawLine3D( // Z basis
			origin, 
			Vector3Add( origin, Vector3Transform( Vector3( 0.0, 0.0, len ), T)  ),
			Colors.BLUE
		);
	}
}

class GridXY{
	// XY grid to replace the stock XZ grid (Z is UP!)
	Vector3   origin; // ----- Location of grid center
	float     unitLen; // ---- Dimension of grid unit
	uint      oneSideLen_u; // Extent of grid in each direction +/-, in units
	Color     color; // ------ Color of grid lines
	bool      drawAxes; // --- Whether or not to draw axes
	FrameAxes frame; // ------ Optional coordinate frame

	this(){
		origin       = Vector3( 0.0, 0.0, 0.0 ); // Location of grid center
		unitLen      =  1.0; // ------------------- Dimension of grid unit
		oneSideLen_u = 10; // --------------------- Extent of grid in each direction +/-, in units
		color        = Colors.BLACK; // ----------- Color of grid lines
		drawAxes     = true; // ------------------- Whether or not to draw axes
		frame        = new FrameAxes( unitLen, Vector3Add( origin, Vector3( 0.0, 0.0, unitLen ) ) ); // -------- Optional coordinate frame
	}

	this( Vector3 orig, float unit = 1.0, uint oneSide_u = 10, Color clr = Colors.BLACK, bool axes = true ){
		origin       = orig; // Location of grid center
		unitLen      = unit; // ------------------- Dimension of grid unit
		oneSideLen_u = oneSide_u; // --------------------- Extent of grid in each direction +/-, in units
		color        = clr; // ----------- Color of grid lines
		drawAxes     = axes; // ------------------- Whether or not to draw axes
		if( drawAxes ){
			frame = new FrameAxes( unitLen, Vector3Add( origin, Vector3( 0.0, 0.0, unitLen ) ) );
		}
	}

	public void draw(){
		// Render an XY grid, with optional coordinate axes (frame debugging)
		float dHalf = unitLen * oneSideLen_u;
		// Draw the center cross
		DrawLine3D( 
			Vector3Add( origin, Vector3( -dHalf, 0.0, 0.0 )  ),
			Vector3Add( origin, Vector3(  dHalf, 0.0, 0.0 )  ),
			color
		);
		DrawLine3D( 
			Vector3Add( origin, Vector3( 0.0, -dHalf, 0.0 )  ),
			Vector3Add( origin, Vector3( 0.0,  dHalf, 0.0 )  ),
			color
		);
		// Draw X and Y lines, double-sided
		for (uint i = 1; i < oneSideLen_u; i++){

			DrawLine3D( 
				Vector3Add( origin, Vector3( -dHalf,  unitLen*i, 0.0 )  ),
				Vector3Add( origin, Vector3(  dHalf,  unitLen*i, 0.0 )  ),
				color
			);
			DrawLine3D( 
				Vector3Add( origin, Vector3( -dHalf, -unitLen*i, 0.0 )  ),
				Vector3Add( origin, Vector3(  dHalf, -unitLen*i, 0.0 )  ),
				color
			);

			DrawLine3D( 
				Vector3Add( origin, Vector3( unitLen*i, -dHalf, 0.0 )  ),
				Vector3Add( origin, Vector3( unitLen*i,  dHalf, 0.0 )  ),
				color
			);
			DrawLine3D( 
				Vector3Add( origin, Vector3( -unitLen*i, -dHalf, 0.0 )  ),
				Vector3Add( origin, Vector3( -unitLen*i,  dHalf, 0.0 )  ),
				color
			);
		}
		// Draw optional axes
		if( drawAxes ){  frame.draw();  }
	}
}