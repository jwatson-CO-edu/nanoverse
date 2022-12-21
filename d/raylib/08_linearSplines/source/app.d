////////// INIT ////////////////////////////////////////////////////////////////////////////////////

import std.stdio; // ------ writeline
import core.time;
import core.math; // `sqrt`
import std.math.exponential; // `pow`
import std.random;
import core.stdc.stdlib; //abs

import raylib; // --------- easy graphics

import toybox;

// Randomness
Mt19937 rnd;



////////// STRUCTS /////////////////////////////////////////////////////////////////////////////////

struct Node3f{
	// A point on a `Grid3i`
	Vector3   point;
	bool /**/ occupied;
	Edge3f*[] edges;

	this( Vector3 pnt ){
		point     = pnt;
		occupied  = false;
		edges     = [];
	}
}


struct Edge3f{
	// A segment on a `Grid3i`
	Node3f* head;
	Node3f* tail;
}


void paint_edges( Edge3f*[] edges ){
    foreach( Edge3f* edge; edges ){
        DrawLine3D( // X basis
			edge.head.point, 
			edge.tail.point,
			Colors.RED
		);
    }
}


struct Arc3f{
    Vector3 center;
    Vector3 normal;
    Vector3 start;
    float   angle_rad; 
}




////////// MAIN ////////////////////////////////////////////////////////////////////////////////////


void main(){
    rnd = Random( unpredictableSeed );
	

    // call this before using raylib
	validateRaylibBinding();
	
    // Window / Display Params
    InitWindow(1200, 900, "Linear Splines");
    SetTargetFPS(60);
    rlDisableBackfaceCulling();
    rlEnableSmoothLines();

    // Camera
    Camera camera = Camera(
        Vector3(5.0, 0.0, 0.0), 
        Vector3(0.0, 0.0, 0.0), 
        Vector3(0.0, 1.0, 0.0), 
        45.0, 
        0
    );

    float   dPerFrame  = 0.2;
    int     edgeDex    = 0;
    Vector3 trailFront = edges[ edgeDex ].head.point;
    

    while ( !WindowShouldClose() ){
		BeginDrawing();
		ClearBackground( Colors.RAYWHITE );

        BeginMode3D( camera );
        
        // Paint track
        paint_edges( edges );

        // Follow track


        EndMode3D();

		// DrawText("Hello, World!", 400, 300, 28, Colors.BLACK);
		EndDrawing();
	}
	CloseWindow();
}