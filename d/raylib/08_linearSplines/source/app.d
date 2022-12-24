////////// INIT ////////////////////////////////////////////////////////////////////////////////////

import std.stdio; // ------ writeline
import core.time;
import core.math; // `sqrt`
import std.math.exponential; // `pow`
import std.random;
import core.stdc.stdlib; //abs

import raylib; // --------- easy graphics

// import toybox;

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
	Node3f* tail;
    Node3f* head;
}


Vector3 get_midpoint( Edge3f* edge ){
    // Get the midpoint of an `edge`
    return Vector3(
        (edge.head.point.x + edge.tail.point.x) / 2.0,
        (edge.head.point.y + edge.tail.point.y) / 2.0,
        (edge.head.point.z + edge.tail.point.z) / 2.0
    );
}

float get_length( Edge3f* edge ){
    // Get the length of an `edge`
    return Vector3Distance( edge.head.point, edge.tail.point );
}

Vector3 vec3( Vector3 original ){
    // `Vector3` copy constructor
    return Vector3(
        original.x,
        original.y,
        original.z
    );
}

Edge3f*[] get_linear_mid_spline( Edge3f* e1, Edge3f* e2, uint steps = 10 ){
    // Create a linear spline between midpoints of the `e1` and `e2`
    // NOTE: This function assumes that the tail of `e2` is near to, or coincident with, the head of `e1`
    // NOTE: This will work for, and be colinear with, colinear edges

    Edge3f*[] rtnEdges;
    Edge3f*   rule = new Edge3f(
        new Node3f( vec3( e1.head.point ) ),
        new Node3f( vec3( e1.tail.point ) )
    );
    Vector3 e1dir     = Vector3Normalize( Vector3Subtract( e1.head.point, e1.tail.point ) );
    // Vector3 e1org     = vec3( e1.tail.point );
    Vector3 e1org     = get_midpoint( e1 );
    Vector3 e2dir     = Vector3Normalize( Vector3Subtract( e2.head.point, e2.tail.point ) );
    Vector3 e2org     = vec3( e2.tail.point );
    // Vector3 e2org     = get_midpoint( e2 );
    Node3f* lastPoint = new Node3f( get_midpoint( e1 ) );
    Node3f* currPoint = null;
    float   mag1Step  = get_length( e1 ) / 2.0 / steps;
    float   mag2Step  = get_length( e2 ) / 2.0 / steps;

    for( uint i = 1; i <= steps; i++ ){
        
        // 1. Construct the rule
        rule.tail.point = Vector3Add( e1org, Vector3Scale( e1dir, mag1Step*i ) );
        rule.head.point = Vector3Add( e2org, Vector3Scale( e2dir, mag2Step*i ) );
        
        // 2. Find the midpoint of the rule as the current point
        // currPoint = new Node3f( get_midpoint( rule ) );
        currPoint = new Node3f( Vector3Add(
            rule.tail.point,
            Vector3Scale( 
                Vector3Normalize( Vector3Subtract( rule.head.point, rule.tail.point ) ), 
                get_length( rule ) / steps * i 
            )

        ) );

        // 3. Construct the next segment of the spline
        rtnEdges ~= new Edge3f( currPoint, lastPoint );

        // 4. Setup next iter
        lastPoint = currPoint;
    }

    return rtnEdges;
}


Edge3f*[] spline_loop_from_edge_loop( Edge3f*[] edges, uint steps = 10 ){
    // Create splines between segment loop midpoints such as to create a smooth loop of splines 
    Edge3f*[] rtnSplines; 
    Edge3f*   lastEdge = edges[0];
    // foreach( Edge3f* currEdge; edges[1..$-1] ){
    foreach( Edge3f* currEdge; edges ){
        rtnSplines ~= get_linear_mid_spline( lastEdge, currEdge, steps );
        lastEdge = currEdge;
    }
    // Close the spline
    rtnSplines ~= get_linear_mid_spline( lastEdge, edges[0], steps );
    // Close the loop
    // rtnSplines[$-1].head = rtnSplines[0].tail;
    rtnSplines[0].head = rtnSplines[$-1].tail;
    return rtnSplines;
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


// struct Arc3f{
//     Vector3 center;
//     Vector3 normal;
//     Vector3 start;
//     float   angle_rad; 
// }




////////// MAIN ////////////////////////////////////////////////////////////////////////////////////


void main(){
    rnd = Random( unpredictableSeed );
	

    // call this before using raylib
	validateRaylibBinding();

    // Create geometry

    Node3f    p1     = Node3f( Vector3(0,0,0) ); 
    Node3f    p2     = Node3f( Vector3(0,2,0) ); 
    Node3f    p3     = Node3f( Vector3(0,0,2) ); 
    Node3f    p4     = Node3f( Vector3(0,2,2) ); 
    Edge3f    e1     = Edge3f( &p2, &p1 );
    Edge3f    e2     = Edge3f( &p1, &p3 );
    Edge3f    e3     = Edge3f( &p3, &p4 );
    Edge3f    e4     = Edge3f( &p4, &p2 );
    Edge3f*[] edges  = [&e1, &e2, &e3, &e4];
    // Edge3f*[] spline = get_linear_mid_spline( &e1, &e2, 10 );
    Edge3f*[] spline = spline_loop_from_edge_loop( edges, 10 );
    
	
    // Window / Display Params
    InitWindow( 1200, 900, "Linear Splines" );
    SetTargetFPS( 60 );
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
    // Vector3 trailFront = edges[ edgeDex ].head.point;
    

    while ( !WindowShouldClose() ){
		BeginDrawing();
		ClearBackground( Colors.RAYWHITE );

        BeginMode3D( camera );
        
        // Paint track
        // paint_edges( edges  );
        paint_edges( spline );

        // Follow track


        EndMode3D();

		// DrawText("Hello, World!", 400, 300, 28, Colors.BLACK);
		EndDrawing();
	}
	CloseWindow();

    writeln( spline.length );
}