////////// INIT ////////////////////////////////////////////////////////////////////////////////////

import std.stdio; // ------ writeline
import core.time;
import core.math; // `sqrt`
import std.math.exponential; // `pow`
import std.random;
import core.stdc.stdlib; //abs
import std.range;

import raylib; // --------- easy graphics

// import toybox;

// Randomness
Mt19937 rnd;



////////// GRAPHS //////////////////////////////////////////////////////////////////////////////////

struct Node3f{
	// A point on a `Grid3i`
	Vector3   point;
	bool /**/ occupied;
	Edge3f*[] edges;

	this( Vector3 pnt ){
		// Create an empty node
		point     = pnt;
		occupied  = false;
		edges     = [];
	}

	void add_edge( Edge3f* edge ){
		// Add an edge and mark occupied
		occupied =  true;
		edges    ~= edge;
	}
}


struct Edge3f{
	// Directed edges go from `tail` --to-> `head`
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

Vector3 get_direction( Edge3f* edge ){
	// Get the unit vector from the tail to the head of `edge`
	return Vector3Normalize( Vector3Subtract( edge.head.point, edge.tail.point ) );
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

void paint_edges( Edge3f*[] edges ){
    foreach( Edge3f* edge; edges ){
        DrawLine3D( // X basis
			edge.head.point, 
			edge.tail.point,
			Colors.RED
		);
    }
}

////////// SPLINES /////////////////////////////////////////////////////////////////////////////////

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


////////// RAILS ///////////////////////////////////////////////////////////////////////////////////

Vector3 stochastic_jump( Vector3 start, Vector3 jump, Vector3 noise ){
    // Make a `jump` away from `start` with `noise` added
    Vector3 wiggle = Vector3(
        uniform(-noise.x, noise.x, rnd),
        uniform(-noise.y, noise.y, rnd),
        uniform(-noise.z, noise.z, rnd)
    );
    return Vector3Add( Vector3Add( start, jump ), wiggle );
}


Edge3f*[] random_loop_noisy_grid( Vector3 start, float unitJump, Vector3 noiseBounds, float epsilon ){
    // Do a random, noisy gridlike walk
	// NOTE: Directed edges go from `tail` --to-> `head`

    Vector3[6] vnJumps = [
        Vector3(  unitJump,  0.0f    , 0.0f      ),
        Vector3( -unitJump,  0.0f    , 0.0f      ),
        Vector3(  0.0f    ,  unitJump, 0.0f      ),
        Vector3(  0.0f    , -unitJump, 0.0f      ),
        Vector3(  0.0f    ,  0.0f    ,  unitJump ),
        Vector3(  0.0f    ,  0.0f    , -unitJump ),
    ];
    Vector3   lastPnt  = start;
    Node3f*   lastNode = new Node3f( lastPnt );
    Vector3   currPnt;
    Node3f*   currNode;
    Edge3f*[] edges;
    int       lastDex = cast(int) uniform( 0, vnJumps.length, rnd );
    int       currDex;
    bool      closed  = false;
    float     dist2start;
    while( !closed ){

        currDex = cast(int) uniform( 0, vnJumps.length, rnd );

        if( (uniform( 0.0f, 1.0f, rnd) < epsilon) && (edges.length > 1) ){
            dist2start = Vector3Distance(start, edges[$-1].tail.point);
            while( Vector3Distance(start, Vector3Add( edges[$-1].tail.point, vnJumps[currDex] ) ) >= dist2start){
                currDex = cast(int) uniform( 0, vnJumps.length, rnd );
            }
        }else{
            while( Vector3LengthSqr( Vector3Add( vnJumps[currDex], vnJumps[lastDex] )  ) == 0.0 ){
                currDex = cast(int) uniform( 0, vnJumps.length, rnd );
            }
        }
        
        currPnt = stochastic_jump( lastPnt, vnJumps[ currDex ], noiseBounds );
        currNode = new Node3f( currPnt );

        // edges ~= new Edge3f( currNode, lastNode );
        edges ~= new Edge3f( lastNode, currNode );

        lastPnt  = currPnt;
        lastNode = currNode;
        lastDex  = currDex;

        if( (edges.length > 3) && (Vector3Distance(edges[0].head.point, edges[$-1].tail.point) < unitJump) ){
            closed = true;
            edges ~= new Edge3f( edges[0].head, edges[$-1].tail );
        }  
    }
    return edges;
}


////////// RAIL TRAIL //////////////////////////////////////////////////////////////////////////////

float defaultTurn = 3 * PI/180;

// class Contrail{
class RailTrail{
	// Queue of triangles representing a plume that decays from back to front

	// Rendering Vars
	Vector3[2][] coords; // - 3D coordinates that define the trail, back of list is the head
	ushort /*-*/ N_seg; // -- Number of total segments the trail can contain
	ushort /*-*/ C_seg; // -- Count of current segments that the trail contains
	Color /*--*/ color; // -- Color of the trail
	float /*--*/ bgnAlpha; // Alpha value at the head of the trail
	float /*--*/ endAlpha; // Alpha value at the tail of the trail
	
	// Leading edge vars
	Edge3f*[] rail; // -- Path for the contrail to follow (NOTE: Assumed to be a loop)
	ulong     segDex; //- Index of the current segment at which the leading edge is drawn 
	ulong     lasDex; //- Index of the last    segment 
	float     edgHlf; //- Half length of the leading edge // Length// = 4.0;
	float     edgAng; //- Angle of the leading edge relative to the segment normal // = 0.0;
	float     frmTrn; //- Radians that the leading edge should turn about the rail per frame 
	Vector3   edgDir; //- Direction that the leading edge points in 3D space
	Edge3f*   lastSeg; // Pointer to the last    rail segment
	Edge3f*   currSeg; // Pointer to the current rail segment
	Vector3   n_s; // --- Current segment normal (used to calculate `edgAng`)
	Vector3   n_last; //- Last    segment normal // = Vector3( 1, 1, 0 );
	
	/// Constructors ///
	this( Edge3f*[] path, float leadLength, ushort n, Color c, 
		  float bgnA = 1.0f, float endA = 0.0f, float deltaAng = defaultTurn ){

		// Init rendering Vars
		N_seg    = n;
		C_seg    = 0;
		color    = c;
		bgnAlpha = bgnA;
		endAlpha = endA;

		// Init leading edge vars
		rail    = path;
		segDex  = 0;
		lasDex  = rail.length-1;
		edgHlf  = leadLength / 2.0;
		frmTrn  = deltaAng;
		currSeg = rail[ segDex ];
		lastSeg = rail[ lasDex ];
		edgDir  = Vector3Normalize( Vector3CrossProduct( get_direction( currSeg ), Vector3( 1, 0, 0 ) ) );
		n_last  = Vector3CrossProduct( get_direction( lastSeg ), get_direction( currSeg ) );
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

	void advance_by_one_rail_segment(){
		// Move the leading edge ahead one segment and push one contrail segment

		// Get current and last segments
		segDex  = (segDex+1) % (rail.length);
		lasDex  = (lasDex+1) % (rail.length);
		currSeg = rail[ segDex ];
		lastSeg = rail[ lasDex ];

		// Get the normal of a plane containing the segments
		n_s = Vector3CrossProduct( get_direction( lastSeg ), get_direction( currSeg ) );
        if( Vector3DotProduct( n_s, n_last ) < 0.0 ){
            n_s = Vector3Negate( n_s );
        }

		// Calculate angle between the segment normal and the bar
		edgAng = Vector3Angle( edgDir, n_s );
		// If the angle is positive or negative when measured from the segment normal to the bar, then negate angle
		if( Vector3DotProduct( edgDir, Vector3CrossProduct( n_s, get_direction( lastSeg ) ) ) < 0.0 )
			edgAng *= -1.0;

		// Rotate from the segment normal about the current segment, adding frame rotation delta
		edgDir = Vector3Normalize( Vector3RotateByAxisAngle( n_s, get_direction( currSeg ), -(edgAng+frmTrn) ) );

		// Push trail segment
		push_segment( 
			Vector3Add( currSeg.tail.point, Vector3Scale( edgDir,  edgHlf ) ), 
			Vector3Add( currSeg.tail.point, Vector3Scale( edgDir, -edgHlf ) )
		);
	}
}


////////// MAIN ////////////////////////////////////////////////////////////////////////////////////

/* ##### DEV PLAN #####
[Y] Move dot around track, 2022-12-24: Slight pause at the top of next loop, don't care
[Y] Move perpendicular segment around track w/ smooth roll
 * NEXT: Move contrail around track w/ smooth roll
 * LATER: Shiny contrail shader that fades to tail 
*/

void main(){
    rnd = Random( unpredictableSeed );
	

    // call this before using raylib
	validateRaylibBinding();

	// Create path
	float /*--*/ unitJump    = 5.0f;
    float /*--*/ unitNoise   = 1.0f;
    Vector3 /**/ noiseBounds = Vector3( unitNoise, unitNoise, unitNoise );
	const ushort N = 6;
	Edge3f*[][N] jumps;
	Edge3f*[][N] rails;
	RailTrail[N] ribbons;

    // Edge3f*[] edges /*-*/ = random_loop_noisy_grid( Vector3( 0.0, 0.0, 0.0 ), unitJump, noiseBounds, 0.25 );
	// Edge3f*[] rail /*--*/ = spline_loop_from_edge_loop( edges, 10 );

	for( ushort i = 0; i < N; i++ ){
		jumps[i] = random_loop_noisy_grid( Vector3( 
			uniform( -5.0, 5.0, rnd ), 
			uniform( -5.0, 5.0, rnd ), 
			uniform( -5.0, 5.0, rnd ) 
		), unitJump, noiseBounds, 0.25 );
		while( jumps[i].length < 10 ){
			jumps[i] = random_loop_noisy_grid( Vector3( 
				uniform( -5.0, 5.0, rnd ), 
				uniform( -5.0, 5.0, rnd ), 
				uniform( -5.0, 5.0, rnd ) 
			), unitJump, noiseBounds, 0.25 );
		}
		rails[i] = spline_loop_from_edge_loop( jumps[i], 10 );
	}

	// Create self-chasing tail
	ribbons[0] = new RailTrail(
		rails[0], 8.0, 30, Colors.RED, 
		1.0f, 0.0f, defaultTurn
	); 

	ribbons[1] = new RailTrail(
		rails[1], 8.0, 30, Colors.YELLOW, 
		1.0f, 0.0f, defaultTurn
	); 

	ribbons[2] = new RailTrail(
		rails[2], 8.0, 30, Colors.BLUE, 
		1.0f, 0.0f, defaultTurn
	); 

	ribbons[3] = new RailTrail(
		rails[3], 8.0, 30, Colors.ORANGE, 
		1.0f, 0.0f, defaultTurn
	); 

	ribbons[4] = new RailTrail(
		rails[4], 8.0, 30, Colors.GREEN, 
		1.0f, 0.0f, defaultTurn
	); 

	ribbons[5] = new RailTrail(
		rails[5], 8.0, 30, Colors.PURPLE, 
		1.0f, 0.0f, defaultTurn
	); 

    // Window / Display Params
    InitWindow( 1320, 990, "Chasing Ribbons" );
    SetTargetFPS( 60 );
    rlDisableBackfaceCulling();
    rlEnableSmoothLines();

    // Camera
    Camera camera = Camera(
        Vector3(50.0, 0.0, 0.0), 
        Vector3(0.0, 0.0, 0.0), 
        Vector3(0.0, 1.0, 0.0), 
        45.0, 
        0
    );

    while ( !WindowShouldClose() ){
		BeginDrawing();
		// ClearBackground( Colors.RAYWHITE );
		ClearBackground( Colors.BLACK );

        BeginMode3D( camera );
        
        
		// trail.advance_by_one_rail_segment();
		// trail.draw_segments();
		foreach( RailTrail ribbon; ribbons ){
			ribbon.advance_by_one_rail_segment();
			ribbon.draw_segments();
		}


        EndMode3D();

		// DrawText("Hello, World!", 400, 300, 28, Colors.BLACK);
		EndDrawing();

		// count++;
	}
	CloseWindow();

    // writeln( rail.length );
}