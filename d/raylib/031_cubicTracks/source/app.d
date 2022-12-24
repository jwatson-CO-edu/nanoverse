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

Vector3 stochastic_jump( Vector3 start, Vector3 jump, Vector3 noise ){
    // Make a `jump` away from `start` with `noise` added
    Vector3 wiggle = Vector3(
        uniform(-noise.x, noise.x, rnd),
        uniform(-noise.y, noise.y, rnd),
        uniform(-noise.z, noise.z, rnd)
    );
    return Vector3Add( Vector3Add( start, jump ), wiggle );
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


Edge3f*[] random_walk_noisy_grid( Vector3 start, float unitJump, Vector3 noiseBounds, uint jumps ){
    // Do a random, noisy gridlike walk
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
    for( uint i = 0; i < jumps; i++ ){
        currDex = cast(int) uniform( 0, vnJumps.length, rnd );
        while( Vector3LengthSqr( Vector3Add( vnJumps[currDex], vnJumps[lastDex] )  ) == 0.0){
            currDex = cast(int) uniform( 0, vnJumps.length, rnd );
        }
        currPnt = stochastic_jump( lastPnt, vnJumps[ currDex ], noiseBounds );
        currNode = new Node3f( currPnt );
        edges ~= new Edge3f( lastNode, currNode );
        lastPnt  = currPnt;
        lastNode = currNode;
        lastDex  = currDex;
    }
    return edges;
}


Edge3f*[] random_loop_noisy_grid( Vector3 start, float unitJump, Vector3 noiseBounds, float epsilon ){
    // Do a random, noisy gridlike walk
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


struct Arc3f{
    Vector3 center;
    Vector3 normal;
    Vector3 start;
    float   angle_rad; 
}




////////// MAIN ////////////////////////////////////////////////////////////////////////////////////


void main(){
    rnd = Random( unpredictableSeed );
	float   cubeHalfBound = 50.0f;
    float   unitJump      = 5.0f;
    float   unitNoise     = 1.0f;
    Vector3 noiseBounds   = Vector3( unitNoise, unitNoise, unitNoise );
    Vector3[6] vnJumps = [
        Vector3(  unitJump, 0.0f, 0.0f ),
        Vector3( -unitJump, 0.0f, 0.0f ),
        Vector3(  0.0f,  unitJump, 0.0f ),
        Vector3(  0.0f, -unitJump, 0.0f ),
        Vector3(  0.0f, 0.0f,  unitJump ),
        Vector3(  0.0f, 0.0f, -unitJump ),
    ];

    Vector3   lastPnt  = Vector3( 0.0, 0.0, 0.0 );
    Node3f*   lastNode = new Node3f( lastPnt );
    Vector3   currPnt;
    Node3f*   currNode;
    Edge3f*[] edges;

    // for( uint i = 0; i < 50; i++ ){
    //     currPnt = stochastic_jump( lastPnt, vnJumps[ uniform(0, vnJumps.length-1, rnd) ], noiseBounds );
    //     currNode = new Node3f( currPnt );
    //     edges ~= new Edge3f( lastNode, currNode );
    //     lastPnt  = currPnt;
    //     lastNode = currNode;
    // }

    // edges = random_walk_noisy_grid( Vector3( 0.0, 0.0, 0.0 ), unitJump, noiseBounds, 50 );
    edges = random_loop_noisy_grid( Vector3( 0.0, 0.0, 0.0 ), unitJump, noiseBounds, 0.25 );
    while( edges.length < 10 ){
        edges = random_loop_noisy_grid( Vector3( 0.0, 0.0, 0.0 ), unitJump, noiseBounds, 0.25 );
    }

    // call this before using raylib
	validateRaylibBinding();
	
    // Window / Display Params
    InitWindow(1200, 900, "Loop Closer");
    SetTargetFPS(60);
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