import std.stdio;

import raylib;


////////// BRAITENBERG TYPES ///////////////////////////////////////////////////////////////////////

/* Steps //
1. Components receive messages on attachment nodes, in order, so that they know where they came from
2. Components act
3. Components send messages

// Values & Constants //
* `id == 0` is the outside world
* A message with no payload is a dummy message
*/

struct Node_BV{
	// A mount point for one or more `Component_BV`
	Vector2 /*---*/ relPos; //- Relative position on the vehicle
	Component_BV*[] mounted; // Components attached at this node
	Msg_BV[] /*--*/ msgs; // -- Messages to be routed

	void route_messages(){
		// Send message to every other component attached to this node except for the sender
		// 1. For every message for every component, send the message to that component if it is not the sender
		foreach( Msg_BV msg; msgs ){
			foreach( Component_BV* comp; mounted ){
				if( msg.sender != comp.id ){  comp.input ~= msg;  }
			}
		}
		// 2. Flush messages
		msgs = [];
	}
}

struct Msg_BV{
	// Messages passed between components
	ulong /*---*/ sender;
	float[string] payload;
}

Msg_BV dummy_msg(){
	// Send a message from no one
	return Msg_BV( 0 ); // A message with no payload is a dummy message
}

struct Component_BV{
	// Specification of a component
	ulong /*--------------------*/ id; // ----- ID of this particular component
	ushort /*-------------------*/ N_attach; // Number of required attachments for this component to function
	Node_BV[] /*----------------*/ attach; // - Ordered attachment points for this component
	float[string] /*------------*/ params; // - Parameters that govern placement and behavior
	Msg_BV[] /*-----------------*/ input; // -- Message inbox
	static ulong /*-------------*/ ID = 0; // - Total number of IDs created
	void function( Component_BV* ) update; // - Strategy: Per-step actions   for this component
	void function( Component_BV* ) transmit; // Strategy: Per-step messaging for this component
	Vehicle_BV* /*--------------*/ parent; // - The vehicle that this component belongs to

	void step(){  update( &this );    } // `update` this component
	void send(){  transmit( &this );  } // `transmit` from this component
}

Component_BV get_component(){
	// Create a component with a unique ID
	Component_BV.ID++;
	return Component_BV( Component_BV.ID );
}

struct Vehicle_BV{
	Vector2 /*---*/ position; // -- <x,y> position of the vehicle center
	float /*-----*/ orientation; // Absolute orientation of the vehicle
	Node_BV*[] /**/ nodes; // ----- Attachment points for components
	Component_BV*[] comps; // ----- Components that provide updates

	void step(){
		// 1. Route messages
		foreach( Node_BV* node; nodes ){  node.route_messages();  }
		// 2. Update parts
		foreach( Component_BV* comp; comps ){  comp.step();  }
		// 3. Send messages
		foreach( Component_BV* comp; comps ){  comp.send();  }
	}

	void attach_comp( Component_BV* comp ){  comps ~= comp;  } // Attach a `Component_BV` for updating
	void attach_node( Node_BV* node ){  nodes ~= node;  } // Attach a `Node_BV` for updating
}

struct Prop_BV{
	// A thing that is not a vehicle, like a light source
}


struct World_BV{
	// Contains all vehicles and the things that they interact with
}


////////// BRAITENBERG OBJECTS /////////////////////////////////////////////////////////////////////





void main(){
	

	///// Init ///////////////////////////////////

	// call this before using raylib
	validateRaylibBinding();

	float angle = 0.0;

	// Rectangle r = Rectangle( 0, 0, 100, 200 );


	///// Rendering //////////////////////////////

	// Window / Display Params
    // InitWindow(1200, 900, "Hello, Raylib-D!");
    InitWindow( 600, 600, "Hello, Raylib-D!" );
    SetTargetFPS(60);


	///// Load ///////////////////////////////////
	// Textured MUST be loaded AFTER the window is initialized


	while ( !WindowShouldClose() ){

		/// Rendering ///
		BeginDrawing();

		ClearBackground( Colors.RAYWHITE );

		////////// GAME LOGIC //////////////////////////////////////////////////////////////////////

		angle += 1.5;

		// rlTranslatef( -100, -50, 0 );
		rlTranslatef( 300, 300, 0 );
		rlRotatef( angle, 0,0,-1 );
		

		// DrawRectanglePro( r, Vector2( 0, 0 ), 0.0, Colors.RED);
		DrawRectangle( -50, -100, 100, 200, Colors.RED );   


		EndDrawing();

	}

	CloseWindow();

}