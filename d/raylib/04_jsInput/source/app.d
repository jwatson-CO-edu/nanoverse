////////// INIT ////////////////////////////////////////////////////////////////////////////////////

import core.stdc.stdlib; // malloc
import raylib; // --------- easy graphics
import std.stdio; // ------ writeline
import std.conv; // ------- Conversions
import std.string; // ----- toStringz

////////// MAIN ////////////////////////////////////////////////////////////////////////////////////


void main(){
	// call this before using raylib
	validateRaylibBinding();

	// Window / Display Params
    InitWindow(800, 600, "Hello, Raylib-D!");
    SetTargetFPS(60);

	/// Input Devices ///
	bool /*---*/ GP0_found  = false;
	bool /*---*/ GP1_found  = false;
	bool /*---*/ GP2_found  = false;
	const(char)* gpMsg0     = cast(char*) new char[128];
	const(char)* gpMsg1     = cast(char*) new char[128];
	const(char)* gpMsg2     = cast(char*) new char[128];

	while ( !WindowShouldClose() ){

		/// Rendering ///
		BeginDrawing();

		// Check Inputs // 
		GP0_found = IsGamepadAvailable(0);
	    GP1_found = IsGamepadAvailable(1);
		GP2_found = IsGamepadAvailable(2);
		gpMsg0    = toStringz( "Gamepad 0 Available?: " ~ to!string( GP0_found ) ); // Test if gamepad 0 was found
		gpMsg1    = toStringz( "Gamepad 1 Available?: " ~ to!string( GP1_found ) ); // Test if gamepad 1 was found
		gpMsg2    = toStringz( "Gamepad 2 Available?: " ~ to!string( GP2_found ) ); // Test if gamepad 2 was found

		ClearBackground( Colors.RAYWHITE );

		// writeln( GetGamepadAxisMovement(0, GamepadAxis.GAMEPAD_AXIS_RIGHT_X) );
		writeln( GetGamepadAxisCount(0) ); // 6

		// Right Stick
		DrawRectangleLines( 100, 200, 200, 200, Colors.BLACK );
		DrawCircle( cast(int)(200 + GetGamepadAxisMovement(0, GamepadAxis.GAMEPAD_AXIS_RIGHT_X)*200),
                    cast(int)(300 + GetGamepadAxisMovement(0, GamepadAxis.GAMEPAD_AXIS_RIGHT_Y)*200), 
					12.5, Colors.BLUE);

		// Left Stick
		DrawRectangleLines( 500, 200, 200, 200, Colors.BLACK );
		DrawCircle( cast(int)(600 + GetGamepadAxisMovement(0, GamepadAxis.GAMEPAD_AXIS_LEFT_X)*200),
                    cast(int)(300 + GetGamepadAxisMovement(0, GamepadAxis.GAMEPAD_AXIS_LEFT_Y)*200), 
					12.5, Colors.BLUE);

		
		DrawText( gpMsg0, 25, 25, 28, Colors.BLACK );
		DrawText( gpMsg1, 25, 50, 28, Colors.BLACK );
		DrawText( gpMsg2, 25, 75, 28, Colors.BLACK );

		EndDrawing();

	}

	CloseWindow();
}