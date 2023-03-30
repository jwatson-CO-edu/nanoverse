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

		// Right Stick
		DrawRectangleLines( 400, 200, 100, 100, Colors.BLACK );
		DrawCircle( cast(int)(450 + GetGamepadAxisMovement(0, GamepadAxis.GAMEPAD_AXIS_RIGHT_X)*50),
                    cast(int)(250 + GetGamepadAxisMovement(0, GamepadAxis.GAMEPAD_AXIS_RIGHT_Y)*50), 
					12.5, Colors.BLUE);

		// Left Stick
		DrawRectangleLines( 250, 200, 100, 100, Colors.BLACK );
		DrawCircle( cast(int)(300 + GetGamepadAxisMovement(0, GamepadAxis.GAMEPAD_AXIS_LEFT_X)*50),
                    cast(int)(250 + GetGamepadAxisMovement(0, GamepadAxis.GAMEPAD_AXIS_LEFT_Y)*50), 
					12.5, Colors.BLUE);

		// Face Buttons
		DrawCircle( 650, 200, 25, IsGamepadButtonDown(0, GamepadButton.GAMEPAD_BUTTON_RIGHT_FACE_UP) ? 
			Colors.GREEN : Colors.RED);
		DrawCircle( 600, 250, 25, IsGamepadButtonDown(0, GamepadButton.GAMEPAD_BUTTON_RIGHT_FACE_LEFT) ? 
			Colors.GREEN : Colors.RED);
		DrawCircle( 700, 250, 25, IsGamepadButtonDown(0, GamepadButton.GAMEPAD_BUTTON_RIGHT_FACE_RIGHT) ? 
			Colors.GREEN : Colors.RED);
		DrawCircle( 650, 300, 25, IsGamepadButtonDown(0, GamepadButton.GAMEPAD_BUTTON_RIGHT_FACE_DOWN) ? 
			Colors.GREEN : Colors.RED);

		// D-Pad
		DrawRectangle( 100, 175, 25, 50, IsGamepadButtonDown(0, GamepadButton.GAMEPAD_BUTTON_LEFT_FACE_UP) ? 
			Colors.GREEN : Colors.BLACK);
		DrawRectangle(  37, 238, 50, 25, IsGamepadButtonDown(0, GamepadButton.GAMEPAD_BUTTON_LEFT_FACE_LEFT) ? 
			Colors.GREEN : Colors.BLACK);
		DrawRectangle( 138, 238, 50, 25, IsGamepadButtonDown(0, GamepadButton.GAMEPAD_BUTTON_LEFT_FACE_RIGHT) ? 
			Colors.GREEN : Colors.BLACK);
		DrawRectangle( 100, 275, 25, 50, IsGamepadButtonDown(0, GamepadButton.GAMEPAD_BUTTON_LEFT_FACE_DOWN) ? 
			Colors.GREEN : Colors.BLACK);
		
		

		// Gamepads detected
		DrawText( gpMsg0, 25, 25, 28, Colors.BLACK );
		DrawText( gpMsg1, 25, 50, 28, Colors.BLACK );
		DrawText( gpMsg2, 25, 75, 28, Colors.BLACK );

		EndDrawing();

	}

	CloseWindow();
}