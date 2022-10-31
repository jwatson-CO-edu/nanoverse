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
	bool         GP0_found  = IsGamepadAvailable(0);
	bool         GP1_found  = IsGamepadAvailable(1);
	bool         GP2_found  = IsGamepadAvailable(2);
	const(char)* gpMsg0     =  toStringz( "Gamepad 0 Available?: " ~ to!string( GP0_found ) ); // Test if gamepad 0 was found
	const(char)* gpMsg1     =  toStringz( "Gamepad 1 Available?: " ~ to!string( GP1_found ) ); // Test if gamepad 1 was found
	const(char)* gpMsg2     =  toStringz( "Gamepad 2 Available?: " ~ to!string( GP2_found ) ); // Test if gamepad 2 was found

	while (!WindowShouldClose()){

		/// Rendering ///

		BeginDrawing();
		ClearBackground(Colors.RAYWHITE);

		DrawRectangleLines(100, 200, 200, 200, Colors.BLACK);
		DrawRectangleLines(500, 200, 200, 200, Colors.BLACK);

		DrawCircle(200, 300, 12.5, Colors.BLUE);
		DrawCircle(600, 300, 12.5, Colors.BLUE);

		
		DrawText( gpMsg0, 25, 25, 28, Colors.BLACK);
		DrawText( gpMsg1, 25, 50, 28, Colors.BLACK);
		DrawText( gpMsg2, 25, 75, 28, Colors.BLACK);

		EndDrawing();

	}

	CloseWindow();
}