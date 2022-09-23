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
	const(char)* gpMsg      =  toStringz( "Gamepad 0 Available?: " ~ to!string( GP0_found ) ); // Test if gamepad 0 was found

	while (!WindowShouldClose()){

		/// Rendering ///

		BeginDrawing();
		ClearBackground(Colors.RAYWHITE);

		DrawRectangleLines(100, 200, 200, 200, Colors.BLACK);
		DrawRectangleLines(500, 200, 200, 200, Colors.BLACK);

		DrawCircle(200, 300, 12.5, Colors.BLUE);
		DrawCircle(600, 300, 12.5, Colors.BLUE);

		
		DrawText( gpMsg, 25, 25, 28, Colors.BLACK);

		EndDrawing();

	}

	CloseWindow();
}