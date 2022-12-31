import std.stdio;

import raylib;


struct SpriteRL{
	// Convenience struct for displaying and moving a sprite
	Texture2D image; // Sprite image
	uint /**/ cenX; //- X Offset from upper left in the image frame
	uint /**/ cenY; //- Y Offset from upper left in the image frame
	float     rot_deg; // Rotation of the sprite in degrees

	void load( string imagePath ){
		// FIXME, START HERE: LOAD THE SPRITE
	}
}


void main(){


	

	///// Init ///////////////////////////////////

	// call this before using raylib
	validateRaylibBinding();


	



	///// Rendering //////////////////////////////

	// Window / Display Params
    // InitWindow(1200, 900, "Hello, Raylib-D!");
    InitWindow(600, 600, "Hello, Raylib-D!");
    SetTargetFPS(60);


	///// Load ///////////////////////////////////
	// Textured MUST be loaded AFTER the window is initialized
	Texture2D tankBody = LoadTexture( "Resources/PNG/Hulls_Color_B/Hull_08.png" );

	while ( !WindowShouldClose() ){

		/// Rendering ///
		BeginDrawing();

		ClearBackground( Colors.DARKBLUE );

		DrawTextureEx( tankBody, Vector2(300,300), 90, 1.0f, Colors.RAYWHITE );

		
		EndDrawing();

	}

	CloseWindow();

}
