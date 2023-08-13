/*
Compile Command:
g++ 08_SDL-JS-detect.cpp -lSDL2main -lSDL2 -lraylib

https://lazyfoo.net/tutorials/SDL/19_gamepads_and_joysticks/index.php
LINUX NOTE: https://github.com/gabomdq/SDL_GameControllerDB/blob/master/gamecontrollerdb.txt
*/

////////// INIT ////////////////////////////////////////////////////////////////////////////////////

#include <iostream>
using std::cout, std::endl;
#include <stdlib.h>
#include <string>
using std::string;
// #include <SDL2/SDL.h> 
// #define SDL_MAIN_HANDLED
#include <raylib.h>
#include <rlgl.h>


///// Constants //////////////////////////////////

const float  JS_AXIS_MAX_VAL    = 32768.0f; // Max analog joystick value range in SDL2
const int    JOYSTICK_DEAD_ZONE =  4000; // -- Analog joystick dead zone
const string controllerProfiles = "controllerDB.txt";



////////// MAIN ////////////////////////////////////////////////////////////////////////////////////

int main(){

    char* udevFlag = (char*) string( "SDL_JOYSTICK_DISABLE_UDEV=1" ).c_str();
    putenv( udevFlag );


    /// Window Init ///
    InitWindow( 900, 500, "Gamepad Test" );
    SetTargetFPS( 60 );
    rlEnableSmoothLines();

    SetGamepadMappings( controllerProfiles.c_str() );

    cout << "Found EasySMX: " << IsGamepadAvailable(0) << endl; // NOT FOUND
    // cout << "Found EasySMX: " << GetGamepadName(0) << endl;

    int   cntr_LS[2]    = {300, 250};
    int   cntr_RS[2]    = {600, 250};
    int   stickHalfSpan = 50;
    float stickRad /**/ = 25.0f;
    
    while( !WindowShouldClose() ){

        
        /// Begin Drawing ///
        BeginDrawing();
        ClearBackground( RAYWHITE );

        DrawRectangleLines(
            (int) cntr_LS[0] - stickHalfSpan, 
            (int) cntr_LS[1] - stickHalfSpan, 
            2 * stickHalfSpan, 
            2 * stickHalfSpan, 
            BLACK
        );
        // DrawCircle(
        //     (int) cntr_LS[0] + stickHalfSpan * joyState.GAMEPAD_AXIS_LEFT_X, 
        //     (int) cntr_LS[1] + stickHalfSpan * joyState.GAMEPAD_AXIS_LEFT_Y, 
        //     stickRad, 
        //     BLACK
        // ); 

        DrawRectangleLines(
            (int) cntr_RS[0] - stickHalfSpan, 
            (int) cntr_RS[1] - stickHalfSpan, 
            2 * stickHalfSpan, 
            2 * stickHalfSpan, 
            BLACK
        );
        // DrawCircle(
        //     (int) cntr_RS[0] + stickHalfSpan * joyState.GAMEPAD_AXIS_RIGHT_X, 
        //     (int) cntr_RS[1] + stickHalfSpan * joyState.GAMEPAD_AXIS_RIGHT_Y, 
        //     stickRad, 
        //     BLACK
        // ); 

        EndDrawing();
    }

    
    return 0;
}