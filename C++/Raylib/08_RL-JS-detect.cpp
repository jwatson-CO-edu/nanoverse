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

const float JS_AXIS_MAX_VAL    = 32768.0f; // Max analog joystick value range in SDL2
const int   JOYSTICK_DEAD_ZONE =  4000; // -- Analog joystick dead zone

///// Globals ////////////////////////////////////

//Event handler
SDL_Event e;

////////// JOYSTICK @ SDL2 /////////////////////////////////////////////////////////////////////////

SDL_Joystick* init_js0_SDL2(){
    // Open Joystick/Gamepad 0 and return a pointer to it
    
    SDL_Joystick* rtnPtr = nullptr;
    int /*-----*/ N_js   = SDL_NumJoysticks();
    cout << "Attached joysticks: " << N_js << endl; // Detects the GameSir JS that Raylib IGNORES!
    if( N_js > 0 ){
        rtnPtr = SDL_JoystickOpen( 0 );
        // rtnPtr = SDL_SYS_JoystickOpen( 0 );
        // rtnPtr->bXInputDevice = SDL_FALSE;
        if( rtnPtr == nullptr ){
            cout << "Warning: Unable to open game controller! SDL Error:\n" << SDL_GetError() << endl;
        }else{
            cout << "Connected to joystick!" << endl;
        }
    }
    return rtnPtr;
}

void close_js_SDL2( SDL_Joystick*& jsPtr ){
    // Close the joystick connected to `jsPtr` and set the pointer to null
    SDL_JoystickClose( jsPtr );
    jsPtr = nullptr;
    cout << "Connection to joystick closed!" << endl;
}

///// Input Struct ////////////////////////////////////

struct InputState{
    // Container struct for input state

    // Thumb Sticks
    float GAMEPAD_AXIS_RIGHT_X;
    float GAMEPAD_AXIS_RIGHT_Y;
    float GAMEPAD_AXIS_LEFT_X;
    float GAMEPAD_AXIS_LEFT_Y;
    
    // Face buttons
    bool GAMEPAD_BUTTON_RIGHT_FACE_UP;
    bool GAMEPAD_BUTTON_RIGHT_FACE_LEFT;
    bool GAMEPAD_BUTTON_RIGHT_FACE_RIGHT;
    bool GAMEPAD_BUTTON_RIGHT_FACE_DOWN;

    // D-pad
    bool GAMEPAD_BUTTON_LEFT_FACE_UP;
    bool GAMEPAD_BUTTON_LEFT_FACE_LEFT;
    bool GAMEPAD_BUTTON_LEFT_FACE_RIGHT;
    bool GAMEPAD_BUTTON_LEFT_FACE_DOWN;
};

void poll_SDL2_inputs( InputState& state, SDL_Joystick* jsPtr ){
    // Populate the `state` based on the SDL2 input queue

    // SDL_JoystickUpdate();

    // Handle events in queue
    while( SDL_PollEvent( &e ) != 0 ){
        // if Motion on controller 0
        if( (e.type == SDL_JOYAXISMOTION) ){
        // if( (e.type == SDL_CONTROLLERAXISMOTION) ){

            // https://wiki.libsdl.org/SDL2/SDL_GameControllerAxis
            // switch( e.jaxis.axis ){
            switch( e.caxis.axis ){
                case SDL_CONTROLLER_AXIS_LEFTX: 
                    if( abs( e.caxis.value ) > JOYSTICK_DEAD_ZONE ){ //Above of dead zone
                        state.GAMEPAD_AXIS_LEFT_X = e.caxis.value / JS_AXIS_MAX_VAL;
                    }else{ //Below of dead zone
                        state.GAMEPAD_AXIS_LEFT_X = 0.0f;
                    }
                    break;
                case SDL_CONTROLLER_AXIS_LEFTY: 
                    if( abs( e.caxis.value ) > JOYSTICK_DEAD_ZONE ){ //Above of dead zone
                        state.GAMEPAD_AXIS_LEFT_Y = e.caxis.value / JS_AXIS_MAX_VAL;
                    }else{ //Below of dead zone
                        state.GAMEPAD_AXIS_LEFT_Y = 0.0f;
                    }
                    break;
                // case SDL_CONTROLLER_AXIS_RIGHTX:
                case 2:
                    state.GAMEPAD_AXIS_RIGHT_X = SDL_JoystickGetAxis(
                        jsPtr, 
                        SDL_GameControllerAxis::SDL_CONTROLLER_AXIS_RIGHTX
                    ) / JS_AXIS_MAX_VAL;
                    break;
                // case SDL_CONTROLLER_AXIS_RIGHTY: 
                case 3: 
                    state.GAMEPAD_AXIS_RIGHT_Y = SDL_JoystickGetAxis(
                        jsPtr, 
                        SDL_GameControllerAxis::SDL_CONTROLLER_AXIS_RIGHTY
                    ) / JS_AXIS_MAX_VAL;
                    break;
                // case SDL_CONTROLLER_AXIS_RIGHTX: 
                //     if( abs( e.jaxis.value ) > JOYSTICK_DEAD_ZONE ){ //Above of dead zone
                //         state.GAMEPAD_AXIS_RIGHT_X = e.jaxis.value / JS_AXIS_MAX_VAL;
                //     }else{ //Below of dead zone
                //         state.GAMEPAD_AXIS_RIGHT_X = 0.0f;
                //     }
                //     break;
                // case SDL_CONTROLLER_AXIS_RIGHTY: 
                //     if( abs( e.jaxis.value ) > JOYSTICK_DEAD_ZONE ){ //Above of dead zone
                //         state.GAMEPAD_AXIS_RIGHT_Y = e.jaxis.value / JS_AXIS_MAX_VAL;
                //     }else{ //Below of dead zone
                //         state.GAMEPAD_AXIS_RIGHT_Y = 0.0f;
                //     }
                //     break;
                default:
                    break;
            }

        // if button down on controller 0, Set button flag true
        }else if( e.type == SDL_CONTROLLERBUTTONDOWN ){

            // https://wiki.libsdl.org/SDL2/SDL_GameControllerButton
            switch( e.button.button ){
                case SDL_CONTROLLER_BUTTON_DPAD_UP:
                    state.GAMEPAD_BUTTON_LEFT_FACE_UP = true;
                    break;
                case SDL_CONTROLLER_BUTTON_DPAD_DOWN:
                    state.GAMEPAD_BUTTON_LEFT_FACE_DOWN = true;
                    break;
                case SDL_CONTROLLER_BUTTON_DPAD_LEFT:
                    state.GAMEPAD_BUTTON_LEFT_FACE_LEFT = true;
                    break;
                case SDL_CONTROLLER_BUTTON_DPAD_RIGHT:
                    state.GAMEPAD_BUTTON_LEFT_FACE_RIGHT = true;
                    break;
                case SDL_CONTROLLER_BUTTON_A:
                    state.GAMEPAD_BUTTON_RIGHT_FACE_DOWN = true;
                    break;
                case SDL_CONTROLLER_BUTTON_B:
                    state.GAMEPAD_BUTTON_RIGHT_FACE_RIGHT = true;
                    break;
                case SDL_CONTROLLER_BUTTON_X:
                    state.GAMEPAD_BUTTON_RIGHT_FACE_LEFT = true;
                    break;
                case SDL_CONTROLLER_BUTTON_Y:
                    state.GAMEPAD_BUTTON_RIGHT_FACE_UP = true;
                    break;
                default:
                    break;
            }

        }else if( e.type == SDL_CONTROLLERBUTTONUP ){

            // https://wiki.libsdl.org/SDL2/SDL_GameControllerButton
            switch( e.button.button ){
                case SDL_CONTROLLER_BUTTON_DPAD_UP:
                    state.GAMEPAD_BUTTON_LEFT_FACE_UP = false;
                    break;
                case SDL_CONTROLLER_BUTTON_DPAD_DOWN:
                    state.GAMEPAD_BUTTON_LEFT_FACE_DOWN = false;
                    break;
                case SDL_CONTROLLER_BUTTON_DPAD_LEFT:
                    state.GAMEPAD_BUTTON_LEFT_FACE_LEFT = false;
                    break;
                case SDL_CONTROLLER_BUTTON_DPAD_RIGHT:
                    state.GAMEPAD_BUTTON_LEFT_FACE_RIGHT = false;
                    break;
                case SDL_CONTROLLER_BUTTON_A:
                    state.GAMEPAD_BUTTON_RIGHT_FACE_DOWN = false;
                    break;
                case SDL_CONTROLLER_BUTTON_B:
                    state.GAMEPAD_BUTTON_RIGHT_FACE_RIGHT = false;
                    break;
                case SDL_CONTROLLER_BUTTON_X:
                    state.GAMEPAD_BUTTON_RIGHT_FACE_LEFT = false;
                    break;
                case SDL_CONTROLLER_BUTTON_Y:
                    state.GAMEPAD_BUTTON_RIGHT_FACE_UP = false;
                    break;
                default:
                    break;
            }
        }
    }
}


////////// MAIN ////////////////////////////////////////////////////////////////////////////////////

int main(){

    char* udevFlag = (char*) string( "SDL_JOYSTICK_DISABLE_UDEV=1" ).c_str();
    putenv( udevFlag );

    //Game Controller 1 handler
    SDL_Joystick* js0ptr = NULL;
    InputState    joyState{};

    SDL_SetHint(SDL_HINT_JOYSTICK_ALLOW_BACKGROUND_EVENTS,"1");
    // SDL_SetHint(SDL_HINT_LINUX_JOYSTICK_DEADZONES,"0");
    // SDL_SetHint(SDL_HINT_XINPUT_ENABLED,"0");
    // SDL_SetHint(SDL_HINT_XINPUT_USE_OLD_JOYSTICK_MAPPING,"1");
    // SDL_SetHint(SDL_HINT_GAMECONTROLLERTYPE,"Xbox360");
    SDL_SetHint(SDL_HINT_JOYSTICK_HIDAPI_XBOX,"1");
    // SDL_SetHint(SDL_HINT_AUTO_UPDATE_JOYSTICKS,"0");

    // cout << "SDL Init: " << (SDL_Init( SDL_INIT_JOYSTICK ) == 0); // Req'd for joystick detection
    cout << SDL_JoystickEventState( SDL_ENABLE ) << " ";
    cout << "SDL Init: " << (SDL_Init( SDL_INIT_GAMECONTROLLER ) == 0) << endl; // Req'd for joystick detection
    SDL_SetMainReady();

    // cout << "SDL Init: " << (SDL_Init( SDL_INIT_EVERYTHING ) == 0); // Req'd for joystick detection
    
    
    js0ptr = init_js0_SDL2();

    /// Window Init ///
    InitWindow( 900, 500, "Gamepad Test" );
    SetTargetFPS( 60 );
    rlEnableSmoothLines();

    int   cntr_LS[2]    = {300, 250};
    int   cntr_RS[2]    = {600, 250};
    int   stickHalfSpan = 50;
    float stickRad /**/ = 25.0f;
    
    while( !WindowShouldClose() ){

        poll_SDL2_inputs( joyState, js0ptr );
        
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
        DrawCircle(
            (int) cntr_LS[0] + stickHalfSpan * joyState.GAMEPAD_AXIS_LEFT_X, 
            (int) cntr_LS[1] + stickHalfSpan * joyState.GAMEPAD_AXIS_LEFT_Y, 
            stickRad, 
            BLACK
        ); 

        DrawRectangleLines(
            (int) cntr_RS[0] - stickHalfSpan, 
            (int) cntr_RS[1] - stickHalfSpan, 
            2 * stickHalfSpan, 
            2 * stickHalfSpan, 
            BLACK
        );
        DrawCircle(
            (int) cntr_RS[0] + stickHalfSpan * joyState.GAMEPAD_AXIS_RIGHT_X, 
            (int) cntr_RS[1] + stickHalfSpan * joyState.GAMEPAD_AXIS_RIGHT_Y, 
            stickRad, 
            BLACK
        ); 

        EndDrawing();
    }

    

    //Close game controller
    close_js_SDL2( js0ptr );
    SDL_Quit();
    return 0;
}