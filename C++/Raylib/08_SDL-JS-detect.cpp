/*
Compile Command:
g++ 08_SDL-JS-detect.cpp -lSDL2main -lSDL2

https://lazyfoo.net/tutorials/SDL/19_gamepads_and_joysticks/index.php

*/

////////// INIT ////////////////////////////////////////////////////////////////////////////////////

#include <iostream>
using std::cout, std::endl;
#include <SDL2/SDL.h> 

///// Constants //////////////////////////////////

const float JS_AXIS_MAX_VAL    = 32768.0f; // Max analog joystick value range in SDL2
const int   JOYSTICK_DEAD_ZONE =  8000; // -- Analog joystick dead zone

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

void poll_SDL2_inputs( InputState& state ){
    // Populate the `state` based on the SDL2 input queue

    // Handle events in queue
    while( SDL_PollEvent( &e ) != 0 ){
        // if Motion on controller 0
        if( (e.type == SDL_JOYAXISMOTION) && (e.jaxis.which == 0) ){

            // https://wiki.libsdl.org/SDL2/SDL_GameControllerAxis
            switch( e.jaxis.axis ){
                case SDL_CONTROLLER_AXIS_LEFTX: 
                    if( abs( e.jaxis.value ) > JOYSTICK_DEAD_ZONE ){ //Above of dead zone
                        state.GAMEPAD_AXIS_LEFT_X = e.jaxis.value / JS_AXIS_MAX_VAL;
                    }else{ //Below of dead zone
                        state.GAMEPAD_AXIS_LEFT_X = 0.0f;
                    }
                    break;
                case SDL_CONTROLLER_AXIS_LEFTY: 
                    if( abs( e.jaxis.value ) > JOYSTICK_DEAD_ZONE ){ //Above of dead zone
                        state.GAMEPAD_AXIS_LEFT_Y = e.jaxis.value / JS_AXIS_MAX_VAL;
                    }else{ //Below of dead zone
                        state.GAMEPAD_AXIS_LEFT_Y = 0.0f;
                    }
                    break;
                case SDL_CONTROLLER_AXIS_RIGHTX: 
                    if( abs( e.jaxis.value ) > JOYSTICK_DEAD_ZONE ){ //Above of dead zone
                        state.GAMEPAD_AXIS_RIGHT_X = e.jaxis.value / JS_AXIS_MAX_VAL;
                    }else{ //Below of dead zone
                        state.GAMEPAD_AXIS_RIGHT_X = 0.0f;
                    }
                    break;
                case SDL_CONTROLLER_AXIS_RIGHTY: 
                    if( abs( e.jaxis.value ) > JOYSTICK_DEAD_ZONE ){ //Above of dead zone
                        state.GAMEPAD_AXIS_RIGHT_Y = e.jaxis.value / JS_AXIS_MAX_VAL;
                    }else{ //Below of dead zone
                        state.GAMEPAD_AXIS_RIGHT_Y = 0.0f;
                    }
                    break;
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

    //Game Controller 1 handler
    SDL_Joystick* js0ptr = NULL;
    InputState    joyState{};

    cout << "SDL Init: " << (SDL_Init( SDL_INIT_JOYSTICK ) == 0) << endl; // Req'd for joystick detection
    
    js0ptr = init_js0_SDL2();

    
    

    

    //Close game controller
    close_js_SDL2( js0ptr );
    SDL_Quit();
    return 0;
}