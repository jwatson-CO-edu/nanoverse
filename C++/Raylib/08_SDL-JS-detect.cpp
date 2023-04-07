/*
Compile Command:
g++ 08_SDL-JS-detect.cpp -lSDL2main -lSDL2

https://lazyfoo.net/tutorials/SDL/19_gamepads_and_joysticks/index.php
*/

////////// INIT ////////////////////////////////////////////////////////////////////////////////////

#include <iostream>
using std::cout, std::endl;
#include <SDL2/SDL.h> 

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

    //Analog joystick dead zone
    const int JOYSTICK_DEAD_ZONE = 8000;

    //Normalized direction
    int xDir = 0;
    int yDir = 0;

    //Handle events on queue
    while( SDL_PollEvent( &e ) != 0 ){
        // if Motion on controller 0
        if( (e.type == SDL_JOYAXISMOTION) && (e.jaxis.which == 0) ){
            
            //X axis motion
            if( e.jaxis.axis == 0 ){
                //Left of dead zone
                if( e.jaxis.value < -JOYSTICK_DEAD_ZONE ){
                    xDir = -1;
                }
                //Right of dead zone
                else if( e.jaxis.value > JOYSTICK_DEAD_ZONE ){
                    xDir = 1;
                }else{
                    xDir = 0;
                }
            }
            //Y axis motion
            else if( e.jaxis.axis == 1 ){
                //Below of dead zone
                if( e.jaxis.value < -JOYSTICK_DEAD_ZONE ){
                    yDir = -1;
                }
                //Above of dead zone
                else if( e.jaxis.value > JOYSTICK_DEAD_ZONE ){
                    yDir =  1;
                }else{
                    yDir = 0;
                }
            }
        }
    }

    
}


////////// MAIN ////////////////////////////////////////////////////////////////////////////////////

int main(){

    //Game Controller 1 handler
    SDL_Joystick* js0ptr = NULL;

    cout << "SDL Init: " << (SDL_Init( SDL_INIT_JOYSTICK ) == 0) << endl; // Req'd for joystick detection
    
    js0ptr = init_js0_SDL2();

    // https://lazyfoo.net/tutorials/SDL/19_gamepads_and_joysticks/index.php
     //Main loop flag
    

    

    //Close game controller
    close_js_SDL2( js0ptr );
    SDL_Quit();
    return 0;
}