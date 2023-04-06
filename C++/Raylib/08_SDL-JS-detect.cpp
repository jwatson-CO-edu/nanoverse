/*
Compile Command:
g++ 08_SDL-JS-detect.cpp -lSDL2main -lSDL2

https://lazyfoo.net/tutorials/SDL/19_gamepads_and_joysticks/index.php
*/

#include <iostream>
using std::cout, std::endl;
#include <SDL2/SDL.h> 

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

////////// MAIN ////////////////////////////////////////////////////////////////////////////////////

int main(){

    //Game Controller 1 handler
    SDL_Joystick* js0ptr = NULL;

    cout << "SDL Init: " << (SDL_Init( SDL_INIT_JOYSTICK ) == 0) << endl; // Req'd for joystick detection
    
    js0ptr = init_js0_SDL2();

    // FIXME, START HERE: ADD EVENT LOOP FROM https://lazyfoo.net/tutorials/SDL/19_gamepads_and_joysticks/index.php

    //Close game controller
    close_js_SDL2( js0ptr );
    SDL_Quit();
    return 0;
}