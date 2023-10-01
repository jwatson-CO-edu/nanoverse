#ifndef UTILS_H
#define UTILS_H

////////// INIT ////////////////////////////////////////////////////////////////////////////////////

///// Imports ////////////////////////////////////

/// Standard ///
#include <stdlib.h>  // srand, rand
#include <time.h>
#include <iostream>
using std::cout, std::endl, std::ostream, std::flush;
#include <string>
using std::string;
#include <cmath>

///// Aliases ////////////////////////////////////
typedef unsigned char /*---*/ ubyte;
typedef vector<vector<float>> vvf;


////////// RANDOM NUMBERS //////////////////////////////////////////////////////////////////////////

float randf(){
    // Return a pseudo-random number between 0.0 and 1.0
    return  1.0f * rand() / RAND_MAX;
}

float randf( float lo, float hi ){
    // Return a pseudo-random number between `lo` and `hi`
    // NOTE: This function assumes `hi > lo`
    float span = hi - lo;
    return lo + span * randf();
}

int randi( int lo, int hi ){
    // Return a pseudo-random number between `lo` and `hi` (int)
    int span = hi - lo;
    return lo + (rand() % span);
}

ubyte rand_ubyte(){
    // Return a pseudo-random unsigned byte
    return (ubyte) randf( 0.0, 256.0 );
}

void rand_seed(){  srand( time(NULL) );  } // Seed RNG with unpredictable time-based seed

#endif /* UTILS_H */