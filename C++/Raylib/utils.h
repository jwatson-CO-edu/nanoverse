#ifndef UTILS_H
#define UTILS_H

#include <stdlib.h>  // srand, rand
#include <time.h>

////////// UNIFORM RANDOM SAMPLING /////////////////////////////////////////////////////////////////


float randf(){
    // Return a pseudo-random number between 0.0 and 1.0
    return  1.0f * rand() / RAND_MAX;
}

float randf( float lo, float hi ){
    // Return a pseudo-random number between `lo` and `hi` (float)
    float span = hi - lo;
    return lo + randf() * span;
}

int randi( int lo, int hi ){
    // Return a pseudo-random number between `lo` and `hi` (int)
    int span = hi - lo;
    return lo + (rand() % span);
}

void rand_seed(){  srand( time(NULL) );  } // Seed RNG with unpredictable time-based seed

#endif /* UTILS_H */
