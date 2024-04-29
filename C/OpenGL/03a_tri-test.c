// gcc -std=gnu17 -O3 -Wall 03a_tri-test.c -lglut -lGLU -lGL -lm -o triTest.out


////////// INIT ////////////////////////////////////////////////////////////////////////////////////

#include "TriNet.h"



////////// MAIN ////////////////////////////////////////////////////////////////////////////////////

vec2f p0 = { 0.0f, 0.0f };
vec2f p1 = { 1.0f, 0.0f };
vec2f p2 = { 0.0f, 1.0f };

vec2f q = { 0.333f, 0.333f };

vec2f d = { -0.1f, 0.0f };

void test_inside(){
    print_vec2f( q );
    printf( 
        " [%i,%i,%i]\n", 
        (int) p_pnt_positive_angle_from_seg( &q, &p0, &p1 ),
        (int) p_pnt_positive_angle_from_seg( &q, &p1, &p2 ),
        (int) p_pnt_positive_angle_from_seg( &q, &p2, &p0 )
    );
}

int main(){
    test_inside();
    for( ubyte i = 0; i < 4; ++i ){
        sub_vec2f( &q, &q, &d );
        test_inside();
    }
}
