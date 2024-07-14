////////// INIT ////////////////////////////////////////////////////////////////////////////////////
#pragma GCC diagnostic ignored "-Wunused-result"

///// Includes ////////////////////////////////////////////////////////////

/// Standard ////
#include <stdio.h>

///// Type Defines ////////////////////////////////////////////////////////

typedef unsigned char ubyte;
typedef unsigned int  uint;
typedef unsigned long ulong;



////////// HELPER FUNCTIONS ////////////////////////////////////////////////////////////////////////
#define _EPSILON 0.000001

// Return 1 if `op1` and `op2` are nearly equal
inline ubyte equal_dbbl( double op1, double op2 ){  return ((op1 - op2) <= _EPSILON);  }


////////// MULTIVECTOR STRUCTS /////////////////////////////////////////////////////////////////////

typedef struct{
    // Multivector for a 3D space
    double ps; // - Pseudoscalar
    double e1; // - X component
    double e2; // - Y component
    double e3; // - Z component
    double e12; //- XY bivector
    double e13; //- XZ bivector
    double e23; //- YZ bivector
    double e123; // XYZ oriented volume
}mv3;



////////// MULTIVECTOR OPERATIONS //////////////////////////////////////////////////////////////////


///// Algebraic Ops ///////////////////////////////////////////////////////

mv3 wedge_mv3( mv3 op1, mv3 op2 ){
    // Compute the wedge product of 3D multivectors `op1` and `op2` 
    mv3 wdgProd = {
        // 0-blade //
        0.0, // op1.ps * op2.ps
        // 1-blades //
        0.0, // op1.ps * op2.e1 + op2.ps * op1.e1,
        0.0, // op1.ps * op2.e2 + op2.ps * op1.e2,
        0.0, // op1.ps * op2.e3 + op2.ps * op1.e3,
        // 2-blades //
        op1.e1 * op2.e2 + op2.e1 * op1.e2,
        op1.e1 * op2.e3 + op2.e1 * op1.e3,
        op1.e2 * op2.e3 + op2.e2 * op1.e3,
        // 3-blade //
        op1.e1 * op2.e23 + op2.e1 * op1.e23  +  
            op1.e2 * op2.e13 + op2.e2 * op1.e13  +  
            op1.e3 * op2.e12 + op2.e3 * op1.e12  + 
            op1.e123 * op2.e123
    };
    return wdgProd;
}


///// Multivector I/O /////////////////////////////////////////////////////

void print_mv3( mv3 mltVec ){
    // Display the nonzero elements of the 3D multivector
    printf( "[" );
    ubyte prv = 0;
    if( !equal_dbbl( mltVec.ps, 0.0 ) ){  printf( "(%.4lf)", mltVec.ps );  prv = 1;   }
    if( !equal_dbbl( mltVec.e1, 0.0 ) ){  printf( "%s(%.4lf)e1", (prv?", ":""), mltVec.e1 );  prv = 1;     }
    if( !equal_dbbl( mltVec.e2, 0.0 ) ){  printf( "%s(%.4lf)e2", (prv?", ":""), mltVec.e2 );  prv = 1;     }
    if( !equal_dbbl( mltVec.e3, 0.0 ) ){  printf( "%s(%.4lf)e3", (prv?", ":""), mltVec.e3 );  prv = 1;     }
    if( !equal_dbbl( mltVec.e12, 0.0 ) ){  printf( "%s(%.4lf)e12", (prv?", ":""), mltVec.e12 );  prv = 1;  }
    if( !equal_dbbl( mltVec.e13, 0.0 ) ){  printf( "%s(%.4lf)e13", (prv?", ":""), mltVec.e13 );  prv = 1;  }
    if( !equal_dbbl( mltVec.e23, 0.0 ) ){  printf( "%s(%.4lf)e23", (prv?", ":""), mltVec.e23 );  prv = 1;  }
    if( !equal_dbbl( mltVec.e123, 0.0 ) ){  printf( "%s(%.4lf)e123", (prv?", ":""), mltVec.e123 );         }
    printf( "]" );
}


mv3 read_mv3( void ){
    mv3 mltVec = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    int Nreads;
    Nreads = scanf( 
        "%lf %lf %lf %lf %lf %lf %lf %lf",
        &mltVec.ps, //- Pseudoscalar
        &mltVec.e1, //- X component
        &mltVec.e2, //- Y component
        &mltVec.e3, //- Z component
        &mltVec.e12, // XY bivector
        &mltVec.e13, // XZ bivector
        &mltVec.e23, // YZ bivector
        &mltVec.e123 // XYZ oriented volume
    );
    if( Nreads != 8 ){  printf( "`read_mv3` WARNING: Read %i elements instead of 8\n", Nreads );  }
    return mltVec;
}



////////// MAIN ////////////////////////////////////////////////////////////////////////////////////
int main( int argc, char* argv[] ){
    mv3   op1;
    mv3   op2;
    mv3   ans;
    char  option;
    int   Nread;
    while( 1 ){

        printf( "{[w] for wedge product, [q] to quit}, Then [Enter]\nMultivector elements should be separated by a space.\n\n" );
        Nread = scanf( "%c", &option );
        if( Nread != 1 ){  break;  }
        scanf("%*c"); //consume the line break
        if( option != 'w' ){  break;  }

        printf( "\nEnter 3D multivector `A`:  " );
        op1 = read_mv3();
        scanf("%*c"); //consume the line break

        printf( "Enter 3D multivector `B`:  " );
        op2 = read_mv3();
        scanf("%*c"); //consume the line break

        ans = wedge_mv3( op1, op2 );
        printf( "A^B = " );
        print_mv3( ans );
        printf( "\n\n\n" );
    }
}