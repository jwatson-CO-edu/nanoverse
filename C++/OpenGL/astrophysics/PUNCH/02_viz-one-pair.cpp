////////// INIT ////////////////////////////////////////////////////////////////////////////////////

///// Imports /////////////////////////////////////////////////////////////
/// Standard ///
#include <list>
using std::list;
/// Local ///
#include "../include/PUNCH.hpp"

///// Aliases /////////////////////////////////////////////////////////////
typedef array<vec4f,2> seg4f;
typedef list<seg4f>    lseg4f;



////////// HELPER FUNCTIONS ////////////////////////////////////////////////////////////////////////

lseg4f point_cross( const vec4f pnt, float width_m ){
    // Return segments representing a point
    lseg4f cross;
    float  wHlf = width_m / 2.0;
    // X-cross
    cross.push_back( seg4f{
        pnt + vec4f{ wHlf, 0.0, 0.0, 1.0 },
        pnt - vec4f{ wHlf, 0.0, 0.0, 1.0 }
    } );
    // Y-cross
    cross.push_back( seg4f{
        pnt + vec4f{ 0.0, wHlf, 0.0, 1.0 },
        pnt - vec4f{ 0.0, wHlf, 0.0, 1.0 }
    } );
    // Z-cross
    cross.push_back( seg4f{
        pnt + vec4f{ 0.0, 0.0, wHlf, 1.0 },
        pnt - vec4f{ 0.0, 0.0, wHlf, 1.0 }
    } );
    return cross;
}



////////// VARIABLES ///////////////////////////////////////////////////////////////////////////////
lseg4f pointPaint;
lseg4f camRays;



////////// MAIN ////////////////////////////////////////////////////////////////////////////////////
int main( int argc, char* argv[] ){
    // 0. Load
    string tPath = "/home/james/nanoverse/C++/OpenGL/astrophysics/data/cme0_dcmer_0000_bang_0000_tB/stepnum_005.fits";
    string pPath = "/home/james/nanoverse/C++/OpenGL/astrophysics/data/cme0_dcmer_0000_bang_0000_pB/stepnum_005.fits";
    // 1. Solve
    SolnPair solnOne = calc_coords( tPath, pPath, 0.0f, 0.50f );
    
}