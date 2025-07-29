////////// INIT ////////////////////////////////////////////////////////////////////////////////////

///// Imports /////////////////////////////////////////////////////////////
/// Standard ///
#include <list>
using std::list;
/// Local ///
#include "../include/PUNCH.hpp"
#include "../include/toolbox.hpp"

///// Aliases /////////////////////////////////////////////////////////////
typedef array<vec4f,2> seg4f;
typedef list<seg4f>    lseg4f;



////////// HELPER FUNCTIONS ////////////////////////////////////////////////////////////////////////

lseg4f point_cross( const vec4f& pnt, float width_m ){
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


float normSqr( float x, float y, float z ){  return (x*x + y*y + z*z);  }
float normSqr( const vec4f pnt ){  return (normSqr( pnt[0], pnt[1], pnt[2] ) * pnt[3]);  }


lseg4f extract_point_crosses( const Mat& matx, float width_m, float sqrThresh = 2.0f ){
    // Get segments that represent points
    lseg4f rtnSeg;
    int    Mrows = matx.rows;
    int    Ncols = matx.cols;
    vec4f  point{ 0.0, 0.0, 0.0, 1.0 };
    for( int i = 0; i < Mrows; ++i ){
        for( int j = 0; j < Ncols; ++j ){
            for( int k = 0; k < 3; ++k ){  point[k] = matx.at<float>(i,j,k);  }   
            if( normSqr( point ) >= sqrThresh ){
                rtnSeg.splice( rtnSeg.end(), point_cross( point, width_m ) );
            }
        }
    }
    return rtnSeg;
}


lseg4f extract_point_rays_from_origin( const Mat& matx, float sqrThresh = 2.0f ){
    // Get segments that represent points
    lseg4f rtnSeg;
    int    Mrows = matx.rows;
    int    Ncols = matx.cols;
    vec4f  point{ 0.0, 0.0, 0.0, 1.0 };
    vec4f  pZero{ 0.0, 0.0, 0.0, 1.0 };
    for( int i = 0; i < Mrows; ++i ){
        for( int j = 0; j < Ncols; ++j ){
            for( int k = 0; k < 3; ++k ){  point[k] = matx.at<float>(i,j,k);  }   
            if( normSqr( point ) >= sqrThresh ){  rtnSeg.push_back( seg4f{ pZero, point } );  }
        }
    }
    return rtnSeg;
}



////////// RENDERING ///////////////////////////////////////////////////////////////////////////////

void draw_segments( const lseg4f& segments, const vec4f& color ){
    // Draw a collection of segments
    glClr4f( color );
    glBegin( GL_LINE );
    for( const seg4f& segment : segments ){
        glVtx4f( segment[0] );
        glVtx4f( segment[1] );
    }
    glEnd();
}



////////// VARIABLES ///////////////////////////////////////////////////////////////////////////////
lseg4f   pointPaint;
lseg4f   camRays;
vec4f    pointColor{ 255.0f/255.0f, 229.0f/255.0f, 115.0f/255.0f, 1.0f };
vec4f    rayColor{   128.0f/255.0f, 128.0f/255.0f, 128.0f/255.0f, 0.5f };
Camera3D cam{ vec3f{ 0.0f, 0.0f, 100.0f }, vec3f{ 0.0f, 0.0f, 200.0f }, vec3f{ 0.0f, 1.0f, 0.0f } };



////////// RENDERING ///////////////////////////////////////////////////////////////////////////////

void display(){
    // Refresh display

    // Erase the window and the depth buffer
    clear_screen();

    draw_segments( pointPaint, pointColor );
    draw_segments( camRays   , rayColor );
}



////////// MAIN ////////////////////////////////////////////////////////////////////////////////////
int main( int argc, char* argv[] ){
    ///// Make Calculations ///////////////////////////////////////////////
    // 0. Load
    string tPath = "/home/james/nanoverse/C++/OpenGL/astrophysics/data/cme0_dcmer_0000_bang_0000_tB/stepnum_005.fits";
    string pPath = "/home/james/nanoverse/C++/OpenGL/astrophysics/data/cme0_dcmer_0000_bang_0000_pB/stepnum_005.fits";
    // 1. Solve
    SolnPair solnOne = calc_coords( tPath, pPath, 0.0f, 0.50f );
    // 2. Near Points
    // 3. Far Points
    // 4. Color Change && Rays
    // 5. Setup OpenGL
    // 6. Render


    ///// Initialize GLUT /////////////////////////////////////////////////
    glutInit( &argc , argv );

    // Request window with size specified in pixels
    glutInitWindowSize( _WINDOW_W, _WINDOW_H );

    // Create the window
    glutCreateWindow( "Vertex Array Object (VAO) Test" );

    // NOTE: Set modes AFTER the window / graphics context has been created!
    // Request double buffered, true color window 
    glutInitDisplayMode( GLUT_RGB | GLUT_DOUBLE | GLUT_DEPTH );
    
    glEnable( GL_CULL_FACE );
    // OpenGL should normalize normal vectors
	glEnable( GL_NORMALIZE );
    glDepthRange( 0.0f , 1.0f ); 
    glEnable( GL_BLEND );
    glBlendFunc( GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA );
}