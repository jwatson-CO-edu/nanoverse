////////// INIT ////////////////////////////////////////////////////////////////////////////////////

///// Imports /////////////////////////////////////////////////////////////
/// Standard ///
#include <list>
using std::list;
/// Local ///
#include "../../include/PUNCH.hpp"
#include "../../include/toolbox.hpp"
#include "../../include/utils.hpp"

///// Aliases /////////////////////////////////////////////////////////////
typedef array<vec4f,2> seg4f;
typedef list<seg4f>    lseg4f;

extern int _WINDOW_W;
extern int _WINDOW_H;



////////// CME CLASSES /////////////////////////////////////////////////////////////////////////////

class SolnFrame{ public:
    // Container for combined CME data from all 4 satellites
    vector<PairPtr> pairs;
    vvstr paths;
    list<vec3f> totPts;

    SolnFrame( vstr pathList ){
        // PUNCH has four satellites
        pairs.reserve(4);   
        paths = list_files_at_paths_w_ext( pathList, "fits", true ); // Fetch all relevant filenames
    } 

    void solve_one_frame( const vvec2u& pathAddrs ){
        // Fetch the files at the addresses, Assume [ ..., <tB_i>, <pB_i>, ... ]
        string tBpath;
        string pBpath;
        for( ubyte i = 0; i < 8; i += 2 ){
            tBpath = paths[pathAddrs[i  ][0]][pathAddrs[i  ][1]];
            pBpath = paths[pathAddrs[i+1][0]][pathAddrs[i+1][1]];
            pairs.push_back( PairPtr{
                new SolnPair{}
            } )
        }
    }
};



////////// HELPER FUNCTIONS ////////////////////////////////////////////////////////////////////////

vec4f add_vec4f( const vec4f& op1, const vec4f& op2 ){
    return vec4f{ op1.x+op2.x, op1.y+op2.y, op1.z+op2.z, max( op1.x, op2.x ) };
}

vec4f sub_vec4f( const vec4f& op1, const vec4f& op2 ){
    return vec4f{ op1.x-op2.x, op1.y-op2.y, op1.z-op2.z, min( op1.x, op2.x ) };
}


lseg4f point_cross( const vec4f& pnt, float width_m ){
    // Return segments representing a point
    lseg4f cross;
    float  wHlf = width_m / 2.0;
    // X-cross
    cross.push_back( seg4f{
        add_vec4f( pnt, vec4f{ wHlf, 0.0, 0.0, 1.0 } ) ,
        sub_vec4f( pnt, vec4f{ wHlf, 0.0, 0.0, 1.0 } ) 
    } );
    // Y-cross
    cross.push_back( seg4f{
        add_vec4f( pnt, vec4f{ 0.0, wHlf, 0.0, 1.0 } ),
        sub_vec4f( pnt, vec4f{ 0.0, wHlf, 0.0, 1.0 } )
    } );
    // Z-cross
    cross.push_back( seg4f{
        add_vec4f( pnt, vec4f{ 0.0, 0.0, wHlf, 1.0 } ),
        sub_vec4f( pnt, vec4f{ 0.0, 0.0, wHlf, 1.0 } )
    } ); 
    return cross;
}


float normSqr( float x, float y, float z ){  return (x*x + y*y + z*z);  }
float normSqr( const vec4f pnt ){  return (normSqr( pnt[0], pnt[1], pnt[2] ) * pnt[3]);  }


lseg4f extract_point_crosses( const Mat& matx, float width_m, float sqrThresh = 2.0f ){
    // Get segments that represent points
    lseg4f rtnSeg;
    lseg4f temp;
    int    Mrows = matx.size[0];
    int    Ncols = matx.size[1];
    vec4f  point{ 0.0, 0.0, 0.0, 1.0 };
    cout << "Matx: " << Mrows << " x " << Ncols << endl;
    for( int i = 0; i < Mrows; ++i ){
        for( int j = 0; j < Ncols; ++j ){
            for( int k = 0; k < 3; ++k ){  point[k] = matx.at<float>(i,j,k);  }   
            
            if( normSqr( point ) >= sqrThresh ){
                // cout << point << endl;
                temp = point_cross( point, width_m );
                rtnSeg.splice( rtnSeg.end(), temp, temp.begin(), temp.end() );
            }
        }
    }
    return rtnSeg;
}


lseg4f extract_point_rays_from_origin( const Mat& matx, float sqrThresh = 2.0f ){
    // Get segments that represent points
    lseg4f rtnSeg;
    lseg4f temp;
    int    Mrows = matx.size[0];
    int    Ncols = matx.size[1];
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
    glBegin( GL_LINES );
    // cout << "There are " << segments.size() << " segments!" << endl;
    for( const seg4f& segment : segments ){
        glVtx3f( segment[0] );
        glVtx3f( segment[1] );
        // cout << segment[0] << ", " << segment[1] << endl;
    }
    
    glEnd();
    ErrCheck( "draw_segments" );
}



////////// SIMULATION //////////////////////////////////////////////////////////////////////////////

void tick(){
    // Background work
    // 3. Set projection
	project();
    // Tell GLUT it is necessary to redisplay the scene
	glutPostRedisplay();
}



////////// VARIABLES ///////////////////////////////////////////////////////////////////////////////
lseg4f   pointPaint;
lseg4f   camRays;
vec4f    pointColor{ 255.0f/255.0f, 229.0f/255.0f, 115.0f/255.0f, 1.00f };
vec4f    rayColor{   128.0f/255.0f, 128.0f/255.0f, 128.0f/255.0f, 0.25f };
Camera3D cam{ vec3f{ 0.0f, 0.0f, 100.0f }, vec3f{ 0.0f, 0.0f, 200.0f }, vec3f{ 0.0f, 1.0f, 0.0f } };
int  xLast  = INT32_MAX, xDelta = 0;
int  yLast  = INT32_MAX, yDelta = 0;
bool leftClicked = false;
float _ROT_RAD_PER_PXL = 0.5f * (M_PI / 180.0f); 



////////// RENDERING ///////////////////////////////////////////////////////////////////////////////

void display(){
    // Refresh display

    // Erase the window and the depth buffer
    clear_screen();

    cam.look();
    ///// DRAW LOOP BEGIN /////////////////////////////////////////////////
    glPushMatrix();
    glTranslatef( 0.0f, 0.0f, 214.61f );
    glClr4f( vec4f{0.0, 0.0, 0.0, 1.0} );
    glutSolidSphere( 20.0f, 20, 20 );
    glPopMatrix();
    
    draw_segments( pointPaint, pointColor );
    draw_segments( camRays   , rayColor );
    ///// DRAW LOOP END ///////////////////////////////////////////////////
    

    // Check for errors, Flush, and swap
	ErrCheck( "display" );
	glFlush();
	glutSwapBuffers();
}


void mouse_move( int x, int y ){
    // Mouse activity callback
    // cout << "MOVE!" << endl;
    if( xLast < INT32_MAX ){
        xDelta = x - xLast;
        yDelta = y - yLast;     
    }
    if( leftClicked ){
        cam.trackball_rotate( 
           -_ROT_RAD_PER_PXL * xDelta, 
            _ROT_RAD_PER_PXL * yDelta 
        );
    }
    xLast = x;
    yLast = y;
    glutPostRedisplay();
}


void mouse_click( int button, int state, int x, int y ){
    // cout << "CLICK!" << endl;
    switch (button){
        case GLUT_LEFT_BUTTON:
            if( state == GLUT_DOWN ){  leftClicked = true;   }
            if( state == GLUT_UP   ){  leftClicked = false;  }
            break;
        
        default:
            break;
    }
    glutPostRedisplay();
}



////////// MAIN ////////////////////////////////////////////////////////////////////////////////////
int main( int argc, char* argv[] ){
    ///// Make Calculations ///////////////////////////////////////////////
    lseg4f temp;
    // 0. Load
    string tPath = "./astrophysics/data/cme0_dcmer_0000_bang_0000_tB/stepnum_005.fits";
    string pPath = "./astrophysics/data/cme0_dcmer_0000_bang_0000_pB/stepnum_005.fits";
    // 1. Solve
    SolnPair solnOne = calc_coords( tPath, pPath, 0.50f );
    // cout << solnOne.zetaMinus.rows << " x " << solnOne.zetaMinus.cols << " || " << solnOne.zetaMinus.rows << " x " << solnOne.zetaMinus.cols << endl;
    // 2. Near Points
    temp = extract_point_crosses( solnOne.zetaPlus, 2.0f, 2.0f );
    pointPaint.splice( pointPaint.end(), temp, temp.begin(), temp.end() );
    // 3. Far Points
    temp = extract_point_crosses( solnOne.zetaMinus, 2.0f, 2.0f );
    pointPaint.splice( pointPaint.end(), temp, temp.begin(), temp.end() );
    temp = extract_point_rays_from_origin( solnOne.zetaPlus, 2.0f );
    camRays.splice( camRays.end(), temp, temp.begin(), temp.end() );
    
    set_near_far_draw_distance( 0.5f, 250.0f );
    cam.look_at( vec3f{ 0.0f, 0.0f, 214.61f } );

    ///// Initialize GLUT /////////////////////////////////////////////////
    glutInit( &argc , argv );

    // Request window with size specified in pixels
    glutInitWindowSize( _WINDOW_W, _WINDOW_H );

    // Create the window
    glutCreateWindow( "Solar Data Test" );

    // NOTE: Set modes AFTER the window / graphics context has been created!
    // Request double buffered, true color window 
    glutInitDisplayMode( GLUT_RGBA | GLUT_DOUBLE | GLUT_DEPTH );
    
    // glEnable( GL_CULL_FACE );
    // OpenGL should normalize normal vectors
	// glEnable( GL_NORMALIZE );
    	
    // glDepthFunc( GL_GREATER );
    glDepthRange( 0.0f , 1.0f ); 
    glEnable( GL_BLEND );
    glBlendFunc( GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA );

    ///// Initialize GLUT Callbacks ///////////////////////////////////////
    printf( "About to assign callbacks ...\n" );

    //  Tell GLUT to call "display" when the scene should be drawn
    glutDisplayFunc( display );

    // Tell GLUT to call "idle" when there is nothing else to do
    glutIdleFunc( tick );
    
    // Tell GLUT to call "reshape" when the window is resized
    glutReshapeFunc( reshape );
    
    //  Tell GLUT to call "special" when an arrow key is pressed or released
    // glutSpecialFunc( special_dn );
    // glutSpecialUpFunc( special_up );

    // Tell GLUT to call "mouse" when mouse input arrives
    glutMouseFunc( mouse_click ); // Clicks
    glutPassiveMotionFunc( mouse_move ); // Movement
    glutMotionFunc( mouse_move ); // Movement
    
    // //  Tell GLUT to call "key" when a key is pressed or released
    // glutKeyboardFunc( key_dn );
    // glutKeyboardUpFunc( key_up );

    ///// GO ///// GO ///// GO ////////////////////////////////////////////
    printf( "Entering main loop ...\n" );
    
    // Pass control to GLUT so it can interact with the user
    glutMainLoop();
    
    printf( "\n### DONE ###\n\n" );

    //  Return code
    return 0;
}