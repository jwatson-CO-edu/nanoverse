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


////////// HELPER FUNCTIONS ////////////////////////////////////////////////////////////////////////

vec4f add_vec4f( const vec4f& op1, const vec4f& op2 ){
    return vec4f{ op1.x+op2.x, op1.y+op2.y, op1.z+op2.z, max( op1.x, op2.x ) };
}

vec4f sub_vec4f( const vec4f& op1, const vec4f& op2 ){
    return vec4f{ op1.x-op2.x, op1.y-op2.y, op1.z-op2.z, min( op1.x, op2.x ) };
}

void init_rand(){  srand( time( NULL ) );  }

float randf(){
    // Return a pseudo-random number between 0.0 and 1.0
    return  1.0f * rand() / RAND_MAX;
}

float randf_range( float lo, float hi ){
    // Return a pseudo-random number between `lo` and `hi`
    // NOTE: This function assumes `hi > lo`
    return lo + (hi - lo) * randf();
}

vec3f rand_vec3f( float scale = 1.0 ){
    vec3f rtnVec{ randf_range( -1.0f, 1.0f ), randf_range( -1.0f, 1.0f ), randf_range( -1.0f, 1.0f ) };
    return normalize( rtnVec ) * scale;
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


lseg4f point_cross_rand( const vec4f& pnt, float width_m ){
    // Return segments representing a point, with a random orientation
    lseg4f crossPts;
    float  wHlf   = width_m / 2.0;
    vec3f  pntP   = no_scale( pnt );
    vec3f  xBasis = rand_vec3f();
    vec3f  zBasis = normalize( cross( xBasis, rand_vec3f() ) );
    vec3f  yBasis = normalize( cross( zBasis, xBasis ) );
    // X-cross
    crossPts.push_back( seg4f{
        extend( pntP + (xBasis*wHlf) ),
        extend( pntP - (xBasis*wHlf) )
    } );
    // Y-cross
    crossPts.push_back( seg4f{
        extend( pntP + (yBasis*wHlf) ),
        extend( pntP - (yBasis*wHlf) )
    } );
    // Z-cross
    crossPts.push_back( seg4f{
        extend( pntP + (zBasis*wHlf) ),
        extend( pntP - (zBasis*wHlf) )
    } ); 
    // cout << "About to return a cross of " << crossPts.size() << " segments!" << endl;
    return crossPts;
}


float normSqr( float x, float y, float z ){  return (x*x + y*y + z*z);  }
float normSqr( const vec3f pnt ){  return normSqr( pnt[0], pnt[1], pnt[2] );  }
float normSqr( const vec4f pnt ){  return (normSqr( pnt[0], pnt[1], pnt[2] ) * pnt[3]);  }


lseg4f extract_point_crosses( const Mat& matx, float scale, float sqrThresh = 2.0f ){
    // Get segments that represent points
    lseg4f rtnSeg;
    lseg4f temp;
    int    Mrows = matx.size[0];
    int    Ncols = matx.size[1];
    vec4f  point{ 0.0, 0.0, 0.0, 0.0 };
    cout << "Matx: " << Mrows << " x " << Ncols << endl;
    for( int i = 0; i < Mrows; ++i ){
        for( int j = 0; j < Ncols; ++j ){
            for( int k = 0; k < 4; ++k ){  point[k] = matx.at<float>(i,j,k);  }   
            
            if( normSqr( no_scale(point) ) >= sqrThresh ){
                temp = point_cross_rand( one_scale( point ), point[3]*scale );
                rtnSeg.splice( rtnSeg.end(), temp, temp.begin(), temp.end() );
            }
        }
    }
    return rtnSeg;
}


lseg4f extract_point_crosses( const list<vec4f>& pntScl, float scale, float sqrThresh = 2.0f ){
    // Get segments that represent points
    lseg4f rtnSeg;
    lseg4f temp;
    for( const vec4f& point : pntScl ){
        if( normSqr( no_scale(point) ) >= sqrThresh ){
            temp = point_cross_rand( one_scale( point ), point[3]*scale );
            rtnSeg.splice( rtnSeg.end(), temp, temp.begin(), temp.end() );
        }
    }
    cout << "About to return " << rtnSeg.size() << " segments!" << endl;
    return rtnSeg;
}


void extract_points_into_list( const Mat& matx, list<vec4f>& dstLst ){
    int    Mrows = matx.size[0];
    int    Ncols = matx.size[1];
    vec4f  point{ 0.0, 0.0, 0.0, 0.0 };
    for( int i = 0; i < Mrows; ++i ){
        for( int j = 0; j < Ncols; ++j ){
            for( int k = 0; k < 4; ++k ){  point[k] = matx.at<float>(i,j,k);  }  
            dstLst.push_back( point );
        }
    }
}


////////// CME CLASSES /////////////////////////////////////////////////////////////////////////////

class SolnFrame{ public:
    // Container for combined CME data from all 4 satellites
    vector<PairPtr> pairs;
    vvstr /*-----*/ paths;
    list<vec4f>     ptsPlus;
    list<vec4f>     ptsMinus;
    list<vec4f>     clrPlus;
    list<vec4f>     clrMinus;
    vector<vec4f>   vColors;

    SolnFrame( vstr pathList ){
        // PUNCH has four satellites
        pairs.reserve(4);   
        paths = list_files_at_paths_w_ext( pathList, "fits", true ); // Fetch all relevant filenames
        // cout << paths << endl;
    } 

    void solve_one_frame( const vvec2u& pathAddrs, const vector<vec4f>& colorSeq ){
        // Fetch the files at the addresses, Assume [ ..., <tB_i>, <pB_i>, ... ]
        string tBpath;
        string pBpath;
        ubyte  Nclr = (ubyte) min( (size_t) 255, colorSeq.size() );
        size_t Npls = 0;
        size_t Nmns = 0;
        for( ubyte i = 0; i < 8; i += 2 ){
            tBpath = paths[pathAddrs[i  ][0]][pathAddrs[i  ][1]];
            pBpath = paths[pathAddrs[i+1][0]][pathAddrs[i+1][1]];

            // pairs[i/2] = calc_coords_ptr( tBpath, pBpath, 0.5f );
            pairs.push_back( calc_coords_ptr( tBpath, pBpath, 0.5f ) );

            Npls = pairs[i/2]->pntsPlus.size()*3;
            Nmns = pairs[i/2]->pntsMinus.size()*3;
            ptsPlus.splice(  ptsPlus.end() , pairs[i/2]->pntsPlus , pairs[i/2]->pntsPlus.begin() , pairs[i/2]->pntsPlus.end()  );
            ptsMinus.splice( ptsMinus.end(), pairs[i/2]->pntsMinus, pairs[i/2]->pntsMinus.begin(), pairs[i/2]->pntsMinus.end() );
            cout << (int) (i/2) << ", " << ptsPlus.size() << ", " << ptsMinus.size() << endl;
            for( size_t j = 0; j < Npls; ++j ){  clrPlus.push_back(  colorSeq[(i/2)%Nclr] );  }
            for( size_t j = 0; j < Nmns; ++j ){  clrMinus.push_back( colorSeq[(i/2)%Nclr] );  }
            // cout << "Collected " << colors.size() << " colors!, " << Npnt << ", " << pairs[i/2]->pntsPlus.size() << endl;
        }
    }
};



////////// RENDERING ///////////////////////////////////////////////////////////////////////////////

void draw_segments( const lseg4f& segments, const vector<vec4f>& color ){
    // Draw a collection of segments
    size_t i = 0;
    vec4f prvClr{ 0.0, 0.0, 0.0, 0.0 };
    glBegin( GL_LINES );
    // cout << "There are " << segments.size() << " segments!" << endl;
    for( const seg4f& segment : segments ){
        if( color[i] != prvClr ){
            glClr4f( color[i] );
            prvClr = color[i];
        }  
        glVtx3f( segment[0] );
        glVtx3f( segment[1] );
        ++i;
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



////////// CONSTANTS ///////////////////////////////////////////////////////////////////////////////
const vec4f RED{   255.0f/255.0f,   0.0f/255.0f,   0.0f/255.0f, 1.00f };
const vec4f GREEN{   0.0f/255.0f, 255.0f/255.0f,   0.0f/255.0f, 1.00f };
const vec4f BLUE{    0.0f/255.0f,   0.0f/255.0f, 255.0f/255.0f, 1.00f };
const vec4f WHITE{ 255.0f/255.0f, 255.0f/255.0f, 255.0f/255.0f, 1.00f };
const vec4f BLACK{   0.0f/255.0f,   0.0f/255.0f,   0.0f/255.0f, 1.00f };



////////// VARIABLES ///////////////////////////////////////////////////////////////////////////////
lseg4f /*--*/ pointPaint;
lseg4f /*--*/ camRays;
vec4f /*---*/ pointColor{ 255.0f/255.0f, 229.0f/255.0f, 115.0f/255.0f, 1.00f };
vec4f /*---*/ rayColor{   128.0f/255.0f, 128.0f/255.0f, 128.0f/255.0f, 0.25f };
vector<vec4f> pntClr;
list<vec4f> totClr;
Camera3D /**/ cam{ vec3f{ 0.0f, 0.0f, 100.0f }, vec3f{ 0.0f, 0.0f, 200.0f }, vec3f{ 0.0f, 1.0f, 0.0f } };
int /*-----*/ xLast /*------*/ = INT32_MAX, xDelta = 0;
int /*-----*/ yLast /*------*/ = INT32_MAX, yDelta = 0;
bool /*----*/ leftClicked /**/ = false;
float /*---*/ _ROT_RAD_PER_PXL = 0.5f * (M_PI / 180.0f); 
float /*---*/ _POINT_SCALE     = 50000.0f;



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
    
    // cout << "There are " << pointPaint.size() << " points to paint!" << endl;
    draw_segments( pointPaint, pntClr );
    // draw_segments( camRays   , rayColor );
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
    vstr fitsPaths = {
        "./astrophysics/data/cme0_dcmer_0000_bang_0000_pB",
        "./astrophysics/data/cme0_dcmer_0000_bang_0000_tB",
        "./astrophysics/data/cme0_dcmer_030E_bang_0000_pB",
        "./astrophysics/data/cme0_dcmer_030E_bang_0000_tB",
        "./astrophysics/data/cme0_dcmer_060W_bang_0000_pB",
        "./astrophysics/data/cme0_dcmer_060W_bang_0000_tB",
        "./astrophysics/data/cme0_dcmer_090E_bang_0000_pB",
        "./astrophysics/data/cme0_dcmer_090E_bang_0000_tB",
    };
    SolnFrame frame{ fitsPaths };
    uint /**/ seq = 70; //0 // 40 // 60
    vvec2u    files = {
        vec2u{ 1, seq },
        vec2u{ 0, seq },
        vec2u{ 3, seq },
        vec2u{ 2, seq },
        vec2u{ 5, seq },
        vec2u{ 4, seq },
        vec2u{ 7, seq },
        vec2u{ 6, seq },
    };
    vvec4f colorSequence = { RED, GREEN, BLUE, WHITE };
    frame.solve_one_frame( files, colorSequence );
    
    temp = extract_point_crosses( frame.ptsPlus, _POINT_SCALE );
    pointPaint.splice( pointPaint.end(), temp, temp.begin(), temp.end() );
    totClr.splice( totClr.end(), frame.clrPlus, frame.clrPlus.begin(), frame.clrPlus.end() );

    temp = extract_point_crosses( frame.ptsMinus, _POINT_SCALE );
    pointPaint.splice( pointPaint.end(), temp, temp.begin(), temp.end() );
    totClr.splice( totClr.end(), frame.clrMinus, frame.clrMinus.begin(), frame.clrMinus.end() );

    pntClr = list_2_vector( totClr );

    cout << "There are " << pointPaint.size() << " segments!" << endl;
    cout << "There are " << pntClr.size() << " colors!" << endl;
    
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