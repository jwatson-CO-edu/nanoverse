////////// INIT ////////////////////////////////////////////////////////////////////////////////////
#include "toolbox.h"



////////// PROGRAM SETTINGS ////////////////////////////////////////////////////////////////////////

/// Simulation Settings ///
const uint _N_ATTRCTR = 1; // Number of attractor ribbons

/// View Settings ///
const float _SCALE   = 10.0; // Scale Dimension
const int   _FOV_DEG = 55; // - Field of view (for perspective)



////////// PROGRAM STRUCTS /////////////////////////////////////////////////////////////////////////

///// 6D Lorenz Attractor Dynamics ////////////////////////////////////////

typedef struct{
    // State -or- Rate of Change
    union{ float x;  float xDot; };
    union{ float y;  float yDot; };
    union{ float z;  float zDot; };
    union{ float v;  float vDot; };
    union{ float u;  float uDot; };
    union{ float o;  float oDot; };
}L6DStatef;


L6DStatef step_state( const L6DStatef state, const L6DStatef veloc, float timeStep ){
    // Rectangular integration of state
    L6DStatef rtnState = {
        state.x + veloc.xDot * timeStep,
        state.y + veloc.yDot * timeStep,
        state.z + veloc.zDot * timeStep,
        state.v + veloc.vDot * timeStep,
        state.u + veloc.uDot * timeStep,
        state.o + veloc.oDot * timeStep
    };
    return rtnState;
}


L6DStatef calc_rates_of_change( const L6DStatef s, float sigma, float r, float b ){
    // Get the state rate of change for a 6D Lorenz Attractor, given the present state and the system params
    /* Felicio, Carolini C., and Paulo C. Rech. "On the dynamics of five-and six-dimensional Lorenz models." 
       Journal of Physics Communications 2, no. 2 (2018): 025028.*/
    float b2p1 = 1.0f + 2.0f*b;
    L6DStatef rtnState = { // Section 2.2, Page 8
        sigma*(s.y - s.x),
        r*s.x - s.y - s.x*s.z + s.z*s.v - 2.0f*s.v*s.o,
        s.x*s.y - b*s.z - s.x*s.u - s.y*s.v,
        -1.0f*b2p1*sigma*s.v + sigma*s.u/(b2p1) ,
        s.x*s.z - 2.0f*s.x*s.o + r*s.v - b2p1*s.u,
        2.0f*s.x*s.u + 2.0f*s.y*s.v - 4.0f*b*s.o
    };
    return rtnState;
}

vec4f get_state_half( const L6DStatef s, ubyte half ){
    // Return either the first or second half of `s` as a scaled 3D vector
    vec4f rtnHalf = make_0_vec4f();
    if( half ){
        rtnHalf.x = s.v;
        rtnHalf.y = s.u;
        rtnHalf.z = s.o;
    }else{
        rtnHalf.x = s.x;
        rtnHalf.y = s.y;
        rtnHalf.z = s.z;
    }
    return rtnHalf;
}



///// 6D Lorenz Attractor /////////////////////////////////////////////////

typedef struct{
    // 6D Lorenz Attractor, See above ref
    float     sigma;
    float     r;
    float     b;
    L6DStatef state;
    float     tsSec;
    uint /**/ Nstat;
    uint /**/ bgnDx;
    uint /**/ endDx;
    vec4f*    edge0;
    vec4f*    edge1;
    vec4f     color;
    float     h1scl;
}Attractor6D;


Attractor6D* make_6D_attractor( float sigma_, float r_, float b_, L6DStatef initState, 
                                float tsSec_, uint Nstates, vec4f color_, float h1scl_ ){
    // Setup the attractor with simulation params and history space
    Attractor6D* rtnStruct = (Attractor6D*) malloc( sizeof( Attractor6D ) );
    // Set Simulation Params //
    rtnStruct->sigma = sigma_;
    rtnStruct->r     = r_;
    rtnStruct->b     = b_;
    rtnStruct->state = initState;
    rtnStruct->tsSec = tsSec_;
    // Set Bookkeeping //
    rtnStruct->Nstat = Nstates;
    rtnStruct->bgnDx = 0;
    rtnStruct->endDx = 1;
    // Alloc Mem //
    rtnStruct->edge0 = (vec4f*) malloc( Nstates * sizeof( vec4f ) );
    rtnStruct->edge1 = (vec4f*) malloc( Nstates * sizeof( vec4f ) );
    // Set Display Params //
    rtnStruct->color = color_;
    rtnStruct->h1scl = h1scl_;
    // Set Init State Display //
    rtnStruct->edge0[0] = get_state_half( initState, 0 );
    rtnStruct->edge1[0] = get_state_half( initState, 1 );
    // Return //
    return rtnStruct;
}


void delete_6D_attractor( Attractor6D* attractor ){
    // Free attractor memory
    free( attractor->edge0 );
    free( attractor->edge1 );
    free( attractor );
}


void step_6D_attractor( Attractor6D* attractor ){
    // Run one timestep and append to state history
    vec4f h0, h1;
    uint end = attractor->endDx;
    attractor->state = step_state( 
        attractor->state, 
        calc_rates_of_change( attractor->state, attractor->sigma, attractor->r, attractor->b ), 
        attractor->tsSec
    );
    h0 = get_state_half( attractor->state, 0 );
    h1 = get_state_half( attractor->state, 1 );
    attractor->edge0[ end ] = h0;
    attractor->edge1[ end ] = add_vec4f( h0, scale_vec4f( h1, attractor->h1scl ) );
    attractor->endDx = (end+1)%(attractor->Nstat);
    if( attractor->bgnDx == end ){  attractor->bgnDx = attractor->endDx;  }
}


void draw_6D_attractor( Attractor6D* attractor ){
    // Render the state history as a triangle strip with receding
    uint  Ntot, ndx, ldx; 
    uint  bgn = attractor->bgnDx;
    uint  len = attractor->Nstat;
    float alpha;
    if( bgn == attractor->endDx ){  Ntot = attractor->Nstat;  }else{  Ntot = attractor->endDx;  }
    ldx = (bgn + 1) % len;
    glBegin( GL_TRIANGLE_STRIP );
    for( uint i = 0; i < Ntot; ++i ){
        ndx   = (bgn + i) % len;
        alpha = (Ntot - i)*1.0f/Ntot;
        glColor4f( attractor->color.r , attractor->color.g , attractor->color.b, alpha );
        if( i < 1 ){  glNrm4f( get_CCW_tri_norm(  
            attractor->edge1[ ndx ],
            attractor->edge0[ ndx ],
            attractor->edge0[ ldx ]
        ) );  }else{  glNrm4f( get_CCW_tri_norm(
            attractor->edge1[ ldx ],
            attractor->edge0[ ndx ],
            attractor->edge1[ ndx ]
        ) ) ; }
        glVtx4f( attractor->edge0[ ndx ] );  glVtx4f( attractor->edge1[ ndx ] );
        ldx = ndx;
    }
    glEnd();
}



////////// PROGRAM GLOBALS /////////////////////////////////////////////////////////////////////////

Attractor6D** attractors = NULL;
Camera3D /**/ cam = { {4.0f, 2.0f, 2.0f}, {0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 1.0f} };



////////// WINDOW & VIEW STATE /////////////////////////////////////////////////////////////////////
float w2h = 0.0f; // Aspect ratio


static void project(){
	// Set projection
	// Adapted from code provided by Willem A. (Vlakkies) Schreüder  
	// NOTE: This function assumes that aspect rario will be computed by 'resize'
	//  Tell OpenGL we want to manipulate the projection matrix
	glMatrixMode( GL_PROJECTION );
	//  Undo previous transformations
	glLoadIdentity();
	gluPerspective( _FOV_DEG , //- Field of view angle, in degrees, in the y direction.
					w2h , // ----- Aspect ratio , the field of view in the x direction. Ratio of x (width) to y (height).
					_SCALE/4 , //- Specifies the distance from the viewer to the near clipping plane (always positive).
					4*_SCALE ); // Specifies the distance from the viewer to the far clipping plane (always positive).
	// Switch back to manipulating the model matrix
	glMatrixMode( GL_MODELVIEW );
	// Undo previous transformations
	glLoadIdentity();
}


void reshape( int width , int height ){
	// GLUT calls this routine when the window is resized
    // Adapted from code provided by Willem A. (Vlakkies) Schreüder  
	// Calc the aspect ratio: width to the height of the window
	w2h = ( height > 0 ) ? (float) width / height : 1;
	// Set the viewport to the entire window
	glViewport( 0 , 0 , width , height );
	// Set projection
	project();
}



////////// SIMULATION //////////////////////////////////////////////////////////////////////////////

void idle(){
	// Simulation updates in between repaints
	
    for( uint i = 0; i < _N_ATTRCTR; ++i ){
        step_6D_attractor( attractors[i] );
    }

	// Tell GLUT it is necessary to redisplay the scene
	glutPostRedisplay();
}



////////// RENDERING ///////////////////////////////////////////////////////////////////////////////

void display(){
	// Display the scene
	
	// Clear the image
	glClearDepth( 1.0f );
	glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );
	
	// Reset previous transforms to the identity matrix
	glLoadIdentity();

    look( cam );

    for( uint i = 0; i < _N_ATTRCTR; ++i ){
        draw_6D_attractor( attractors[i] );
    }

    // Check for errors, Flush, and swap
	ErrCheck( "display" );
	glFlush();
	glutSwapBuffers();
}



////////// MAIN ////////////////////////////////////////////////////////////////////////////////////

// Start up GLUT and tell it what to do
int main( int argc , char* argv[] ){
	init_rand(); // initialize random seed based on system clock

    ///// Construct Geometry & Set Params /////////////////////////////////
    attractors = (Attractor6D**) malloc( _N_ATTRCTR * sizeof( Attractor6D ) );

    cam.eyeLoc = make_vec4f( 20.0, 20.0, 20.0 );

    L6DStatef initState = {
        randf_range( -0.5f, +0.5f ),
        randf_range( -0.5f, +0.5f ),
        randf_range( -0.5f, +0.5f ),
        0.0f,
        0.0f,
        0.0f
    };
    vec4f rbbnClr = {0.0f,1.0f,0.0f,1.0f};

    for( uint i = 0; i < _N_ATTRCTR; ++i ){
        attractors[i] = make_6D_attractor(
            randf_range(  1.0f ,  50.0f ),
            randf_range( 50.0f , 300.0f ),
            randf_range(  0.01f,   5.0f ),
            initState,
            0.05,
            50,
            rbbnClr,
            0.5
        );
    }


    ///// Setup GLUT & Create Window //////////////////////////////////////

    // Initialize GLUT and process user parameters
	glutInit( &argc , argv );
	
	// Request double buffered, true color window 
	glutInitDisplayMode( GLUT_RGB | GLUT_DOUBLE | GLUT_DEPTH );
	
	// Request 500 x 500 pixel window
	glutInitWindowSize( 975 , 725 );

    // Create the window
	glutCreateWindow( "6D Lorenz Attractor" );
	
	// Tell GLUT to call "idle" when there is nothing else to do
	glutIdleFunc( idle );
	
	// Enable z-testing at the full ranger
	glEnable( GL_DEPTH_TEST );
	glDepthRange( 0.0f , 1.0f );
	
	// Tell GLUT to call "display" when the scene should be drawn
	glutDisplayFunc( display );
	
	// Tell GLUT to call "reshape" when the window is resized
	glutReshapeFunc( reshape );
	
	// Tell GLUT to call "special" when an arrow key is pressed
	// glutSpecialFunc( special );
	
	// Tell GLUT to call "key" when a key is pressed
	// glutKeyboardFunc( key );
	
	// Check for errors
	ErrCheck( "main" );
	
	//  Pass control to GLUT so it can interact with the user
	glutMainLoop();


    ///// Free ALL Allocated Heap Memory //////////////////////////////////

    for( uint i = 0; i < _N_ATTRCTR; ++i ){
        delete_6D_attractor( attractors[i] );
    }
    free( attractors );


	///// End /////////////////////////////////////////////////////////////
	return 0;

}