////////// INIT ////////////////////////////////////////////////////////////////////////////////////
#include "toolbox.h"



////////// PROGRAM SETTINGS ////////////////////////////////////////////////////////////////////////

/// View Settings ///
const float _SCALE /**/ = 750.0; // Scale Dimension
const int   _FOV_DEG    =  55; // - Field of view (for perspective)
const float _TARGET_FPS =  60.0f; // Desired framerate

/// Simulation Settings ///
const uint  _N_ATTRCTR  =     5; // ------ Number of attractor ribbons
const uint  _N_STATES   =   300; // ------ Number of attractor states to store 
const float _DIM_KILL   = 20000.0f; // --- If {X,Y,Z} state wanders outside of this box, then consider it unstable
const float _TIMESTEP_S =     0.00025f; // Number of seconds to advance in integration
const float _HLF_1_SCL  =     0.1f; // --- Scaling factor for second half of state
const float _MAX_AGE_S  =    30.0f; // --- Number of seconds a ribbon can stay unchanged
const ubyte _N_STP_FRM  =     3; // ------ Number of simulation steps to run per frame
const uint  _MAX_AGE_T  = (uint) (_MAX_AGE_S * _TARGET_FPS * _N_STP_FRM); // Number of timesteps a ribbon can stay unchanged
const float _DEL_FACTOR = 0.10; // Degree to which params can change after `_MAX_AGE_S` has elapsed

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
    uint /**/ age;
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
    rtnStruct->age   = 0;
    // Alloc Mem //
    rtnStruct->edge0 = (vec4f*) malloc( Nstates * sizeof( vec4f ) );
    rtnStruct->edge1 = (vec4f*) malloc( Nstates * sizeof( vec4f ) );
    // Set to zero for when we calc centroid early on
    for( uint i = 1; i < Nstates; ++i ){
        rtnStruct->edge0[i] = make_0_vec4f();    
        rtnStruct->edge1[i] = make_0_vec4f();
    }
    // Set Display Params //
    rtnStruct->color = color_;
    rtnStruct->h1scl = h1scl_;
    // Set Init State Display //
    rtnStruct->edge0[0] = get_state_half( initState, 0 );
    rtnStruct->edge1[0] = get_state_half( initState, 1 );
    // Return //
    return rtnStruct;
}

Attractor6D* make_rand_6D_attractor( void ){
    // Start an attractor in the stable region shown in Figure 7
    L6DStatef initState = {
        randf_range( -0.5f , +0.5f  ),
        randf_range( -0.5f , +0.5f  ),
        randf_range( -0.5f , +0.5f  ),
        0.0f, // randf_range( -0.01f, +0.01f ),
        0.0f, // randf_range( -0.01f, +0.01f ),
        0.0f //- randf_range( -0.01f, +0.01f )
    };
    vec4f rbbnClr = rand_vec4f();
    Attractor6D* rtnPtr = make_6D_attractor(
        randf_range(  25.0f ,  50.0f ),
        randf_range( 200.0f , 300.0f ),
        randf_range(   1.0f,    2.0f ),
        initState,
        _TIMESTEP_S,
        _N_STATES,
        rbbnClr,
        _HLF_1_SCL
    );
    return rtnPtr;
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
    // Increment age, and if the attractor is older than max age, then perturb params and reset age
    ++(attractor->age);
    if( attractor->age >= _MAX_AGE_T ){
        attractor->sigma += randf_range( -(attractor->sigma)*_DEL_FACTOR, (attractor->sigma)*_DEL_FACTOR );
        attractor->r     += randf_range( -(attractor->r)*_DEL_FACTOR    , (attractor->r)*_DEL_FACTOR     );
        attractor->b     += randf_range( -(attractor->b)*_DEL_FACTOR    , (attractor->b)*_DEL_FACTOR     );
        attractor->age   = 0;
        printf( "Attractor at %p is OLD!\n", attractor );
    }
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
        alpha = 1.0f - 1.0f*(Ntot - i)/Ntot;
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

void centroid_AABB_attractor( Attractor6D* attractor, vec4f* centroid, vec4f* bbLo, vec4f* bbHi ){
    // Set the centroid and Axis Aligned Boudning Box of the points in the first edge
    *bbLo = make_vec4f(  1e9f,  1e9f,  1e9f );
    *bbHi = make_vec4f( -1e9f, -1e9f, -1e9f );
    uint  N   = attractor->Nstat;
    vec4f tot = make_0_vec4f();
    vec4f v_i = make_0_vec4f();
    for( uint i = 0; i < N; ++i ){  
        v_i = attractor->edge0[i];
        tot = add_vec4f( tot, v_i ); 
        if( v_i.x < bbLo->x )  bbLo->x = v_i.x;
        if( v_i.y < bbLo->y )  bbLo->y = v_i.y;
        if( v_i.z < bbLo->z )  bbLo->z = v_i.z;
        if( v_i.x > bbHi->x )  bbHi->x = v_i.x;
        if( v_i.y > bbHi->y )  bbHi->y = v_i.y;
        if( v_i.z > bbHi->z )  bbHi->z = v_i.z;
    }
    *centroid = div_vec4f( tot, 1.0f*N );
}


///// Smoothed Follow Camera //////////////////////////////////////////////

typedef struct{
    // Camera intended to somewhat smoothly follow quickly moving targets
    /// Basic Camera ///
    vec4f eyeLoc; // Current camera location (world frame)
    vec4f lookPt; // Current focus of camera (world frame)
    vec4f upVctr; // Direction of "up"
    /// Smoothing ///
    vec4f eyeTgt; // Target camera location
    vec4f lukTgt; // Target focus of camera
    float alpha; //- Blending rate for targets on [0.0, 1.0]
}FollowCameraSmooth;


FollowCameraSmooth get_smooth_camera( const vec4f eyeLoc_, const vec4f lookPt_, float alpha_ ){
    // Return a smoothed camera with blending factor set
    FollowCameraSmooth rtnCam = {
        eyeLoc_,
        lookPt_,
        {0.0f,0.0f,1.0f,1.0f},
        eyeLoc_,
        lookPt_,
        alpha_
    };
    return rtnCam;
}


void cam_smooth_update( FollowCameraSmooth* cam ){
    /// Approach, but never reach, the intended targets
    cam->eyeLoc = blend_vec4f( cam->eyeTgt, cam->alpha, cam->eyeLoc, 1.0f-(cam->alpha) );
    cam->lookPt = blend_vec4f( cam->lukTgt, cam->alpha, cam->lookPt, 1.0f-(cam->alpha) );
    cam->upVctr = cross_vec4f(
        unit_vec4f( sub_vec4f( cam->lookPt, cam->eyeLoc ) ),
        make_vec4f( 1.0f, 0.0f, 0.0f )
    );
}


void look_smooth( const FollowCameraSmooth* camera ){
    // Set camera position, target, and orientation
    // cam_smooth_update( camera );
    gluLookAt( (double) camera->eyeLoc.x, (double) camera->eyeLoc.y, (double) camera->eyeLoc.z,  
               (double) camera->lookPt.x, (double) camera->lookPt.y, (double) camera->lookPt.z,  
               (double) camera->upVctr.x, (double) camera->upVctr.y, (double) camera->upVctr.z );
}



////////// PROGRAM GLOBALS /////////////////////////////////////////////////////////////////////////

Attractor6D** /**/ attractors = NULL;
ubyte* /*-------*/ active     = NULL;
FollowCameraSmooth cam;
float /*--------*/ thetaCam   = 0.0f;
vec4f /*--------*/ camOfstDir  = {0.0f, 0.0f, 0.0f, 1.0f};



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
    vec4f h0_i, h1_i;
    vec4f attrCntr, aabbLo, aabbHi;
    // vec4f totCntr = make_0_vec4f();
    // vec4f totLo   = make_0_vec4f();
    // vec4f totHi   = make_0_vec4f();
    // float factor  = 1.0f*_N_ATTRCTR;
    float width;
    vec4f camOfst = make_vec4f( 1.0f, 1.0f, 0.0f );

    // thetaCam   += _CAM_ROT_DG;
    // camOfstDir = make_vec4f( Cosf( thetaCam ), Sinf( thetaCam ), 0.0f );
    
    for( uint i = 0; i < _N_ATTRCTR; ++i ){
        for( ubyte j = 0; j < _N_STP_FRM; ++j ){  step_6D_attractor( attractors[i] );  }
        // If this attractor is unstable, then replace it with new
        h0_i = attractors[i]->edge0[ attractors[i]->bgnDx ];
        h1_i = attractors[i]->edge1[ attractors[i]->bgnDx ];
        if((norm_vec4f( h0_i ) > _DIM_KILL) || (norm_vec4f( h1_i ) > _DIM_KILL)){
            printf( "!! UNSTABLE %p !!\n", attractors[i] );
            delete_6D_attractor( attractors[i] );
            attractors[i] = make_rand_6D_attractor();
        }
        // centroid_AABB_attractor( attractors[i], &attrCntr, &aabbLo, &aabbHi );
        // totCntr = add_vec4f( totCntr, attrCntr );
        // totLo   = add_vec4f( totLo, aabbLo );
        // totHi   = add_vec4f( totHi, aabbHi );
    }
    // totCntr = div_vec4f( totCntr, factor );
    // totLo   = div_vec4f( totLo  , factor );
    // totHi   = div_vec4f( totHi  , factor );
    centroid_AABB_attractor( attractors[0], &attrCntr, &aabbLo, &aabbHi );
    width = norm_vec4f( sub_vec4f( aabbLo  , aabbHi  ) );
    // camOfst = sub_vec4f( cam.lookPt, cam.eyeLoc );
    
    // Set camera targets
    if( width < (_DIM_KILL/12.5f) ){
        cam.lukTgt = attrCntr;
        cam.eyeTgt = add_vec4f(
            attrCntr,
            // stretch_to_len_vec4f( camOfst, (width/2.0f)/Tanf(1.0f*_FOV_DEG/2.0f) )
            stretch_to_len_vec4f( camOfst, width/Tanf(1.0f*_FOV_DEG/2.0f) )
            // stretch_to_len_vec4f( camOfstDir, width/Tanf(1.0f*_FOV_DEG/2.0f) )
        );
    }
    cam_smooth_update( &cam );

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

    look_smooth( &cam );

    for( uint i = 0; i < _N_ATTRCTR; ++i ){
        draw_6D_attractor( attractors[i] );
    }

    heartbeat_FPS( _TARGET_FPS );

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

    vec4f eyeBgn = {750.0f, 750.0f, 350.0f, 1.0f};
    vec4f lukBgn = {  0.0f,   0.0f, 200.0f, 1.0f};
    /*-*/ cam    = get_smooth_camera( eyeBgn, lukBgn, 1.0f/180.0f );

    for( uint i = 0; i < _N_ATTRCTR; ++i ){
        attractors[i] = make_rand_6D_attractor();
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
    glBlendFunc( GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA );
    glEnable( GL_BLEND );

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