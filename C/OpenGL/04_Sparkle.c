////////// INIT ////////////////////////////////////////////////////////////////////////////////////

#include "geometry.h"
#include "matrix4x4f.h"



////////// PROGRAM SETTINGS ////////////////////////////////////////////////////////////////////////

/// View Settings ///
const float _DRAW_DIST_MIN = 10.0f / 16.0f; // Scale Dimension
const float _DRAW_DIST_MAX = 10.0f * 16.0f; // Scale Dimension
const int   _FOV_DEG    =   55; // - Field of view (for perspective)
const float _TARGET_FPS =   60.0f; // Desired framerate
const int   _WINDOW_W   = 1200;
const int   _WINDOW_H   =  900;

/// Model Settings ///
const float _GRID_UNIT    =  1.0f;
const uint  _N_UNIT /*-*/ = 50;
const vec4f _GRID_CLR     = {0.5f, 0.5f, 0.5f, 1.0f};
const float _GRID_THICC   = 3.0f;

////////// PROGRAM STRUCTS /////////////////////////////////////////////////////////////////////////

///// Light Source ////////////////////////////////////////////////////////

typedef struct{
    // Light source for default Phong shading
    uint  ID; // ----- GL light source enum
    vec4f position; // Position in the world frame
    vec4f ambient;
	vec4f diffuse;
	vec4f specular;
}LightSource;


LightSource* make_white_light_source( const vec4f posn, uint sourcEnum, 
                                      int ambientPrcnt, int diffusePrcnt, int specularPrcnt ){
    // White light source
    LightSource* lite = (LightSource*) malloc( sizeof( LightSource ) );
    lite->ID /*-*/ = sourcEnum;
    lite->position = posn;
    lite->ambient  = make_vec4f( 0.01f*ambientPrcnt  , 0.01f*ambientPrcnt , 0.01f*ambientPrcnt  );
    lite->diffuse  = make_vec4f( 0.01f*diffusePrcnt  , 0.01f*diffusePrcnt , 0.01f*diffusePrcnt  );
	lite->specular = make_vec4f( 0.01f*specularPrcnt , 0.01f*specularPrcnt, 0.01f*specularPrcnt );
    return lite;
}


void illuminate_with_source( LightSource* lite ){
    // Use this `lite` in the scene
    float Position[] = { lite->position.x, lite->position.y, lite->position.z, lite->position.w };
    float Ambient[]  = { lite->ambient.r , lite->ambient.g , lite->ambient.b , lite->ambient.a  };
	float Diffuse[]  = { lite->diffuse.r , lite->diffuse.g , lite->diffuse.b , lite->diffuse.a  };
	float Specular[] = { lite->specular.r, lite->specular.g, lite->specular.b, lite->specular.a };
    // glColorMaterial( GL_FRONT_AND_BACK , GL_AMBIENT_AND_DIFFUSE );
	// glEnable( GL_COLOR_MATERIAL );
	//  Enable light 0
	glEnable( lite->ID );
	//  Set ambient, diffuse, specular components and position of light 0
    glLightfv( lite->ID, GL_POSITION , Position );
	glLightfv( lite->ID, GL_AMBIENT  , Ambient  );
	glLightfv( lite->ID, GL_DIFFUSE  , Diffuse  );
	glLightfv( lite->ID, GL_SPECULAR , Specular );

    // glDisable( lite->ID );
}


///// Firework ////////////////////////////////////////////////////////////

typedef struct{
    // A bright projectile that explodes even more brightly, with particles!
    VNCT_f* shell; // --- Shell geometry
    vec4f   color; // --- Shell color
    uint    Nsprk; // --- Number of sparkles
    uint    mode; // ---- 0: Shell, 1: Detonation
    int     sprkShdr; //- Particle system shader
    int     xpldShdr; //- Explosion animation shader
    uint    posArr_ID; // GPU position buffer
    uint    clrArr_ID; // GPU color buffer
}Firework;


void init_particles( Firework* fw ){
    // Get geometry shader_ID info and allocate buffer space on the GPU
    // Adapted from code by: Willem A. (Vlakkies) Schreüder

    vec4f* target = NULL;

    // Initialize position buffer
    glGenBuffers( 1, &(fw->posArr_ID) );
    glBindBuffer( GL_SHADER_STORAGE_BUFFER, fw->posArr_ID );
    glBufferData( GL_SHADER_STORAGE_BUFFER, (fw->Nsprk) * sizeof( vec4f ), NULL, GL_STATIC_DRAW );

    // Initialize color buffer
    glGenBuffers( 1, &(fw->clrArr_ID) );
    glBindBuffer( GL_SHADER_STORAGE_BUFFER, fw->clrArr_ID );
    glBufferData( GL_SHADER_STORAGE_BUFFER, (fw->Nsprk) * sizeof( vec4f ), NULL, GL_STATIC_DRAW );

    glBindBuffer( GL_SHADER_STORAGE_BUFFER, 0 );

    ///// Particle Positions /////////////////////

    glBindBuffer( GL_SHADER_STORAGE_BUFFER, fw->posArr_ID ); // Set buffer object to point to this ID, for writing
    // Get pointer to buffer and cast as a struct array
    target = (vec4f*) glMapBufferRange( GL_SHADER_STORAGE_BUFFER, 0, (fw->Nsprk) * sizeof( vec4f ),
                                        GL_MAP_WRITE_BIT|GL_MAP_INVALIDATE_BUFFER_BIT );
    // Load init positions into buffer
    for( uint i = 0; i < fw->Nsprk; ++i ){  target[i] = make_0_vec4f();  }

    ///// Particle Colors ////////////////////////

    glBindBuffer( GL_SHADER_STORAGE_BUFFER, fw->clrArr_ID ); // Set buffer object to point to this ID, for writing
    // Get pointer to buffer and cast as a struct array
    target = (vec4f*) glMapBufferRange( GL_SHADER_STORAGE_BUFFER, 0, (fw->Nsprk) * sizeof( vec4f ),
                                        GL_MAP_WRITE_BIT|GL_MAP_INVALIDATE_BUFFER_BIT );
    // Load init positions into buffer
    for( uint i = 0; i < fw->Nsprk; ++i ){  target[i] = make_0w0_vec4f();  }
    
    glUnmapBuffer( GL_SHADER_STORAGE_BUFFER ); // Release buffer object
}


Firework* make_Firework( float width, const vec4f bodyClr, uint Nsprk_ ){
    // Setup the firework struct for rendering
    Firework* rtnFw = (Firework*) malloc( sizeof( Firework ) );
    /// CPU-Side ///
    rtnFw->shell    = octahedron_VNC_f( width, 2.0f*width, bodyClr );
    rtnFw->color    = bodyClr;
    rtnFw->Nsprk    = Nsprk_;
    rtnFw->mode     = 0; // Begin in Shell Mode
    rtnFw->sprkShdr = 0; // TBD
    rtnFw->xpldShdr = 0; // TBD
    /// GPU-Side ///
    // Allocate
    allocate_and_load_VBO_VNC_at_GPU( rtnFw->shell );
    init_particles( rtnFw );
    
    // Create shaders
    rtnFw->sprkShdr = CreateShaderGeom( "shaders/basic.vert", "shaders/sparkleTrail.geom", "shaders/basic.frag" );

    /// Return ///
    return rtnFw;
}



////////// PROGRAM STATE ///////////////////////////////////////////////////////////////////////////

/// Geometry ///
Firework*    frwk     = NULL;
bool /*---*/ fwActive = true;
VNCT_f* /**/ grnd     = NULL;
LightSource* lite     = NULL;
Camera3D     cam /**/ = { {0.25f, 0.75f, 0.75f, 1.0f}, {0.0f, 0.0f, 0.0f, 1.0f}, {0.0f, 0.0f, 1.0f, 1.0f} };



////////// WINDOW & VIEW STATE /////////////////////////////////////////////////////////////////////
float w2h = 0.0f; // Aspect ratio


static void project(){
	// Set projection
	// Adapted from code provided by Willem A. (Vlakkies) Schreüder  
	// NOTE: This function assumes that aspect rario will be computed by 'resize'
	// 1. Tell OpenGL we want to manipulate the projection matrix
	glMatrixMode( GL_PROJECTION );
	//  Undo previous transformations
	glLoadIdentity();
	gluPerspective( _FOV_DEG , // ------ Field of view angle, in degrees, in the y direction.
					w2h , // ----------- Aspect ratio , the field of view in the x direction. Ratio of x (width) to y (height).
					_DRAW_DIST_MIN , //- Specifies the distance from the viewer to the near clipping plane (always positive).
					_DRAW_DIST_MAX ); // Specifies the distance from the viewer to the far clipping plane (always positive).
	// 2. Switch back to manipulating the model matrix
	glMatrixMode( GL_MODELVIEW );
	// 3. Undo previous transformations
	glLoadIdentity();
}


void reshape( int width , int height ){
	// GLUT calls this routine when the window is resized
    // Adapted from code provided by Willem A. (Vlakkies) Schreüder  
	// 1. Calc the aspect ratio: width to the height of the window
	w2h = ( height > 0 ) ? (float) width / height : 1;
	// 2. Set the viewport to the entire window
	glViewport( 0 , 0 , width , height );
	// 3. Set projection
	project();
}



////////// RENDERING ///////////////////////////////////////////////////////////////////////////////

void display(){
    // Refresh display

    //  Erase the window and the depth buffer
    glClearDepth( 1.0f );
    glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT | GL_STENCIL_BUFFER_BIT );
    glEnable( GL_DEPTH_TEST );
    glLoadIdentity();

    ///// DRAW LOOP BEGIN /////////////////////////////////////////////////
    
	//  Enable lighting , From this point until 'glDisable' lighting is applied
	glEnable( GL_LIGHTING );
    glColorMaterial( GL_FRONT_AND_BACK , GL_AMBIENT_AND_DIFFUSE );
	glEnable( GL_COLOR_MATERIAL );

    look( cam ); // ------------------ NOTE: Look THEN illuminate!
    illuminate_with_source( lite ); // NOTE: Illumination THEN transformation!

    // printf( "About to draw ground ...\n" );
    draw_VNC_f( grnd );

    if( fwActive ){  
        // printf( "About to draw firework ...\n" );
        draw_VNC_f( frwk->shell );
    }
        
    glDisable( GL_LIGHTING );
    ///// DRAW LOOP END ///////////////////////////////////////////////////

    //  Display parameters
    glDisable( GL_DEPTH_TEST );
    glWindowPos2i( 5, 5 );
    glColor3f( 1.0f, 1.0f, 1.0f );
    Print( "FPS=%f", heartbeat_FPS( _TARGET_FPS ) );

    // Check for errors, Flush, and swap
	ErrCheck( "display" );
	glFlush();
	glutSwapBuffers();

}



////////// INTERACTION /////////////////////////////////////////////////////////////////////////////

/* KB + MOUSE INPUT GOES HERE */



////////// SIMULATION //////////////////////////////////////////////////////////////////////////////

void tick(){
    // Background work
    
    ///// TICK LOOP BEGIN /////////////////////////////////////////////////
    



    ///// TICK LOOP END ///////////////////////////////////////////////////
    
    // 3. Set projection
	project();
    // Tell GLUT it is necessary to redisplay the scene
	glutPostRedisplay();
}



////////// MAIN ////////////////////////////////////////////////////////////////////////////////////


int main( int argc, char* argv[] ){

    init_rand();

    ///// Initialize GLUT /////////////////////////////////////////////////

    glutInit( &argc , argv );
    // initGL();

    // Request window with size specified in pixels
    glutInitWindowSize( _WINDOW_W, _WINDOW_H );

    // Create the window
    glutCreateWindow( "Firework with Particle (Geometry) Shader" );

    // NOTE: Set modes AFTER the window / graphics context has been created!
    // Request double buffered, true color window 
    glutInitDisplayMode( GLUT_RGB | GLUT_DOUBLE | GLUT_DEPTH );
    
    glEnable( GL_CULL_FACE );
    //  OpenGL should normalize normal vectors
	glEnable( GL_NORMALIZE );
    glDepthRange( 0.0f , 1.0f ); 
    glutSetCursor( GLUT_CURSOR_NONE ); // Hide the cursor while in the window
    glutWarpPointer( _WINDOW_W/2, _WINDOW_H/2 );


    ///// Initialize Geometry /////////////////////////////////////////////
    vec4f gClr = make_vec4f( 31.0f/255.0f, 120.0f/255.0f, 55.0f/255.0f );
    vec4f fClr = make_vec4f( 1.0, 0.0f, 0.0f );
    vec4f mLoc = make_vec4f( 50.0f, 0.0f, 50.0f );
    
    printf( "About to make ground ...\n" );
    grnd = plane_XY_VNC_f( 2.0f*_GRID_UNIT*_N_UNIT, 2.0f*_GRID_UNIT*_N_UNIT, _N_UNIT, _N_UNIT, gClr );
    allocate_and_load_VBO_VNC_at_GPU( grnd );
    
    printf( "About to make light ...\n" );
    lite = make_white_light_source( stretch_to_len_vec4f( mLoc, 10.0f ), GL_LIGHT0, 5, 50, 100 );
    
    printf( "About to make firework ...\n" );
    frwk = make_Firework( 0.070, fClr, 1024 );

    ///// Initialize GLUT Callbacks ///////////////////////////////////////
    printf( "About to assign callbacks ...\n" );

    //  Tell GLUT to call "display" when the scene should be drawn
    glutDisplayFunc( display );

    // Tell GLUT to call "idle" when there is nothing else to do
    glutIdleFunc( tick );
    
    //  Tell GLUT to call "reshape" when the window is resized
    glutReshapeFunc( reshape );
    
    //  Tell GLUT to call "special" when an arrow key is pressed or released
    // glutSpecialFunc( special_dn );
    // glutSpecialUpFunc( special_up );

    //  Tell GLUT to call "mouse" when mouse input arrives
    // glutMouseFunc( mouse ); // Clicks
    // glutPassiveMotionFunc( mouse_move ); // Movement
    
    // //  Tell GLUT to call "key" when a key is pressed or released
    // glutKeyboardFunc( key_dn );
    // glutKeyboardUpFunc( key_up );

    ///// GO ///// GO ///// GO ////////////////////////////////////////////
    printf( "Entering main loop ...\n" );
    
    // Pass control to GLUT so it can interact with the user
    glutMainLoop();
    
    
    ///// Free Memory /////////////////////////////////////////////////////
    printf( "Cleanup!\n" );
    
    delete_VNCT_f( grnd );
    delete_VNCT_f( frwk->shell );

    printf( "\n### DONE ###\n\n" );
    //  Return code
    return 0;
}