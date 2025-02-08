#ifndef SFM_HPP // This pattern is to prevent symbols to be loaded multiple times
#define SFM_HPP // from multiple imports

////////// INIT ////////////////////////////////////////////////////////////////////////////////////
#include "SfM.hpp"



/////////// GRAPHICS HELPERS ///////////////////////////////////////////////////////////////////////

bool ErrCheck( const char* where ){
	// See if OpenGL has raised any errors
	// Author: Willem A. (Vlakkies) Schreüder  
	int err = glGetError();
	if( err ){  
		fprintf( stderr , "ERROR: %s [%s]\n" , gluErrorString( err ) , where );  
		return true;
	}else{  return false;  }
}


/////////// GRAPHICS STRUCTS ///////////////////////////////////////////////////////////////////////

///// Camera3D ////////////////////////////////////////////////////////////

Camera3D::Camera3D(){
    eyeLoc = {1.0, 1.0, 1.0};
    lookPt = {0.0, 0.0, 0.0};
    upVctr = {0.0, 0.0, 1.0}; // Up is +Z, I WILL FIGHT YOU
}


void Camera3D::look(){
    // Set camera position, target, and orientation
    gluLookAt( eyeLoc[0], eyeLoc[1], eyeLoc[2],  
               lookPt[0], lookPt[1], lookPt[2],  
               upVctr[0], upVctr[1], upVctr[2] );
}


void Camera3D::set_position( const vec3d& loc ){
    // Move the camera to `loc` and set view
    eyeLoc = loc;
    look();
}


void Camera3D::set_target( const vec3d& target ){
    // Point the camera at `target` and set view
    lookPt = target;
    look();
}


vec3d operator-( const vec3d& left, const vec3d& right ){
    // Subtract two 3D vectors
    return vec3d{
        left[0] - right[0],
        left[1] - right[1],
        left[2] - right[2]
    };
}


vec3d operator-( const vec3d& vec ){
    // Return the opposite of `vec`
    return vec3d{ -vec[0], -vec[1], -vec[2] };
}


vec3d operator+( const vec3d& left, const vec3d& right ){
    // Add two 3D vectors
    return vec3d{
        left[0] + right[0],
        left[1] + right[1],
        left[2] + right[2]
    };
}


vec3d operator/( const vec3d& left, double right ){
    // Divide a 3D vector by a scalar
    return vec3d{
        left[0] / right,
        left[1] / right,
        left[2] / right
    };
}


vec3d operator*( const vec3d& left, double right ){
    // Divide a 3D vector by a scalar
    return vec3d{
        left[0] * right,
        left[1] * right,
        left[2] * right
    };
}


vec3d cross( const vec3d& a, const vec3d& b ){
    // Compute a X b
    return vec3d{
        a[1]*b[2] - a[2]*b[1],
        a[2]*b[0] - a[0]*b[2],
        a[0]*b[1] - a[1]*b[0]
    };
}


double norm( const vec3d& cartCoords ){
    // Length of a 3D vector
    return sqrt( pow( cartCoords[0], 2 ) + pow( cartCoords[1], 2 )  + pow( cartCoords[2], 2 ) );
}


vec3d unit( const vec3d& vec ){
    // Return the unit direction of the vector
    double mag = norm( vec );
    if( mag > 0.0 ){  return vec / mag; }
    else{  return vec;  }
}


vec3d cart_2_spherical( const vec3d& cartCoords ){
    // Cartesian to Spherical Coordinates
    double radius = norm( cartCoords );
    double theta  = atan2( cartCoords[1], cartCoords[0] );
    double xyLen  = sqrt( pow( cartCoords[0], 2 ) + pow( cartCoords[1], 2 ) );
    double phi    = atan2( cartCoords[2], xyLen );
    return vec3d{ radius, theta, phi };
}


vec3d spherical_2_cart( const vec3d& sphrCoords ){
    // Spherical to Cartesian Coordinates
    double radius = sphrCoords[0];
    double xyLen  = radius * cos( sphrCoords[2] );
    return vec3d{
        xyLen  * cos( sphrCoords[1] ),
        xyLen  * sin( sphrCoords[1] ),
        radius * sin( sphrCoords[2] )
    };
}


void Camera3D::rotate_spherical( double delTheta, double delPhi ){
    // Rotate the eye about the target in spherical coordinates
    // 1. Move the eye
    vec3d antiLookSeg = eyeLoc - lookPt;
    vec3d antiLookSph = cart_2_spherical( antiLookSeg );
    antiLookSph[1] += delTheta;
    antiLookSph[2] += delPhi;
    vec3d eyeDiff = spherical_2_cart( antiLookSph );
    eyeLoc = lookPt + eyeDiff;
    // 2. Recalc "up"
    // NOTE: This function assumes that the caller does not exactly rotate the camera between horizontal and vertical in one step
    vec3d xBasis = -eyeDiff;
    vec3d yBasis = cross( upVctr, xBasis );
    upVctr = unit( cross( xBasis, yBasis ) );
}


void Camera3D::zoom_spherical( double delDist ){
    // Move the camera along the line of sight while preserving target and up, Enforce non-negative distance
    vec3d  antiLookSeg = eyeLoc - lookPt;
    double antiLookMag = norm( antiLookSeg );
    vec3d  antiLookDir = unit( antiLookSeg );
    antiLookMag = max( antiLookMag + delDist, 0.0 );
    antiLookSeg = antiLookDir * antiLookMag;
    eyeLoc = lookPt + antiLookSeg;
}



///// OGL_window //////////////////////////////////////////////////////////
static OGL_window instance;

OGL_window::OGL_window( VIEWMODE view ){
    CURRVIEW = view;
    dim /**/ =  1.0; 
    w2h /**/ =  1.0; 
    fov /**/ = 55.0; 
    cam /**/ = Camera3D();
}


void OGL_window::create( int width , int height, string name ){
    // Actually create the window
    cout << "About to init ..." << endl;
    char fakeParam[] = "fake";
    char *fakeargv[] = { fakeParam, NULL };
    int fakeargc = 1;
    glutInit( &fakeargc, fakeargv );
    cout << "About to set mode ..." << endl;
    //  Request double buffered, true color window 
    glutInitDisplayMode( GLUT_RGB | GLUT_DOUBLE | GLUT_DEPTH );
    //  Request W x H pixel window
    glutInitWindowSize( width, height );
    cout << "About to make window ..." << endl;
    //  Create the window
    glutCreateWindow( name.c_str() );
    cout << "About to make enable ..." << endl;
    // Don't ask questions!
    glEnable( GL_DEPTH_TEST );
    glDepthRange( 0.0f , 1.0f );
}


void OGL_window::Project(){
    // Set projection
    // Adapted from code provided by Willem A. (Vlakkies) Schreüder  
    
    //  Tell OpenGL we want to manipulate the projection matrixrestart
    glMatrixMode( GL_PROJECTION );
    //  Undo previous transformations
    glLoadIdentity();
    
    switch( CURRVIEW ){
        case ORTHO:
            //  aspect ratio of the window
            glOrtho( -dim * w2h , +dim * w2h , 
                     -dim       , +dim       , 
                     -dim       , +dim       );
        
            break;
        case PERSP:
            gluPerspective( (double) fov , // -- Field of view angle, in degrees, in the y direction.
                            (double) w2h , // -- Aspect ratio , the field of view in the x direction. Ratio of x (width) to y (height).
                            (double) dim/16.0 , //- Specifies the distance from the viewer to the near clipping plane (always positive).
                            (double) 4.0*dim ); // Specifies the distance from the viewer to the far clipping plane (always positive).
            break;
    }
    
    // Switch back to manipulating the model matrix
    glMatrixMode( GL_MODELVIEW );
    // Undo previous transformations
    glLoadIdentity();
}


void OGL_window::OGL_frame_start(){
	// Do this before drawing anything
	// Clear the image
	glClearDepth( 1.0f );
	glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );
	// Reset previous transforms to the identity matrix
	glLoadIdentity();
}


void OGL_window::OGL_frame_end(){
	// Do this after drawing everything, Flush and swap
	glFlush();
	glutSwapBuffers();
}


void reshape( int width, int height ){
    // GLUT calls this routine when the window is resized
    // Ratio of the width to the height of the window
    instance.w2h = (height > 0) ? (float) width/height : 1;
    // Set the viewport to the entire window
    glViewport( 0, 0, width, height );
    // Set projection
    instance.Project();
}


void dummy_cb(){  return;  }


void idle(){
    // Run this when the user isn't doing anything
	instance.idle_cb();
	// Reproject
	instance.Project();
	// Tell GLUT it is necessary to redisplay the scene
	glutPostRedisplay();
}


void display(){
    // Don't bother the user with frame setup and cleanup
    cout << "! DRAW !" << endl;
    instance.cam.look();
    instance.OGL_frame_start();
    instance.draw_cb();
    instance.OGL_frame_end();
}


void OGL_window::set_callbacks( void (*draw_cb_)(), void (*idle_cb_)() ){
    cout << "About to set callbacks ..." << endl;
    draw_cb = draw_cb_;
    idle_cb = (idle_cb_ ? idle_cb_ : dummy_cb );

    // Tell GLUT to call "display" when the scene should be drawn
	glutDisplayFunc( display );

    // Tell GLUT to call "idle" when there is nothing else to do
	glutIdleFunc( idle );

    // Tell GLUT to call "reshape" when the window is resized
	glutReshapeFunc( reshape );
}


OGL_window& OGL_window::make_window( int width , int height, string name ){
    cout << "About to instantiate ..." << endl;
    instance = OGL_window{};
    cout << "About to create ..." << endl;
    instance.create( width, height, name );
    return instance;
}


void OGL_window::run(){
    // Check for errors
    uchar err = ErrCheck( "run" );
    if( err ){  cout << "Error Code: " << err << endl;  }
    // Pass control to GLUT so it can interact with the user
	glutMainLoop();
}


void OGL_window::set_eye_target( const Point3d& eye, const Point3d& target ){
    cam.set_position( vec3d{ eye.x, eye.y, eye.z } );
    cam.set_target( vec3d{ target.x, target.y, target.z } );
}

////////// END /////////////////////////////////////////////////////////////////////////////////////
#endif;