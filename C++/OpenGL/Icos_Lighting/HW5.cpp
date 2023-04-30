/*****************************
 HW5.cpp
 James Watson, 2018 October
 Lighting Demo
 ****************************/ 
/*
~~ DEV PLAN ~~
[Y] Gut old program
[Y] Eigen switch
	[Y] Create a Math / Geo file
	[Y] Shortcuts for Eigen types
	[Y] Compiler flags for float / double
[Y] Put the robot in its own file
[Y] Put the creatures in their own file
[Y] Compile "Empty" program with imports - Did the refactor succeed?

[Y] Rebuild Grid Scene
[Y] Check for GL errors
[Y] Create a Light
[Y] Create an icosahedron
	[Y] Define material properties for the icosahedron
	[Y] Illuminate the icosahedron
[Y] Add light interactivity per instructions
	[Y] Light orbit control , Auto
	[Y] Light position control , Manual
	[ ] Make properties of light changeable per the example
		[Y] ambient   settable
		[Y] diffuse   settable
		[Y] specular  settable
		[Y] shininess settable
		[Y] emission  settable 
			[Y] Choose object(s) that will emit
[Y] Create a version of cylinder that can be illuminated
	[Y] Normals
	[Y] settable properties
[Y] Virus capsid
	[Y] Cylinder out of each face of the icos
	[Y] Tip each cylinder with an icosahedron
	[Y] Pleasing colors
	[ ] Reactive center
		[Y] Settable emission
		{ } Sparklies from center
{ } Create an illuminated robot
	{ } Pick shiny material props for the arm beams
{ } Robot IK
{ } Camera at robot gripper


*/

// === INIT ================================================================================================================================

// === Includes & Defines ===

// ~~ Standard ~~
#include <stdio.h> //- Streams to communicate with devices such as keyboards, printers, terminals or with any other type of files supported 
#include <stdlib.h> // defines four variable types, several macros, and various functions for performing general functions. , size_t
#include <stdarg.h> // macros to access individual args of a list of unnamed arguments whose number and types are not known to the called function

// ~~ OpenGL ~~

// OpenGL with prototypes for glext
// NOTE: This MUST appear before '#include <GL/glext.h>' , otherwise will not work!
#define GL_GLEXT_PROTOTYPES // Important for all of your 5229 programs

// ~~ System-Specific Includes ~~
#ifdef __APPLE__ // This constant is always defined on Apple machines
	#include <GLUT/glut.h> // GLUT is in a different place on Apple machines
#else
	#include <GL/glut.h>
	#include <GL/glext.h>	
#endif

// ~~ Local ~~
#include <Cpp_Helpers.h> // Favorite C++ tricks! I am the author , Source: https://bitbucket.org/jwatson_utah_edu/cpp_helpers/src/master/
#include "MathGeo.h" // ___ Eigen + Math Utils
#include "OGL_utils.h" // _ Utility functions for 5229
#include "ToyBox.h" // ____ Objects and Critters for Interesting Times

// ___ End Include ___

// === GLOBALS ===

// ~~ Assignment ~~
string HWname = "HW5";

// ~~ View ~~
float dim /* --- */ =  2.0; // Scale Dimension
// ~ Screen ~
float w2h = 0.0f; // Aspect ratio
// ~ Camera ~
float camRadius     =    2.35; // --- Distance of the camera from [0,0,0]
float CAMRADINCR    =    0.0625; // - Zoom in and out by this far each keypress
int   DFLT_THETA    = -315; // ----- Initial rotation for viewing
int   DFLT_PSI      =   25; // ----- Initial elevation for viewing
int   VIEW_DEG_INCR =    5; // ----- View angle change for every key press
int   th /* ---- */ = DFLT_THETA; // Azimuth of view angle
int   ps /* ---- */ = DFLT_PSI; // - Elevation of view angle
int   fov /* --- */ = 55; // ------- Field of view (for perspective)
vec3e eyeLoc{0,0,0}; // ------------ Camera location (world frame)
vec3e lookPt{0,0,0}; // ------------ Focus of camera (world frame)
vec3e upVctr{0,0,0}; // ------------ Direction of "up"
vec3e lookDr; // ------------------- Direction that the camera is looking (not always used)

// ~~ Scene ~~
vec3e gridColor{ 165.0/255 , 189.0/255 , 226.0/255 }; // Color of the world grid lines
// ~ Light Settings ~
int   emission  =   0; // Emission intensity (%)
int   ambient   =  30; // Ambient intensity (%)
int   diffuse   = 100; // Diffuse intensity (%)
int   specular  =   0; // Specular intensity (%)
int   shininess =   0; // Shininess (power of two)
float shiny     =   1; // Shininess (value)
// ~ Example Light source ~
float BALL_ORBIT_INCR = 2.5; // ---- Angle increase for each button press
float ballOrbitSpeed  = 45.0; // --- Orbit speed of ball [deg/s]
float th_ball /* - */ = 0.0f; // --- Ball azimuth angle
float ps_ball /* - */ = -10.0; // Ball elevation angle
float rad_bal /* - */ = 1.25; // ---- Radius of ball from origin
vec3e center_ball{0,0,0}; // ------- Location of the light ball in space
int   emsn_ball       =  20; // ----- Emission intensity (%)
float shiny_ball      =   1; // ----- Shininess (value)
float dimRad_ball     =   0.050; // - Radius of the ball itself

// ~~ Data ~~


// ~~ Control ~~
// ~ Flags ~
bool BALLMOVAUTO = true;
bool ANIMATBEAMS = true;
// ~ Enums ~

// ~~ Geometry ~~
float AXESSCALE     =   0.17;
int   BALL_ANG_INCR =  10;


// ___ END GLOBAL ___

// ___ END INIT ____________________________________________________________________________________________________________________________


// === FUNCTIONS ===========================================================================================================================

static void light_ball( float x , float y , float z , float r ,
						int emission , float shiny ){
	// Draw a glowing white ball at (x,y,z) radius (r)
	// Author: Willem A. (Vlakkies) Schreüder  
	int th , ph;
	float yellow[] = { 1.0f , 1.0f , 0.0f , 1.0f };
	float Emission[] = { 0.0f , 0.0f , 0.01f * emission , 1.0f };
	//  Save transformation
	glPushMatrix();
	//  Offset, scale and rotate
	glTranslated( x , y , z );
	glScaled( r , r , r );
	//  White ball
	glColor3f( 1 , 1 , 1 );
	glMaterialf( GL_FRONT_AND_BACK , GL_SHININESS , shiny );
	glMaterialfv( GL_FRONT_AND_BACK , GL_SPECULAR , yellow );
	glMaterialfv( GL_FRONT_AND_BACK , GL_EMISSION , Emission );
	//  Bands of latitude
	for( ph =- 90 ; ph < 90 ; ph += BALL_ANG_INCR ){
		glBegin( GL_QUAD_STRIP );
		for( th = 0 ; th <= 360 ; th += 2 * BALL_ANG_INCR ){
			Vertex_sphr( th , ph );
			Vertex_sphr( th , ph + BALL_ANG_INCR );
		}
		glEnd();
	}
	//  Undo transofrmations
	glPopMatrix();
}

// ___ END FUNC ____________________________________________________________________________________________________________________________


// === CLASSES =============================================================================================================================




// ___ END CLASS ___________________________________________________________________________________________________________________________


// === VARIABLES & OBJECTS =================================================================================================================
//						  float rad , const vec3e& cntr , const vec3e& colr , float shiny
Icosahedron_OGL icosTest{ 0.5 , vec3e{0,0,0} , vec3e{0,1,0} , 5.5 };
std::vector<Icosahedron_OGL*> nodules;
vec3e RXcolor{ 0.0/255 , 204.0/255 , 102.0/255 };
std::vector<RibbonBolt*> particles;

// ___ END VAR _____________________________________________________________________________________________________________________________


// === DRAWING =============================================================================================================================

static void Project(){
	// Set projection
	// Adapted from code provided by Willem A. (Vlakkies) Schreüder  
	// NOTE: This function assumes that aspect rario will be computed by 'resize'
	
	//  Tell OpenGL we want to manipulate the projection matrix
	glMatrixMode( GL_PROJECTION );
	//  Undo previous transformations
	glLoadIdentity();
	
	gluPerspective( fov , // -- Field of view angle, in degrees, in the y direction.
					w2h , // -- Aspect ratio , the field of view in the x direction. Ratio of x (width) to y (height).
					dim/4 , //- Specifies the distance from the viewer to the near clipping plane (always positive).
					4*dim ); // Specifies the distance from the viewer to the far clipping plane (always positive).
	
	// Switch back to manipulating the model matrix
	glMatrixMode( GL_MODELVIEW );
	// Undo previous transformations
	glLoadIdentity();
}

void display(){
	// Display the scene
	
	//  Clear the image
	glClearDepth( 1.0f );
	glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );
	
	//  Reset previous transforms to the identity matrix
	glLoadIdentity();
	
	// ==== Redraw ====
	
	// 1. Calculate the user view
	// Calc the eye position for perspective view (Orbit [0,0,0] spherically)
	eyeLoc = vec_sphr( camRadius , th , ps );
	lookPt = vec3e{0,0,0};
	lookDr = lookPt - eyeLoc;
	upVctr = ( lookDr.cross( vec3e{0,0,1}.cross( lookDr ) ) ).normalized();
	gluLookAt( eyeLoc[0] , eyeLoc[1] , eyeLoc[2] ,  
			   lookPt[0] , lookPt[1] , lookPt[2] ,  
			   upVctr[0] , upVctr[1] , upVctr[2] );
			   

	// 2. Draw the static scene
	// Draw a grid
	glPushMatrix();
	glTranslated( 0,0,-1 );
	draw_grid_org_XY( 0.250 , 20 , 20 , 0.5 , gridColor );
	glPopMatrix();
			   
	// 2. Lighting Calcs
	//  Translate intensity to color vectors
	float Ambient[]  = { 0.01f*ambient  , 0.01f*ambient  , 0.01f*ambient  , 1.0f };
	float Diffuse[]  = { 0.01f*diffuse  , 0.01f*diffuse  , 0.01f*diffuse  , 1.0f };
	float Specular[] = { 0.01f*specular , 0.01f*specular , 0.01f*specular , 1.0f };
	//  Draw light position as ball (still no lighting here)
	// Calculate the ball position and render ball
	center_ball = vec_sphr( rad_bal , th_ball , ps_ball );
	light_ball( center_ball(0) , center_ball(1) , center_ball(2) , dimRad_ball ,
				emission , shiny );
	//  Light position
	float Position[]  = { center_ball(0) , center_ball(1) , center_ball(2) , 1.0f };
	//  OpenGL should normalize normal vectors
	glEnable( GL_NORMALIZE );
	//  Enable lighting , From this point until 'glDisable' lighting is applied
	glEnable( GL_LIGHTING );
	glColorMaterial( GL_FRONT_AND_BACK , GL_AMBIENT_AND_DIFFUSE );
	glEnable( GL_COLOR_MATERIAL );
	//  Enable light 0
	glEnable( GL_LIGHT0 );
	//  Set ambient, diffuse, specular components and position of light 0
	glLightfv( GL_LIGHT0 , GL_AMBIENT  , Ambient  );
	glLightfv( GL_LIGHT0 , GL_DIFFUSE  , Diffuse  );
	glLightfv( GL_LIGHT0 , GL_SPECULAR , Specular );
	glLightfv( GL_LIGHT0 , GL_POSITION , Position );
	
	// 3. Draw the dynamic scene
	
	//  Enable light 1
	glEnable( GL_LIGHT1 );
	float virsCntr[]  = { 0.0f , 0.0f , 0.0f , 1.0f };
	glLightfv( GL_LIGHT1 , GL_POSITION , virsCntr );
	float virsColr[] = { RXcolor(0) * 0.01f * emission , RXcolor(1) * 0.01f * emission , RXcolor(2) * 0.01f * emission , 1.0f };
	float virsSpec[] = { RXcolor(0) * 0.01f * emission * 0.01f * specular , 
						 RXcolor(1) * 0.01f * emission * 0.01f * specular , 
						 RXcolor(2) * 0.01f * emission * 0.01f * specular , 
						 1.0f };
	glLightfv( GL_LIGHT1 , GL_DIFFUSE  , virsColr );
	glLightfv( GL_LIGHT1 , GL_SPECULAR , virsSpec );
	icosTest.set_emission_intensity( emission );
	icosTest.draw( shiny );
				   
	// 4. For each of the faces
	uint len = icosTest.icosGeo.N.rows();
	float angle;
	vec3e axis;
	vec3e zPos{0,0,1};
	vec3e nrml;
	for( uint i = 0 ; i < len ; i++ ){
		// 5. Find the turn to the face
		nrml  = icosTest.icosGeo.N.row(i);
		angle = angle_between( nrml , zPos );
		axis  = zPos.cross( nrml ).normalized();
		// 6. Push
		glPushMatrix();
		// 7. Rotate
		glRotated( degrees( angle ) , axis(0) , axis(1) , axis(2) );
		// 8. Draw cylinder 
		draw_cylinder( vec3e{ 0 , 0 , 0.5 }  , 0.35 , 0.07 , 75 ,
				       vec3e{ 0.0/255, 102.0/255, 255.0/255 } , shiny );
		
		glTranslated( 0,0,1 );
		
		nodules[i]->draw( shiny );
		
		// 9. Pop
		glPopMatrix();
	}
	
	// 10. draw particles
	for( uint i = 0 ; i < 20 ; i++ ){  particles[i]->draw();  }
	
	glDisable( GL_LIGHTING );
	
	// == Status Message ==

	// NOTE: Text color MUST be specified before raster position for bitmap text
	// https://www.opengl.org/archives/resources/features/KilgardTechniques/oglpitfall/
	
	glColor3f( 249/255.0 , 255/255.0 , 99/255.0 ); // Text Yellow
	
	//  Display status
	glWindowPos2i( 5 , 5 ); // Next raster operation relative to lower lefthand corner of the window
	
	Print( " emission %i , ambient %i , diffuse %i , specular %i , shininess %i | theta %i , psi %i" , 
		   emission , ambient , diffuse , specular , shininess ,
		   th , ps );
		   
	// __ End Message __

	// ____ End Draw ____

	// Check for errors, Flush, and swap
	ErrCheck( "display" );
	glFlush();
	glutSwapBuffers();
	
}

// ___ END DRAW ____________________________________________________________________________________________________________________________


// === INTERACTION =========================================================================================================================



void key( unsigned char ch , int x , int y ){
	// GLUT calls this routine when a key is pressed
	//  Exit on ESC
	
	switch( ch ){
		
		// ~~ Rotate Ctrl ~~
		
		case 27 : // [Esc] : Exit
			exit( 0 );
			break;
			
		case '0' : // 0 : Set view angles to 0
			th = DFLT_THETA;
			ps = DFLT_PSI;
			printf( "theta and psi reset!\n" );
			break;
		
		// ~~ Program Controls ~~
		
		// ~ Light Control ~
		
		case '[':
			ps_ball -= BALL_ORBIT_INCR;
			break;
		
		case ']':
			ps_ball += BALL_ORBIT_INCR;
			break;
			
		case 'm':
			toggle( BALLMOVAUTO );
			break;
			
		case '\'':
			th_ball -= BALL_ORBIT_INCR;
			break;
		
		case '\\':
			th_ball += BALL_ORBIT_INCR;
			break;
			
		// ~ Light Properties ~
		
		// Ambient level
		case 'a':
			if( ambient >   0 ) ambient -= 5;
			break;
		case 'A':
			if( ambient < 100 ) ambient += 5;
			break;
		// Diffuse level
		case 'd':
			if( diffuse >   0 ) diffuse -= 5;
			break;
		case 'D':
			if( diffuse < 100 ) diffuse += 5;
			break;
		// Specular level
		case 's':
			if( specular >   0 ) specular -= 5;
			break;
		case 'S':
			if( specular < 100 ) specular += 5;
			break;
		// Emission level
		case 'e':
			if( emission >   0 ) emission -= 5;
			break;
		case 'E':
			if( emission < 100 ) emission += 5;
			break;
		//  Shininess level
		case 'n':
			if( shininess > -1 ) shininess -= 1;
			break;
		case 'N':
			if( shininess <  7 ) shininess += 1;
			break;
		//  Translate shininess power to value (-1 => 0)
		shiny = shininess<0 ? 0 : pow(2.0,shininess);
			
		// ~ Light Presets ~
		case 'o': // October Mode
			emission  = 95;
			ambient   =  0;
			diffuse   =  5;
			specular  =  5;
			shininess =  1;
			break;
		case 'p': // Pretty Mode
			emission  =  0;
			ambient   = 25;
			diffuse   = 90;
			specular  =  5;
			shininess =  3;
			break;
			
		// ~ Other ~
		case 'b': // Toggle beams
			toggle( ANIMATBEAMS );
			break;
			
		// <?> : Keys are nice, I guess!
			
		default :
			printf( "There is no function for this key!\n" );
		
	}
	// Reproject
	Project();
	// Tell GLUT it is necessary to redisplay the scene
	glutPostRedisplay();
}

void special( int key , int x , int y ){
	// GLUT calls this routine when an arrow key is pressed
	
	switch( key ){
		
		// ~~ Orbit Camera for Perspective View ~~
	
		case GLUT_KEY_RIGHT: // Right arrow key - increase azimuth by 5 degrees
			th += VIEW_DEG_INCR;
			break;
			
		case GLUT_KEY_LEFT: // Left arrow key - decrease azimuth by 5 degrees
			th -= VIEW_DEG_INCR;
			break;
			
		case GLUT_KEY_UP: // Up arrow key - increase elevation by 5 degrees
			ps += VIEW_DEG_INCR;
			break;
			
		case GLUT_KEY_DOWN: // Down arrow key - decrease elevation by 5 degrees
			ps -= VIEW_DEG_INCR;
			break;
			
		case GLUT_KEY_PAGE_DOWN: // Page Down Key - Zoom Out
			camRadius += CAMRADINCR;
			break;
		
		case GLUT_KEY_PAGE_UP: // Page Up Key - Zoom in
			camRadius -= CAMRADINCR;
			break;
			
		// ~~ Light Control ~~
			
		case GLUT_KEY_END: // End Key - Decrease ball radius from [0,0,0]
			rad_bal -= CAMRADINCR;
			break;
		
		case GLUT_KEY_HOME: // Home Key - Increase ball radius from [0,0,0]
			rad_bal += CAMRADINCR;
			break;
		
		// <?> : Keys are nice, I guess!
			
		default :
			printf( "There is no function for this key!\n" );
		
	}
		
	//  Keep angles to +/-360 degrees
	th %= 360;
	//~ ps %= 360;
	ps = clamp_val( ps , -85 ,  85 ); 
	// Reproject
	Project();
	// Tell GLUT it is necessary to redisplay the scene
	glutPostRedisplay();
}

void reshape( int width , int height ){
	// GLUT calls this routine when the window is resized
	// Calc the aspect ratio: width to the height of the window
	w2h = ( height > 0 ) ? (float) width / height : 1;
	// Set the viewport to the entire window
	glViewport( 0 , 0 , width , height );
	// Set projection
	Project();
}

// ___ END INTERACT ________________________________________________________________________________________________________________________


// === SIMULATION ==========================================================================================================================

double _time_elapsed;
double _last_time = 0.0;

void idle(){
	// Simulation updates in between repaints
	_time_elapsed = glutGet( GLUT_ELAPSED_TIME ) / 1000.0 - _last_time;
	_time_elapsed = clamp_val( _time_elapsed , 0.0 , 1.0 / 60 );
	
	// Update light ball
	if( BALLMOVAUTO ){  th_ball += _time_elapsed * ballOrbitSpeed;  }
	th_ball = fmod( th_ball , 360.0f );
	
	// Set number of particles active in proportion to emissitivity of the core
	// Core begins to emit rays at emissitivity >= 50
	uint numActive = (uint) ( emission > 49 ? ( emission * 1.5 / 5 ) : 0 ); 
	// Update particles
	for( uint i = 0 ; i < 20 ; i++ ){
		if( ANIMATBEAMS ){
			if( i < numActive ){
				particles[i]->activate(); 
				particles[i]->set_emission_intensity( emission );
				particles[i]->advance( _time_elapsed );
			}else{
				particles[i]->deactivate(); 
			}
		}else{  particles[i]->deactivate();  }
	}
	
	//  Tell GLUT it is necessary to redisplay the scene
	glutPostRedisplay();
	_last_time = glutGet( GLUT_ELAPSED_TIME ) / 1000.0;
}

// ___ END SIM _____________________________________________________________________________________________________________________________


// === MAIN ================================================================================================================================

// Start up GLUT and tell it what to do
int main( int argc , char* argv[] ){
	rand_init(); // initialize random seed based on system clock
	
	// Start in Pretty Mode
	emission  =  0;
	ambient   = 25;
	diffuse   = 90;
	specular  =  5;
	shininess =  3;
	
	// ~~ Init Work ~~
	icosTest.set_emission_color( RXcolor );
	
	float lenTravelMin = 0.5f;
	float lenTravelMax = 2.0f;
	float travelSpeed  = 1.75f;
	float width /* -*/ = 0.0075;
	float lengthMin    = 0.010;
	float lengthMax    = 1.5f;
	
	for( uint i = 0 ; i < 20 ; i++ ){
		
		nodules.push_back(  
			new Icosahedron_OGL( 0.125 , vec3e{0,0,0} , vec3e{255.0/255, 102.0/255, 0.0/255} , 5.5 )
		);
		
		particles.push_back(
			new RibbonBolt( vec3e{0,0,0} , 
							RXcolor , 1.0f ,
							lenTravelMin , lenTravelMax , travelSpeed ,
							width , lengthMin , lengthMax )
		);
		
	}
	
	
	// Initialize GLUT and process user parameters
	glutInit( &argc , argv );
	
	// Request double buffered, true color window 
	glutInitDisplayMode( GLUT_RGB | GLUT_DOUBLE | GLUT_DEPTH );
	
	// Request 500 x 500 pixel window
	glutInitWindowSize( 975 , 725 );
	
	// Create the window
	glutCreateWindow( ( "James Watson , " + HWname ).c_str() );
	
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
	glutSpecialFunc( special );
	
	// Tell GLUT to call "key" when a key is pressed
	glutKeyboardFunc( key );
	
	// Check for errors
	ErrCheck( "main" );
	
	//  Pass control to GLUT so it can interact with the user
	glutMainLoop();
	
	//  Return code
	return 0;
}

// ___ END MAIN ____________________________________________________________________________________________________________________________


/* === SPARE PARTS =========================================================================================================================

~~ CLASS NOTES ~~
* At discontinuities, lighting normals must be calculated many times for the same point
* If you start with some unit object and distort it , then you must make sure that the lighting normals distort with it
* Diffuse Reflections
	- If the angle of incidence is greater than 90 deg, then there is no light contribution
* Lighting
	- You need to define the normals according to the simulated smoothness of the surface, and this can be difficult
	- In local lighting, objects are oblivious to each other
	- We must to shadows on their own (end of the class)
	- If a broad flat surface only has very few vertices, then the lighting will look the same across the entire surface.  
	  If you want light to be local on the same surface, you must have more vertices/polygons that are sensitive to light distance
	- Normals must be perpendicular to the underlying surface
		> You can get away with fewer polygons if your lighting normals are properly representing the simulated surface
	- glNormal: Specifies the normal that will apply to the following list of vertices
	- glEnable( GL_LIGHTING )
	- glEnable( GL_LIGHT0 ) , GL_LIGHT0 to GL_LIGHT7 , Only 8 lights available in the normal OGL pipeline
	- glColor does not work with all types of lighting, You must use the material properties
		> Only applies to the ambient and diffuse color
	- Normal vectors will scale with the transformation, You can ask OGL to renormalize them for you , glEnable( GL_NORMALIZE )
# DRAW A BALL THAT REPRESENTS THE LIGHT
# FOR EVERY VERTEX THAT IS DRAWN, YOU MUST SPECIFY A VERTEX

   ___ END PARTS ___ */
