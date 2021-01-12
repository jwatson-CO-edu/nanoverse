#pragma once // This also helps things not to be loaded twice , but not always . See below

/***********  
OGL_utils.hpp
James Watson , 2021-01
Common OGL templates and functions

Template Version: 2020-12
***********/

#ifndef OGL_H // This pattern is to prevent symbols to be loaded multiple times
#define OGL_H // from multiple imports

/***** Environment *******************************************************************************/

/***** Include *****/

/*** Standard ***/
#include <stdio.h> //- Streams to communicate with devices such as keyboards, printers, terminals
#include <stdarg.h> // Macros to access individual args of a list of unnamed arguments
#include <string>

/*** OpenGL ***/
#include <GL/glut.h> // OpenGL Utilities

/*** Local ***/
#include "config.hpp"

/***** Constants *****/
const GLclampf MATL_WHITE[] = { 1 , 1 , 1 , 1 };
const GLclampf MATL_BLACK[] = { 0 , 0 , 0 , 1 };
const GLenum   LIGHTS[]     = {
	GL_LIGHT0 , GL_LIGHT1 , GL_LIGHT2 , GL_LIGHT3 , GL_LIGHT4 , GL_LIGHT5 , GL_LIGHT6 , GL_LIGHT7 
};

/***** Namespace *****/
using std::string;

/***** Forward Declarations *****/
struct OGL_ContextConfig;
struct LightSourceConfig;

/***** Utility Functions *************************************************************************/

/*** Window & Context ***/
int init_GLUT( // Start FreeGLUT with the specified parameters
	int argc, char **argv , // -------- Terminal args
	OGL_ContextConfig& params , // ---- Window param structure
	void (*displayCB)() // ------------ Display callback
);

void init_OGL( OGL_ContextConfig& params ); // Initialize OpenGL according to the performance preference

void create_light_source( LightSourceConfig& liteSpec ); // Create a lightsource with the given specification
void default_light_source(); // --------------------------- Create a default light source

int set_redraw_functions(
	void (*reshapeCB)( int , int ) = nullptr , // - Window reshape callback
	void (*timerCB)( int ) = nullptr , // Periodic redraw function (Only set ONE)
	int  refreshPeriod_ms  = 0 , // ----- Period to run `timerCB`
	void (*idleCB)() /*-*/ = nullptr // - Idle redraw function (Only set ONE)
);

int set_GLUT_interaction_handlers( // TODO: Use SDL2 instead?
	void (*keyboardCB)( u_char , int , int ) , // kb handler
	void (*mouseCB)( int , int , int , int ) , // mouse button handler
	void (*mouseMotionCB)( int , int ) // ------- mouse motion handler
);

/*** Errors ***/
bool ErrCheck( const char* where ); // --- See if OpenGL has raised any errors
void Fatal( const char* format , ... ); // Scream and Run

/*** Text ***/
void Print( const char* format , ... );

/*** Simple Graphics ***/
void draw_origin( float scale );

void draw_grid_org_XY( float gridSize , uint xPlusMinus , uint yPlusMinus , 
					   float lineThic , vec3e color );

/*** Defaults ***/
void dflt_displayCB();
void reshapeCB( int a, int b );


/***** OGL_ContextConfg **************************************************************************/

struct OGL_ContextConfig{
// Container for all the settings required/desired for an OpenGL context

/*** Vars ***/
u_int32_t displayMode;
u_int32_t screenDims[2];
u_int32_t screenPosInit[2];
int /*-*/ winHandle;
bool /**/ fastMode;
GLenum    shadeModel;
GLenum    qualityHints;
GLclampf  BGcolor[4];
GLint     stencilBuffer;
GLclampd  clearDepth;

/*** Functions ***/
OGL_ContextConfig(){
	// Default constructor gives the most-common / easiest settings for a context
	
	/* Window Creation Params */ 
	displayMode /**/ = GLUT_RGBA | GLUT_DOUBLE | GLUT_DEPTH | GLUT_STENCIL;
	screenDims[0]    = 600;
	screenDims[1]    = 600;
	screenPosInit[0] = 100;
	screenPosInit[1] = 100;

	/* OGL Params */
	shadeModel   = ( fastMode ? GL_FLAT    : GL_SMOOTH );
	qualityHints = ( fastMode ? GL_FASTEST : GL_NICEST );
}

};


/***** LightSourceConfig *************************************************************************/

struct LightSourceConfig{
// Container for the configuration vars for 1 light source

/*** Vars ***/
GLenum  name; // ------- GL_LIGHT0/1/2/...
GLfloat lightKa[4]; // - Ambient  light
GLfloat lightKd[4]; // - Diffuse  light
GLfloat lightKs[4]; // - Specular light
bool    isPositional; // Is this light at a certain position?
GLfloat position[4]; //- Position to render light from

/*** Functions ***/


};


#endif