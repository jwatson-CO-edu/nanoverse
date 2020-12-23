#pragma once // This also helps things not to be loaded twice , but not always . See below

/***********  
TEMPLATE.hpp
James Watson , YYYY-MM
A_ONE_LINE_DESCRIPTION_OF_THE_FILE

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
const float MATL_WHITE[] = { 1 , 1 , 1 , 1 };
const float MATL_BLACK[] = { 0 , 0 , 0 , 1 };

/***** Namespace *****/
using std::string;

/***** Forward Declarations *****/
struct OGL_ContextConfg;

/***** Utility Functions *************************************************************************/

/*** Window & Context ***/
int init_GLUT( 
	OGL_ContextConfg& params , // ----- Window param structure
	void (*displayCB)() , // ---------- Display callback
	void (*reshapeCB)( int , int ) // - Window reshape callback
);

void init_OGL();

int set_redraw_function(
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

/***** OGL_ContextConfg **************************************************************************/

struct OGL_ContextConfg{
// Container for all the settings required/desired for an OpenGL context

/*** Vars ***/
u_int32_t displayMode;
u_int32_t screenDims[2];
u_int32_t screenPosInit[2];
int /*-*/ winHandle;

/*** Functions ***/
OGL_ContextConfg(){
	// Default constructor gives the most-common / easiest settings for a context
	displayMode /**/ = GLUT_RGBA | GLUT_DOUBLE | GLUT_DEPTH | GLUT_STENCIL;
	screenDims[0]    = 600;
	screenDims[1]    = 600;
	screenPosInit[0] = 100;
	screenPosInit[1] = 100;
}

};


#endif