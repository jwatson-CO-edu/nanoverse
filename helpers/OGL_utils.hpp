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

/***** Utility Functions *************************************************************************/

/*** Errors ***/
bool ErrCheck( const char* where ); // --- See if OpenGL has raised any errors
void Fatal( const char* format , ... ); // Scream and Run

/*** Text ***/
void Print( const char* format , ... );

/*** Simple Graphics ***/
void draw_origin( float scale );

void draw_grid_org_XY( float gridSize , uint xPlusMinus , uint yPlusMinus , 
					   float lineThic , vec3e color );

/***** CLASNAME_1 ********************************************************************************/

class CLASNAME_1{
// A_ONE_LINE_DESCRIPTION_OF_THE_CLASS
/***** Public *****/ public:
/*** Functions ***/


/***** Protected *****/ protected:
/*** Vars ***/

};


#endif