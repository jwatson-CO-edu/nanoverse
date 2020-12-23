#pragma once // This also helps things not to be loaded twice , but not always . See below

/***********  
TEMPLATE.hpp
James Watson , YYYY-MM
A_ONE_LINE_DESCRIPTION_OF_THE_FILE

Template Version: 2020-12
***********/

#ifndef MODEL_H // This pattern is to prevent symbols to be loaded multiple times
#define MODEL_H // from multiple imports

/***** Environment *******************************************************************************/

/***** Include *****/

/*** OpenGL ***/
#include <GL/glut.h> //- OpenGL Utilities
#include <GL/glext.h> // Paint text to screen

/*** Local ***/
#include "mesh.hpp"

/***** Utility Functions *************************************************************************/




/***** Model *************************************************************************************/

/*
http://www.songho.ca/opengl/gl_vbo.html
*/


class Model{
// Virtual class for all other single-mesh models to inherit
/***** Public *****/ public:

~Model(); // Destructor

/***** Protected *****/ protected:
GLuint   vboId; // ID of the VBO
GLfloat* VBO; // - Vertex Buffer Array
GLfloat* N; // --- Normals
};


#endif