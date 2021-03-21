#ifndef CONFIG_H // This pattern is to prevent symbols to be loaded multiple times
#define CONFIG_H // from multiple imports

/***** Defines ***********************************************************************************/

#define MG_FLOAT // ---------- Use floats for all Eigen operations
#define LEN 8192 // ---------- Maximum length of text string
#define _USE_MATH_DEFINES // - M_PI , etc.
#define GL_GLEXT_PROTOTYPES // Important for all of your programs

//~ #define MG_DUBBL // Use doubles for all Eigen operations

/***** Includes *****/

/*** Standard ***/
#include <string>
#include <iostream>

/*** Eigen ***/
#include <Eigen/Core> // ---- The living heart of Eigen
#include <Eigen/Dense> // --- Cross Product , etc.
#include <Eigen/Geometry> //- Quaternion , etc

/*** OGL ***/
#include <GL/gl.h>
#include <GL/glut.h>


/***** Shortcuts and Aliases *****/

/*** Standard ***/
using std::string;
using std::cout;
using std::cerr;
using std::endl;

/*** Eigen ***/
#ifdef MG_FLOAT
	using vec2e = Eigen::Vector2f;
	using vec2r = Eigen::RowVector2f;
	using vec3e = Eigen::Vector3f;
	using vec3r = Eigen::RowVector3f;
	using matXe = Eigen::MatrixXf;
	using typeF = float;
	#define random rand_float
#endif
#ifdef MG_DUBBL
	using vec2e = Eigen::Vector2d;
	using vec3e = Eigen::Vector3d;
	using matXe = Eigen::MatrixXd;
	using typeF = double;
	#define random rand_dbbl
#endif
using vec2i = Eigen::Vector2i;
using matXi = Eigen::MatrixXi;

#endif