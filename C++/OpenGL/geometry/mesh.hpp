#pragma once // This also helps things not to be loaded twice , but not always . See below

/***********  
TEMPLATE.hpp
James Watson , YYYY-MM
A_ONE_LINE_DESCRIPTION_OF_THE_FILE

Template Version: 2020-12
***********/

#ifndef TEMPLATE_H // This pattern is to prevent symbols to be loaded multiple times
#define TEMPLATE_H // from multiple imports

/***** Environment *******************************************************************************/

/***** Include *****/

/*** Local ***/
#include "../helpers/config.hpp"




/***** Utility Functions *************************************************************************/




/***** Mesh **************************************************************************************/

class Mesh{
// A triangle mesh composed of Vertices, Facets, and Normals
/***** Public *****/ public:
EIGEN_MAKE_ALIGNED_OPERATOR_NEW
/*** Functions ***/

/* Construction */
bool load_geo_from_file( string fPath );

/* Getters */
matXe& get_vertices();
matXi& get_facets();
matXe& get_normals();

/*** Vars ***/
matXe V; // Points of the mesh
matXi F; // Facets corresponding to the points V
matXe N; // Mesh normals
};


#endif

/* RESOURCES
* http://strattonbrazil.blogspot.com/2011/09/single-pass-wireframe-rendering_11.html
*/