/***** Environment *******************************************************************************/

/***** Include *****/

/*** Local ***/
#include "../helpers/config.hpp"

/***** Mesh **************************************************************************************/

class Mesh{ public: EIGEN_MAKE_ALIGNED_OPERATOR_NEW
// A triangle mesh composed of Vertices, Facets, and Normals

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

matXe& Mesh::get_vertices(){  return V;  }
matXi& Mesh::get_facets(){    return F;  }
matXe& Mesh::get_normals(){   return N;  }