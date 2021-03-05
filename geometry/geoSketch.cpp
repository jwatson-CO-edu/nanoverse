/***** Environment *******************************************************************************/

/***** Include *****/

/*** Local ***/
#include "../helpers/config.hpp"

/*************************************************************************************************
 *          Mesh Namespace                                                                       *
 *************************************************************************************************/ 
namespace Mesh{


/***** Mesh **************************************************************************************/

class Mesh{ public: EIGEN_MAKE_ALIGNED_OPERATOR_NEW
// A triangle mesh composed of Vertices, Facets, and Normals

/*** Functions ***/

/* Construction */
// bool add_tri( vec3e v1, vec3e v2, vec3e v3 ); // MAYBE LATER
// bool load_geo_from_file( string fPath ); // MAYBE LATER

/* Getters */
matXe& get_vertices();
matXi& get_facets();
matXe& get_normals();

/*** Vars ***/
matXe V; // Points of the mesh
matXi F; // Facets corresponding to the points V
matXe N; // Mesh normals
};

matXe& Mesh::get_vertices(){  return V;  } // Return a reference to vertices
matXi& Mesh::get_facets(){    return F;  } // Return a reference to facets
matXe& Mesh::get_normals(){   return N;  } // Return a reference to normals




/***** Cube **************************************************************************************/
class Cuboid : public Mesh { public: EIGEN_MAKE_ALIGNED_OPERATOR_NEW
// Axis-aligned Cuboid

Cuboid(); // Default Cube
Cuboid( typeF side ); // Cube with `side` length
Cuboid( typeF sideX, typeF sideY, typeF sideZ );

};

/***** Dart **************************************************************************************/
class Dart : public Mesh { public: EIGEN_MAKE_ALIGNED_OPERATOR_NEW
// 

Dart();
Dart( typeF len ); // Dart with `len`
Dart( typeF lenX, typeF lenY, typeF lenZ );

};



//~~~~~~~~ END ~~ Mesh Namespace ~~ END ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
}