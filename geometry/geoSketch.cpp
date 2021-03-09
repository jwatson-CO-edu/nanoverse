/***** Environment *******************************************************************************/

/***** Include *****/
#include <../glm/vec4.hpp> 
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

Cuboid::Cuboid( typeF sideX, typeF sideY, typeF sideZ ){
    typeF hX = sideX;
    typeF hY = sideY;
    typeF hZ = sideZ;
    // cube ///////////////////////////////////////////////////////////////////////

//    v6----- v5
//   /|      /|
//  v1------v0|
//  | |     | |
//  | |v7---|-|v4
//  |/      |/
//  v2------v3

// vertex coords array for glDrawArrays() =====================================
// A cube has 6 sides and each side has 2 triangles, therefore, a cube consists
// of 36 vertices (6 sides * 2 tris * 3 vertices = 36 vertices). And, each
// vertex is 3 components (x,y,z) of floats, therefore, the size of vertex
// array is 108 floats (36 * 3 = 108).
V  <<  hX, 1, 1,  -hX, 1, 1,  -hX,-1, 1,      // v0-v1-v2 (front)
      -hX,-1, 1,   hX,-1, 1,   hX, 1, 1,      // v2-v3-v0

       hX, 1, 1,   hX,-1, 1,   hX,-1,-1 ;      // v0-v3-v4 (right)
       hX,-1,-1,   hX, 1,-1,   1, 1, 1 ;      // v4-v5-v0

       hX, 1, 1,   hX, 1,-1,  -1, 1,-1 ;      // v0-v5-v6 (top)
      -hX, 1,-1,  -hX, 1, 1,   1, 1, 1 ;      // v6-v1-v0

      -hX, 1, 1,  -hX, 1,-1,  -1,-1,-1 ;      // v1-v6-v7 (left)
      -hX,-1,-1,  -hX,-1, 1,  -1, 1, 1 ;      // v7-v2-v1

      -hX,-1,-1,   hX,-1,-1,   1,-1, 1 ;      // v7-v4-v3 (bottom)
       hX,-1, 1,  -hX,-1, 1,  -1,-1,-1 ;      // v3-v2-v7

       hX,-1,-1,  -hX,-1,-1,  -1, 1,-1 ;      // v4-v7-v6 (back)
      -hX, 1,-1,   hX, 1,-1,   1,-1,-1 ;    // v6-v5-v4
}

/***** Dart **************************************************************************************/
class Dart : public Mesh { public: EIGEN_MAKE_ALIGNED_OPERATOR_NEW
// 

Dart();
Dart( typeF len ); // Dart with `len`
Dart( typeF lenX, typeF lenY, typeF lenZ );

};



//~~~~~~~~ END ~~ Mesh Namespace ~~ END ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
}