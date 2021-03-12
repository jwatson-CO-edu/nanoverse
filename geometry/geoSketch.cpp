#ifndef GEOSKETCH
#define GEOSKETCH

/***** Environment *******************************************************************************/

/***** Include *****/
#include <../glm/glm/vec4.hpp> 
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

/*** Vars ***/
matXe V; // Points of the mesh
matXe N; // Mesh normals
matXe C; // Vertex Colors
};






/***** Cube **************************************************************************************/
class Cuboid : public Mesh { public: EIGEN_MAKE_ALIGNED_OPERATOR_NEW
// Axis-aligned Cuboid

Cuboid(); // Default Cube
Cuboid( typeF side ); // Cube with `side` length


Cuboid( typeF sideX, typeF sideY, typeF sideZ ){
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
V.resize( 12, 9 );
V  <<  hX, hY, hZ,  -hX, hY, hZ,  -hX,-hY, hZ, // v0-v1-v2 (front)
      -hX,-hY, hZ,   hX,-hY, hZ,   hX, hY, hZ, // v2-v3-v0

       hX, hY, hZ,   hX,-hY, hZ,   hX,-hY,-hZ, // v0-v3-v4 (right)
       hX,-hY,-hZ,   hX, hY,-hZ,   hX, hY, hZ, // v4-v5-v0

       hX, hY, hZ,   hX, hY,-hZ,  -hX, hY,-hZ, // v0-v5-v6 (top)
      -hX, hY,-hZ,  -hX, hY, hZ,   hX, hY, hZ, // v6-v1-v0

      -hX, hY, hZ,  -hX, hY,-hZ,  -hX,-hY,-hZ, // v1-v6-v7 (left)
      -hX,-hY,-hZ,  -hX,-hY, hZ,  -hX, hY, hZ, // v7-v2-v1

      -hX,-hY,-hZ,   hX,-hY,-hZ,   hX,-hY, hZ, // v7-v4-v3 (bottom)
       hX,-hY, 1,   -hX,-hY, hZ,  -hX,-hY,-hZ, // v3-v2-v7

       hX,-hY,-hZ,  -hX,-hY,-hZ,  -hX, hY,-hZ, // v4-v7-v6 (back)
      -hX, hY,-hZ,   hX, hY,-hZ,   hX,-hY,-hZ; // v6-v5-v4

N.resize( 12, 9 );
N << 0, 0, 1,   0, 0, 1,   0, 0, 1, // v0-v1-v2 (front)
     0, 0, 1,   0, 0, 1,   0, 0, 1, // v2-v3-v0

     1, 0, 0,   1, 0, 0,   1, 0, 0, // v0-v3-v4 (right)
     1, 0, 0,   1, 0, 0,   1, 0, 0, // v4-v5-v0

     0, 1, 0,   0, 1, 0,   0, 1, 0, // v0-v5-v6 (top)
     0, 1, 0,   0, 1, 0,   0, 1, 0, // v6-v1-v0

    -1, 0, 0,  -1, 0, 0,  -1, 0, 0, // v1-v6-v7 (left)
    -1, 0, 0,  -1, 0, 0,  -1, 0, 0, // v7-v2-v1

     0,-1, 0,   0,-1, 0,   0,-1, 0, // v7-v4-v3 (bottom)
     0,-1, 0,   0,-1, 0,   0,-1, 0, // v3-v2-v7

     0, 0,-1,   0, 0,-1,   0, 0,-1, // v4-v7-v6 (back)
     0, 0,-1,   0, 0,-1,   0, 0,-1; // v6-v5-v4

C.resize( 12, 9 );
C << 1, 1, 1,   1, 1, 0,   1, 0, 0, // v0-v1-v2 (front)
     1, 0, 0,   1, 0, 1,   1, 1, 1, // v2-v3-v0

     1, 1, 1,   1, 0, 1,   0, 0, 1, // v0-v3-v4 (right)
     0, 0, 1,   0, 1, 1,   1, 1, 1, // v4-v5-v0

     1, 1, 1,   0, 1, 1,   0, 1, 0, // v0-v5-v6 (top)
     0, 1, 0,   1, 1, 0,   1, 1, 1, // v6-v1-v0

     1, 1, 0,   0, 1, 0,   0, 0, 0, // v1-v6-v7 (left)
     0, 0, 0,   1, 0, 0,   1, 1, 0, // v7-v2-v1

     0, 0, 0,   0, 0, 1,   1, 0, 1, // v7-v4-v3 (bottom)
     1, 0, 1,   1, 0, 0,   0, 0, 0, // v3-v2-v7

     0, 0, 1,   0, 0, 0,   0, 1, 0, // v4-v7-v6 (back)
     0, 1, 0,   0, 1, 1,   0, 0, 1; // v6-v5-v4
}

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




/*************************************************************************************************
 *          Model Namespace                                                                      *
 *************************************************************************************************/ 
namespace Model{


/***** Model **************************************************************************************/

class Model{ public: EIGEN_MAKE_ALIGNED_OPERATOR_NEW

GLfloat*  vertices; // - Vertex array
GLfloat*  normals; // -- Normal array
GLfloat*  colors; // --- Color array
GLboolean bufferMode; // Using buffer objects to render? (Default: 0)

Model(){
     bufferMode = 0;
}

void load_vertices( const matXe& mVertMatx ){
     // Allocate vertices and copy from the matrix
}

void load_normals( const matXe& mNormMatx ){
     // Allocate normals and copy from the matrix
}

void load_colors( const matXe& mColrMatx ){
     // Allocate colors and copy from the matrix
}

Model( const Mesh::Mesh& mesh, GLboolean buffMode_ = 0 ){
     bufferMode = buffMode_;
}

~Model(){
     // De-allocate all memory
}

void draw_array_mode(){

}

void draw_buffer_mode(){
     
}

};


//~~~~~~~~ END ~~ Model Namespace ~~ END ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
}

#endif