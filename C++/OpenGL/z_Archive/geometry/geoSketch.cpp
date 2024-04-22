#ifndef GEOSKETCH
#define GEOSKETCH

/***** Environment *******************************************************************************/

/***** Include *****/
#include <../glm/glm/vec4.hpp> 
/*** Local ***/
#include "../helpers/config.hpp"

/***** Utilities *******************************************************************************/
GLfloat* alloc_OGL_array_for_eigen_matx( const matXe& src ){
     return (GLfloat*) malloc( 
          src.rows() * src.cols() * sizeof( GLfloat ) 
     );
}

void copy_eig_to_ogl_consecutive( const matXe& src, GLfloat* dst ){
     // The vertices are understood to be tightly packed in the array, consecutively in memory
     size_t Nrow = src.rows();
     size_t Mcol = src.cols();
     
     for( size_t i = 0 ; i < Nrow ; ++i ){
          for( size_t j = 0 ; j < Mcol ; ++j ){
               // cout << "Array Offset: " << (j*Nrow + i) << endl;
               *(dst + (i*Mcol + j) ) = (GLfloat) src.coeff( i, j );
          }
     }
}

void set_camera( typeF posX        , typeF posY        , typeF posZ         , 
                 typeF targetX     , typeF targetY     , typeF targetZ      ,
                 typeF upDirX = 0.0, typeF upDirY = 0.0, typeF upDirZ = 1.0 ){
     // AUTHOR: Song Ho Ahn (song.ahn@gmail.com)  // CREATED: 2006-11-14  // UPDATED: 2012-04-11
     gluLookAt( posX   , posY   , posZ    , // - eye(x,y,z)
                targetX, targetY, targetZ , // focal(x,y,z)
                upDirX , upDirY , upDirZ  ); // - up(x,y,z)
}


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

// Delegated Constructors
Cuboid() : Cuboid( 1.0, 1.0, 1.0 ){} // Default Cube
Cuboid( typeF side ) : Cuboid( side, side, side ) {} // Cube with sides of a given length


Cuboid( typeF sideX, typeF sideY, typeF sideZ ){
     // Axis-aligned cuboid with an extent in each axis, ceneted on the origin
     typeF hX = sideX/2.0;
     typeF hY = sideY/2.0;
     typeF hZ = sideZ/2.0;

     V.resize( 12, 9 );
     
     vec3r v0(  hX,  hY,  hZ ); 
     vec3r v1(  hX, -hY,  hZ ); 
     vec3r v2(  hX, -hY, -hZ ); 
     vec3r v3(  hX,  hY, -hZ ); 
     vec3r v4( -hX,  hY, -hZ ); 
     vec3r v5( -hX,  hY,  hZ ); 
     vec3r v6( -hX, -hY,  hZ ); 
     vec3r v7( -hX, -hY, -hZ ); 
    /* cube ///////////////////////////////////////////////////////////////////////

         Z
         |
         v6----- v5
        /|      /|
       v1------v0|
       | |     | |
       | v7----|-v4---Y
       |/      |/
       v2------v3
      /
     X

     vertex coords array for glDrawArrays() =====================================
     A cube has 6 sides and each side has 2 triangles, therefore, a cube consists
     of 36 vertices (6 sides * 2 tris * 3 vertices = 36 vertices). And, each
     vertex is 3 components (x,y,z) of floats, therefore, the size of vertex
     array is 108 floats (36 * 3 = 108). */
     
     V.row( 0) << v0, v1, v2; // (front)
     V.row( 1) << v2, v3, v0;

     V.row( 2) << v0, v3, v4; // (right)
     V.row( 3) << v4, v5, v0;

     V.row( 4) << v0, v5, v6; // (top)
     V.row( 5) << v6, v1, v0;

     V.row( 6) << v1, v6, v7; // (left)
     V.row( 7) << v7, v2, v1;

     V.row( 8) << v7, v4, v3; // (bottom)
     V.row( 9) << v3, v2, v7;

     V.row(10) << v4, v7, v6; // (back)
     V.row(11) << v6, v5, v4;


     N.resize( 12, 9 );
     vec3r front(  1,  0,  0 ); // X
     vec3r back(  -1,  0,  0 );
     vec3r right(  0,  1,  0 ); // Y
     vec3r left(   0, -1,  0 );
     vec3r top(    0,  0,  1 ); // Z
     vec3r bottom( 0,  0, -1 );

     N.row( 0) << front , front , front ; // v0-v1-v2 (front)
     N.row( 1) << front , front , front ; // v2-v3-v0

     N.row( 2) << right , right , right ; // v0-v3-v4 (right)
     N.row( 3) << right , right , right ; // v4-v5-v0

     N.row( 4) << top   , top   , top   ; // v0-v5-v6 (top)
     N.row( 5) << top   , top   , top   ; // v6-v1-v0

     N.row( 6) << left  , left  , left  ; // v1-v6-v7 (left)
     N.row( 7) << left  , left  , left  ; // v7-v2-v1

     N.row( 8) << bottom, bottom, bottom; // v7-v4-v3 (bottom)
     N.row( 9) << bottom, bottom, bottom; // v3-v2-v7

     N.row(10) << back  , back  , back  ; // v4-v7-v6 (back)
     N.row(11) << back  , back  , back  ; // v6-v5-v4

     C.resize( 12, 12 );
     C << 1, 1, 1,1,   1, 1, 0,1,   1, 0, 0,1, // v0-v1-v2 (front)
          1, 0, 0,1,   1, 0, 1,1,   1, 1, 1,1, // v2-v3-v0

          1, 1, 1,1,   1, 0, 1,1,   0, 0, 1,1, // v0-v3-v4 (right)
          0, 0, 1,1,   0, 1, 1,1,   1, 1, 1,1, // v4-v5-v0

          1, 1, 1,1,   0, 1, 1,1,   0, 1, 0,1, // v0-v5-v6 (top)
          0, 1, 0,1,   1, 1, 0,1,   1, 1, 1,1, // v6-v1-v0

          1, 1, 0,1,   0, 1, 0,1,   0, 0.5, 0,1, // v1-v6-v7 (left)
          0, 0.5, 0,1,   1, 0, 0,1,   1, 1, 0,1, // v7-v2-v1

          0, 0.5, 0,1,   0, 0, 1,1,   1, 0, 1,1, // v7-v4-v3 (bottom)
          1, 0, 1,1,   1, 0, 0,1,   0, 0.5, 0,1, // v3-v2-v7

          0, 0, 1,1,   0, 0.5, 0,1,   0, 1, 0,1, // v4-v7-v6 (back)
          0, 1, 0,1,   0, 1, 1,1,   0, 0, 1,1; // v6-v5-v4
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
     vertices = alloc_OGL_array_for_eigen_matx( mVertMatx );
     copy_eig_to_ogl_consecutive( mVertMatx, vertices );
}

void load_normals( const matXe& mNormMatx ){
     // Allocate normals and copy from the matrix
     normals = alloc_OGL_array_for_eigen_matx( mNormMatx );
     copy_eig_to_ogl_consecutive( mNormMatx, normals );
}

void load_colors( const matXe& mColrMatx ){
     // Allocate colors and copy from the matrix
     colors = alloc_OGL_array_for_eigen_matx( mColrMatx );
     copy_eig_to_ogl_consecutive( mColrMatx, colors );
}

Model( const Mesh::Mesh& mesh, GLboolean buffMode_ = 0 ){
     // Load vertices, normals, and colors from the model
     bufferMode = buffMode_;
     load_vertices( mesh.V );
     load_normals( mesh.N );
     load_colors( mesh.C );
}

~Model(){
     // De-allocate all memory
}

void draw_array_mode(){
     // draw a cube using vertex array method
     // AUTHOR: Song Ho Ahn (song.ahn@gmail.com)  // CREATED: 2006-11-14  // UPDATED: 2012-04-11

     // 1. Enable vertex arrays
     glEnableClientState( GL_VERTEX_ARRAY );
     glEnableClientState( GL_NORMAL_ARRAY );
     glEnableClientState( GL_COLOR_ARRAY  );
     
     // 2. Prior to drawing, specify vertex arrays
     glVertexPointer( 3, GL_FLOAT, 0, vertices );
     glNormalPointer(    GL_FLOAT, 0, normals  );
     glColorPointer(  4, GL_FLOAT, 0, colors   );

          // Check for errors
     char* where = nullptr; 
     if( ErrCheck( where ) ){
          cout << "There were errors:" << endl << where << endl << endl;
     }else{
          cout << "There were no errors." << endl;
     }
     
     // 3. Draw the model specified by the arrays
     glDrawArrays( GL_TRIANGLES, 0, 36 );

     // 4. Disable vertex arrays
     glDisableClientState( GL_VERTEX_ARRAY );  
     glDisableClientState( GL_NORMAL_ARRAY );
     glDisableClientState( GL_COLOR_ARRAY  );
        

}

void draw_buffer_mode(){
     
}

void draw(){
     // Render the `Model` using the function appropriate to the mode
     if( bufferMode )
          draw_buffer_mode();
     else
          draw_array_mode();
}

};


//~~~~~~~~ END ~~ Model Namespace ~~ END ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
}

#endif