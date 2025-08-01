#ifndef MATH_GLM_HPP // This pattern is to prevent symbols to be loaded multiple times
#define MATH_GLM_HPP // from multiple imports

#include<vector>
using std::vector;
#include<list>
using std::list;

/// Graphics Language Math ////
#include <glm/glm.hpp>
using glm::vec2, glm::vec3, glm::mat4, glm::normalize, glm::cross, glm::length;
#include <glm/vec4.hpp> 
using glm::vec4;
#include <glm/gtc/matrix_transform.hpp>
using glm::translate, glm::scale, glm::rotate;
#include <glm/gtc/type_ptr.hpp>
using glm::value_ptr;

////////// TYPE DEFINES ////////////////////////////////////////////////////////////////////////////
typedef vec2 /*----*/ vec2f;
typedef vec3 /*----*/ vec3f;
typedef vec4 /*----*/ vec4f;
typedef mat4 /*----*/ mat4f;


////////// CONTAINERS //////////////////////////////////////////////////////////////////////////////
template<typename T>
vector<T> list_2_vector( const list<T>& lst ){
    vector<T> rtnVec;
    size_t    i = 0;
    rtnVec.reserve( lst.size() );
    for( const T& elem : lst ){
        rtnVec[i] = elem;
        ++i;
    }
    return rtnVec;
}


////////// TRIGONOMETRY ////////////////////////////////////////////////////////////////////////////

// Cosine and Sine in degrees
// Author: Willem A. (Vlakkies) Schreüder  
double Cos( double x );
double Sin( double x );
double Tan( double x );
float  Cosf( float x );
float  Sinf( float x );
float  Tanf( float x );
float  Atan2f( float y, float x );


////////// HOMOGENEOUS COORDINATES /////////////////////////////////////////////////////////////////
mat4f R_x( float theta ); // Transform for rotation about the X-axis
mat4f R_y( float theta ); // Transform for rotation about the Y-axis
mat4f R_z( float theta ); // Transform for rotation about the Z-axis

#endif