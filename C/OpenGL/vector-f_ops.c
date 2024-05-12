#include "toolbox.h"

////////// SCALED 3D VECTORS ///////////////////////////////////////////////////////////////////////

vec4f make_vec4f( float x, float y, float z ){
    // Make a 3D float vector with scale = 1.0 from three floats
    vec4f rtnVec = {x,y,z,1.0f};
    return rtnVec;
}


vec4f make_0_vec4f( void ){
    // Make a 3D zero float vector with scale = 1.0
    vec4f rtnVec = {0.0f,0.0f,0.0f,1.0f};
    return rtnVec;
}


vec4f rand_vec4f( void ){
    // Make a random unit 3D float vector with each of {X,Y,Z} on (0.0, 1.0]
    vec4f rtnVec = {randf(), randf(), randf(), 1.0f};
    return unit_vec4f( rtnVec );
}


vec4f sub_vec4f( const vec4f u, const vec4f v ){
    // Calc `u` - `v` = `r`, R^3
    vec4f rtnVec = {
        u.x - v.x,
        u.y - v.y,
        u.z - v.z,
        1.0f
    };
    return rtnVec;
}


vec4f add_vec4f( const vec4f u, const vec4f v ){
    // Calc `u` + `v` = `r`, R^3
    vec4f rtnVec = {
        u.x + v.x,
        u.y + v.y,
        u.z + v.z,
        1.0f
    };
    return rtnVec;
}


float dot_vec4f( const vec4f u, const vec4f v ){
    // Calc `u` * `v` = `r`, R^3
    return u.x * v.x + u.y * v.y + u.z * v.z;
}


float norm_vec4f( const vec4f vec ){  
    // Euclidean length of an R^3
    return sqrtf(vec.x*vec.x + vec.y*vec.y + vec.z*vec.z); // Assume w = 1.0f
} 


float diff_vec4f( const vec4f u, const vec4f v ){  
    // Euclidean length of `u`-`v`
    return norm_vec4f( sub_vec4f( u, v ) );
} 


vec4f unit_vec4f( const vec4f vec ){
    // Calc the unit direction of `vec` and return it, R^3
    vec4f  rtnVec;
    float mag = norm_vec4f( vec );
    if( mag > 0.0f ){
        rtnVec.x = vec.x / mag;
        rtnVec.y = vec.y / mag;
        rtnVec.z = vec.z / mag;
    }else{
        rtnVec.x = 0.0f;
        rtnVec.y = 0.0f;
        rtnVec.z = 0.0f;
    }
    rtnVec.w = 1.0f;
    return rtnVec;
}

vec4f cross_vec4f( const vec4f u, const vec4f v ){
    // Calc `u` X `v` = `p`, R^3
    // Source: http://aleph0.clarku.edu/~djoyce/ma131/dotcross.pdf , pg. 3
    vec4f rtnVec = {
        u.y*v.z - u.z*v.y,
        u.z*v.x - u.x*v.z,
        u.x*v.y - u.y*v.x,
        1.0f
    };
    return rtnVec;
}


vec4f div_vec4f( const vec4f u, float d ){
    // Calc `u` * `f` = `r`, R^3
    vec4f rtnVec = {
        u.x / d,
        u.y / d,
        u.z / d,
        1.0f
    };
    return rtnVec;
}


vec4f scale_vec4f( const vec4f u, float f ){
    // Calc `u` * `f` = `r`, R^3
    vec4f rtnVec = {
        u.x * f,
        u.y * f,
        u.z * f,
        1.0f
    };
    return rtnVec;
}

vec4f blend_vec4f( const vec4f u, float fU, const vec4f v, float fV ){
    // Return the weighted sum of the two verctors, R^3
    vec4f rtnVec = add_vec4f(
        scale_vec4f( u, fU ),
        scale_vec4f( v, fV )
    );
    return rtnVec;
}


vec4f stretch_to_len_vec4f( const vec4f vec, float len ){
    // Stretch `vec` to `len` and return
    return scale_vec4f( unit_vec4f( vec ), len );
}

///// 3D Segments & Triangles /////////////////////////////////////////////

vec4f seg_center( const vec4f v0, const vec4f v1 ){
    // Calc centroid of 2 R^3 points
    vec4f rtnVec = {
        (v0.x + v1.x) / 2.0f,
        (v0.y + v1.y) / 2.0f,
        (v0.z + v1.z) / 2.0f,
        1.0f
    };
    return rtnVec;
}


vec4f tri_center( const vec4f v0, const vec4f v1, const vec4f v2 ){
    // Calc centroid of 3 R^3 points
    vec4f rtnVec = {
        (v0.x + v1.x + v2.x) / 3.0f,
        (v0.y + v1.y + v2.y) / 3.0f,
        (v0.z + v1.z + v2.z) / 3.0f,
        1.0f
    };
    return rtnVec;
}


vec4f get_CCW_tri_norm( const vec4f v0, const vec4f v1, const vec4f v2 ){
    // Find the normal vector `n` of a triangle defined by CCW vertices in R^3: {`v0`,`v1`,`v2`}
    return unit_vec4f( cross_vec4f(
        sub_vec4f( v1, v0 ),
        sub_vec4f( v2, v0 )
    ) );
}



////////// 2D VECTORS //////////////////////////////////////////////////////////////////////////////

vec2f make_vec2f( float x, float y ){
    // Create a 2D float vector from 2 floats
    vec2f rtnVec = {x,y};
    return rtnVec;
}


vec2f add_vec2f( const vec2f u, const vec2f v ){
    // Calc `u` + `v` = `r`, R^2
    vec2f rtnVec = {
        u.x + v.x,
        u.y + v.y
    };
    return rtnVec;
}



////////// UINT VECTORS ////////////////////////////////////////////////////////////////////////////

vec3u make_vec3u( uint u0, uint u1, uint u2 ){
    // Create a 3D uint vector
    vec3u rtnVec = {u0,u1,u2};
    return rtnVec;
}



////////// 2D <---> 3D /////////////////////////////////////////////////////////////////////////////

vec4f lift_pnt_2D_to_3D( const vec2f pnt2f, const vec4f origin, const vec4f xBasis, const vec4f yBasis ){
    // Project the local 2D point to the global 3D frame
    return add_vec4f(
        origin,
        add_vec4f(
            scale_vec4f( xBasis, pnt2f.x ), 
            scale_vec4f( yBasis, pnt2f.y )
        )
    );
}


vec4f lift_vec_2D_to_3D( const vec2f vct2f, const vec4f xBasis, const vec4f yBasis ){
    // Project the local 2D vector to the global 3D frame
    return add_vec4f( 
        scale_vec4f( xBasis, vct2f.x ), 
        scale_vec4f( yBasis, vct2f.y )
    );
}


////////// VECTOR PRINTING /////////////////////////////////////////////////////////////////////////

void print_vec4f( const vec4f vec ){  printf( "[%f, %f, %f]", vec.x, vec.y, vec.z );  }
void print_vec2f( const vec2f vec ){  printf( "[%f, %f]", vec.x, vec.y );  }
void print_vec3u( const vec3u vec ){  printf( "[%u, %u, %u]", vec.v0, vec.v1, vec.v2 );  }


////////// OPENGL HELPERS //////////////////////////////////////////////////////////////////////////

void glVtx4f( const vec4f v ){  glVertex4f( v.x , v.y , v.z, v.w );  } // Set vertex with a vector
void glNrm4f( const vec4f n ){  glNormal3f( n.x , n.y , n.z      );  } // Set normal with a vector
void glClr4f( const vec4f c ){  glColor4f(  c.r , c.g , c.b, c.a );  } // Set color with a vector