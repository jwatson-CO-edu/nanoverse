#include "include/math_glm.hpp"

////////// TRIGONOMETRY ////////////////////////////////////////////////////////////////////////////

// Cosine and Sine in degrees
// Author: Willem A. (Vlakkies) Schreüder  
double Cos( double x ){  return cos( (x) * 3.1415927 / 180 );  }
double Sin( double x ){  return sin( (x) * 3.1415927 / 180 );  }
double Tan( double x ){  return tan( (x) * 3.1415927 / 180 );  }
float  Cosf( float x ){  return cosf( (x) * 3.1415927f / 180.0f );  }
float  Sinf( float x ){  return sinf( (x) * 3.1415927f / 180.0f );  }
float  Tanf( float x ){  return tanf( (x) * 3.1415927f / 180.0f );  }
float  Atan2f( float y, float x ){  return (atan2f( y, x ) * 3.1415927f / 180.0f);  }


mat4f R_x( float theta ){
    mat4f rtnMtx = mat4f{ 1.0 };
    rtnMtx = rotate( rtnMtx, theta, vec3f{ 1.0, 0.0, 0.0 } );
    return rtnMtx;
}


mat4f R_y( float theta ){
    mat4f rtnMtx = mat4f{ 1.0 };
    rtnMtx = rotate( rtnMtx, theta, vec3f{ 0.0, 1.0, 0.0 } );
    return rtnMtx;
}


mat4f R_z( float theta ){
    mat4f rtnMtx = mat4f{ 1.0 };
    rtnMtx = rotate( rtnMtx, theta, vec3f{ 0.0, 0.0, 1.0 } );
    return rtnMtx;
}