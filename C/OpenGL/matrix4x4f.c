//
//  4x4 matrix functions
//  Matrices are interpreted as column major order using OpenGL convention
//  Author: Willem A. (Vlakkies) Schreüder, https://www.prinmath.com/
/*
 | [m0  [m4  [ m8  [m12
 |
 V  m1   m5    m9   m13

    m2   m6   m10   m14

    m3]  m7]  m11]  m15]
*/

#include "matrix4x4f.h"
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <math.h>

//  OpenGL or Vulkan
static int vulkan=0;

//  Identity matrix
static const float I[] = {1,0,0,0 , 0,1,0,0 , 0,0,1,0 , 0,0,0,1};


//
//  Set mat4 mode to OpenGL or Vulkan
//
void mat4vulkan( int k ){  vulkan = k;  }


//
//  Allocate and return Identity 4x4 matrix
//
float* make_identity_mtx44f( void ){
    float* rtnMatx = (float*) malloc( sizeof( I ) );
    identity_mtx44f( rtnMatx );
    return rtnMatx;
}


//
//  Identity 4x4 matrix
//
void identity_mtx44f( float mat[] ){  memcpy( mat, I, sizeof(I) );  }


//
//  Copy 4x4 matrix
//
void copy_mtx44f( float mat[], float m[] ){
    memcpy( mat, m, sizeof(I) );
}


//
//  Right multiply 4x4 matrix
//
void mult_mtx44f( float mat[], float m[] ){
    //  res = mat*m
    float res[16];
    for( int i = 0; i < 4; i++ )
        for( int j = 0; j < 4; j++ )
            res[ 4*i+j ] = mat[j]*m[4*i] + mat[4+j]*m[4*i+1] + mat[8+j]*m[4*i+2] + mat[12+j]*m[4*i+3];
    //  Copy matrix back
    memcpy( mat, res, sizeof( res ) );
}


void Rx_mtx44f( float mat[], /*<<*/ float theta_rad ){
    // Load `mat` from R_x( `theta_rad` ) 
    mat[ 0] = 1.0f;  mat[ 4] = 0.0f; /*--------*/  mat[ 8] = 0.0f; /*---------*/  mat[12] = 0.0f;
    mat[ 1] = 0.0f;  mat[ 5] = cosf( theta_rad );  mat[ 9] = -sinf( theta_rad );  mat[13] = 0.0f;
    mat[ 2] = 0.0f;  mat[ 6] = sinf( theta_rad );  mat[10] =  cosf( theta_rad );  mat[14] = 0.0f;
    mat[ 3] = 0.0f;  mat[ 7] = 0.0f; /*--------*/  mat[11] = 0.0f; /*---------*/  mat[15] = 1.0f;
}


void Ry_mtx44f( float mat[], /*<<*/ float theta_rad ){
    // Load `mat` from R_y( `theta_rad` ) 
    mat[ 0] =  cosf( theta_rad );  mat[ 4] = 0.0f;  mat[ 8] = sinf( theta_rad );  mat[12] = 0.0f;
    mat[ 1] = 0.0f; /*---------*/  mat[ 5] = 1.0f;  mat[ 9] = 0.0f; /*--------*/  mat[13] = 0.0f;
    mat[ 2] = -sinf( theta_rad );  mat[ 6] = 0.0f;  mat[10] = cosf( theta_rad );  mat[14] = 0.0f;
    mat[ 3] = 0.0f; /*---------*/  mat[ 7] = 0.0f;  mat[11] = 0.0f; /*--------*/  mat[15] = 1.0f;
}


void Rz_mtx44f( float mat[], /*<<*/ float theta_rad ){
    // Load `mat` from R_z( `theta_rad` ) 
    mat[ 0] = cosf( theta_rad );  mat[ 4] = -sinf( theta_rad ); mat[ 8] = 0.0f;  mat[12] = 0.0f;
    mat[ 1] = sinf( theta_rad );  mat[ 5] =  cosf( theta_rad ); mat[ 9] = 0.0f;  mat[13] = 0.0f;
    mat[ 2] = 0.0f; /*--------*/  mat[ 6] = 0.0f; /*--------*/  mat[10] = 1.0f;  mat[14] = 0.0f;
    mat[ 3] = 0.0f; /*--------*/  mat[ 7] = 0.0f;  /*-------*/  mat[11] = 0.0f;  mat[15] = 1.0f;
}


void rotate_x_mtx44f( float mat[], /*<<*/ float theta_rad ){
   // Rotate `mat` with R_x( `theta_rad` ) 
   float op2[16];
   Rx_mtx44f( op2, theta_rad );
   mult_mtx44f( mat, op2 );
}


void rotate_y_mtx44f( float mat[], /*<<*/ float theta_rad ){
   // Rotate `mat` with R_y( `theta_rad` ) 
   float op2[16];
   Ry_mtx44f( op2, theta_rad );
   mult_mtx44f( mat, op2 );
}


void rotate_z_mtx44f( float mat[], /*<<*/ float theta_rad ){
   // Rotate `mat` with R_z( `theta_rad` ) 
   float op2[16];
   Rz_mtx44f( op2, theta_rad );
   mult_mtx44f( mat, op2 );
}


void R_RPY_vehicle_mtx44f( float mat[], /*<<*/ float r_, float p_, float y_ ){
    // Get a matrix to increment the world Roll, Pitch, Yaw of the model
    // NOTE: This is for airplanes that move forward in their own Z and have a wingspan across X
    float op2[16];
    // Mult 1: M = Ry( Y ) * Rx( P )
    Ry_mtx44f( mat, y_ );
    Rx_mtx44f( op2, p_ );
    mult_mtx44f( mat, op2 );
    // Mult 1: M = M * Rz( R )
    Rz_mtx44f( op2, r_ );
    mult_mtx44f( mat, op2 );
}


//
//  Rotate Angle-Axis
//
void R_angle_axis_mtx44f( float mat[], float th_deg, float x, float y, float z ){
    //  Normalize axis
    float l = sqrtf( x*x + y*y + z*z );
    if(l==0) return;
    x /= l;
    y /= l;
    z /= l;
    //  Calculate sin and cos
    float s = sin(th_deg*M_PI/180);
    float c = cos(th_deg*M_PI/180);
   //  printf( "%f, %f, %f\n", s, c, l );
    //  Rotation matrix
    float R[16] =
    {
        (1.0f-c)*x*x+c   , (1.0f-c)*x*y+z*s , (1.0f-c)*z*x-y*s , 0.0f ,
        (1.0f-c)*x*y-z*s , (1.0f-c)*y*y+c   , (1.0f-c)*y*z+x*s , 0.0f ,
        (1.0f-c)*z*x+y*s , (1.0f-c)*y*z-x*s , (1.0f-c)*z*z+c   , 0.0f ,
        0.0f             , 0.0f             , 0.0f             , 1.0f ,
    };
    //  Multiply
    mult_mtx44f(mat,R);
}

//
//  Translate
//
void translate_mtx44f( float mat[], float dx, float dy, float dz ){
   // 1. Second operand
   float T[16];
   identity_mtx44f( T );
   T[12] = dx;
   T[13] = dy;
   T[14] = dz;
   // 2. Multiply
   mult_mtx44f( mat, T );
}

//
//  Position
//
void set_position_mtx44f( float mat[], float x, float y, float z ){
   // 1. Second operand
   mat[12] = x;
   mat[13] = y;
   mat[14] = z;
}

//
//  Scale
//
void scale_mtx44f( float mat[], float Sx, float Sy, float Sz ){
   // 1. Scale matrix
   float S[16];
   memset( S, 0, sizeof(S) );
   S[ 0] = Sx;
   S[ 5] = Sy;
   S[10] = Sz;
   S[15] = 1;
   // 2. Multiply
   mult_mtx44f( mat, S );
}

//
//  Normalize vector
//
static int normalize(float* x,float* y,float* z)
{
   float l = sqrt((*x)*(*x)+(*y)*(*y)+(*z)*(*z));
   if (l==0) return -1;
   *x /= l;
   *y /= l;
   *z /= l;
   return 0;
}

//
//  Set eye position
//
void mat4lookAt(float mat[16] , float Ex,float Ey,float Ez , float Cx,float Cy,float Cz , float Ux,float Uy,float Uz)
{
   //  Forward = C-E
   float Fx = Cx-Ex;
   float Fy = Cy-Ey;
   float Fz = Cz-Ez;
   if (normalize(&Fx,&Fy,&Fz)) return;
   // Side = Forward x Up
   float Sx = Fy*Uz-Uy*Fz;
   float Sy = Fz*Ux-Uz*Fx;
   float Sz = Fx*Uy-Ux*Fy;
   if (normalize(&Sx,&Sy,&Sz)) return;
   //  Recalculate Up = Side x Forward
   Ux = Sy*Fz-Fy*Sz;
   Uy = Sz*Fx-Fz*Sx;
   Uz = Sx*Fy-Fx*Sy;
   //  Rotation (inverse read transposed)
   float R[16] =
   {
    Sx, Ux, -Fx, 0,
    Sy, Uy, -Fy, 0,
    Sz, Uz, -Fz, 0,
    0,  0,    0, 1,
   };
   mult_mtx44f(mat,R);
   //  Set eye at the origin
   translate_mtx44f(mat,-Ex,-Ey,-Ez);
}

//
// Compute inverse of a general 3d transformation matrix.
//    Adapted from graphics gems II.
// 
void mat3normalMatrix(float mat[16],float inv[9])
{
   // Calculate the determinant of upper left 3x3 submatrix
   float det = mat[0]*mat[5]*mat[10]
              +mat[1]*mat[6]*mat[8]
              +mat[2]*mat[4]*mat[9]
              -mat[2]*mat[5]*mat[8]
              -mat[1]*mat[4]*mat[10]
              -mat[0]*mat[6]*mat[9];
   if (det*det<1e-25) return;
   //  Compute inverse using Cramer's rule
   inv[0] =  (mat[5]*mat[10]-mat[6]*mat[9])/det;
   inv[1] = -(mat[4]*mat[10]-mat[6]*mat[8])/det;
   inv[2] =  (mat[4]*mat[ 9]-mat[5]*mat[8])/det;
   inv[3] = -(mat[1]*mat[10]-mat[2]*mat[9])/det;
   inv[4] =  (mat[0]*mat[10]-mat[2]*mat[8])/det;
   inv[5] = -(mat[0]*mat[ 9]-mat[1]*mat[8])/det;
   inv[6] =  (mat[1]*mat[ 6]-mat[2]*mat[5])/det;
   inv[7] = -(mat[0]*mat[ 6]-mat[2]*mat[4])/det;
   inv[8] =  (mat[0]*mat[ 5]-mat[1]*mat[4])/det;
}

//
//  4x4 version of normal matrix to fix alignment
void mat4normalMatrix(float mat[16],float inv[16])
{
   //  Compute 3x3 normal matrix
   mat3normalMatrix(mat,inv);
   //  Expand 3x3 to 4x4
   inv[15] = 1;
   inv[14] = 0;
   inv[13] = 0;
   inv[12] = 0;
   inv[11] = 0;
   inv[10] = inv[8];
   inv[9] = inv[7];
   inv[8] = inv[6];
   inv[7] = 0;
   inv[6] = inv[5];
   inv[5] = inv[4];
   inv[4] = inv[3];
   inv[3] = 0;
   inv[2] = inv[2];
   inv[1] = inv[1];
   inv[0] = inv[0];
}

//
//  Orthogonal projection matrix
//
void mat4ortho(float mat[],float left,float right,float bottom,float top,float zNear,float zFar)
{
   //  Projection matrix
   float P[16];
   memset(P,0,sizeof(P));
   P[0] = 2/(right-left);
   P[12] = -(right+left)/(right-left);
   P[13] = -(top+bottom)/(top-bottom);
   P[15] = 1;
   if (vulkan)
   {
      P[5]  = -2/(top-bottom);
      P[10] = 1/(zFar-zNear);
      P[14] = 0.5-(zFar+zNear)/(zFar-zNear);
   }
   else
   {
      P[5]  = 2/(top-bottom);
      P[10] = -2/(zFar-zNear);
      P[14] = -(zFar+zNear)/(zFar-zNear);
   }
   //  Multiply
   mult_mtx44f(mat,P);
}

//
//  Perspective projection matrix
//
void mat4perspective(float mat[],float fovy,float asp,float zNear,float zFar)
{
   //  Cotangent
   float s = sin(fovy/2*M_PI/180);
   float c = cos(fovy/2*M_PI/180);
   if (s==0) return;
   float cot = c/s;
   //  Matrix
   float P[16];
   memset(P,0,sizeof(P));
   P[0]  = cot/asp;
   P[11] = -1;
   if (vulkan)
   {
      P[5]  = -cot;
      P[10] = zNear/(zFar-zNear);
      P[14] = zFar*P[10];
   }
   else
   {
      P[5]  = cot;
      P[10] = -(zFar+zNear)/(zFar-zNear);
      P[14] = -2*zNear*zFar/(zFar-zNear);
   }
   //  Multiply
   mult_mtx44f(mat,P);
}

//
//  Print 4x4 matrix to stderr
//
void print_mtx44f(const char* text,float m[16])
{
   fprintf(stderr,"%s %s\n",text,vulkan?"Vulkan":"OpenGL");
   for (int i=0;i<4;i++)
      fprintf(stderr,"%10.6f %10.6f %10.6f %10.6f\n",m[i],m[i+4],m[i+8],m[i+12]);
}

//
//  Print 3x3 matrix to stderr
//
void mat3print(const char* text,float m[9])
{
   fprintf(stderr,"%s\n",text);
   for (int i=0;i<3;i++)
      fprintf(stderr,"%10.6f %10.6f %10.6f\n",m[i],m[i+3],m[i+6]);
}


