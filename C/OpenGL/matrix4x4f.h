//
//  4x4 matrix functions
//  Matrices are interpreted as column major order using OpenGL convention
//  Author: Willem A. (Vlakkies) Schreüder, https://www.prinmath.com/
//
void   mat4vulkan(int k);
void   mat4identity(float mat[]);
float* make_identity( void );
void   mat4copy(float mat[],float m[]);
void   mat4multMatrix(float mat[],float m[]);
void   mat4rotate(float mat[],float th,float x,float y,float z);
void   mat4translate(float mat[],float dx,float dy,float dz);
void   mat4scale(float mat[],float Sx,float Sy,float Sz);
void   mat4lookAt(float mat[16] , float Ex,float Ey,float Ez , float Cx,float Cy,float Cz , float Ux,float Uy,float Uz);
void   mat3normalMatrix(float mat[16],float inv[9]);
void   mat4normalMatrix(float mat[16],float inv[16]);
void   mat4ortho(float mat[],float left,float right,float bottom,float top,float zNear,float zFar);
void   mat4perspective(float mat[],float fovy,float asp,float zNear,float zFar);
void   mat4print(const char* text,float m[16]);
void   mat3print(const char* text,float m[9]);
