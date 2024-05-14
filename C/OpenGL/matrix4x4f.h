//
//  4x4 matrix functions
//  Matrices are interpreted as column major order using OpenGL convention
//  Authors: Willem A. (Vlakkies) Schreüder, https://www.prinmath.com/
//           James Watson, https://github.com/jwatson-CO-edu
void   mat4vulkan(int k);
void   identity_mtx44f(float mat[]);
float* make_identity_mtx44f( void );
void   mat4copy(float mat[],float m[]);
void   mult_mtx44f(float mat[],float m[]);
void   rot_x_mtx44f( float mat[], /*<<*/ float theta_rad ); // Load `mat` from R_x( `theta_rad` )
void   rot_y_mtx44f( float mat[], /*<<*/ float theta_rad ); // Load `mat` from R_y( `theta_rad` ) 
void   rot_z_mtx44f( float mat[], /*<<*/ float theta_rad ); // Load `mat` from R_z( `theta_rad` ) 
// Get a matrix to increment the world Roll, Pitch, Yaw of the model
void   rot_RPY_vehicle_mtx44f( float mat[], /*<<*/ float r_, float p_, float y_ );
void   rot_angle_axis_mtx44f(float mat[],float th,float x,float y,float z);
void   translate_mtx44f(float mat[],float dx,float dy,float dz);
void   scale_mtx44f(float mat[],float Sx,float Sy,float Sz);
void   mat4lookAt(float mat[16] , float Ex,float Ey,float Ez , float Cx,float Cy,float Cz , float Ux,float Uy,float Uz);
void   mat3normalMatrix(float mat[16],float inv[9]);
void   mat4normalMatrix(float mat[16],float inv[16]);
void   mat4ortho(float mat[],float left,float right,float bottom,float top,float zNear,float zFar);
void   mat4perspective(float mat[],float fovy,float asp,float zNear,float zFar);
void   mat4print(const char* text,float m[16]);
void   mat3print(const char* text,float m[9]);
