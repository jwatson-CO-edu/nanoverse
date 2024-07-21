//
//  4x4 matrix functions
//  Matrices are interpreted as column major order using OpenGL convention
//  Authors: Willem A. (Vlakkies) Schreüder, https://www.prinmath.com/
//           James Watson, https://github.com/jwatson-CO-edu
void   mat4vulkan(int k);
void   identity_mtx44f( float mat[] );
float* make_identity_mtx44f( void );
void   copy_mtx44f( float mat[], /*<<*/ const float m[] );
void   mult_mtx44f( float mat[], float m[] );
void   Rx_mtx44f( float mat[], /*<<*/ float theta_rad ); // Load `mat` from R_x( `theta_rad` )
void   Ry_mtx44f( float mat[], /*<<*/ float theta_rad ); // Load `mat` from R_y( `theta_rad` ) 
void   Rz_mtx44f( float mat[], /*<<*/ float theta_rad ); // Load `mat` from R_z( `theta_rad` ) 
void   rotate_x_mtx44f( float mat[], /*<<*/ float theta_rad ); // Rotate `mat` with R_x( `theta_rad` ) 
void   rotate_y_mtx44f( float mat[], /*<<*/ float theta_rad ); // Rotate `mat` with R_y( `theta_rad` ) 
void   rotate_z_mtx44f( float mat[], /*<<*/ float theta_rad ); // Rotate `mat` with R_z( `theta_rad` ) 
// Get a matrix to increment the world Roll, Pitch, Yaw of the model
void   R_RPY_vehicle_mtx44f( float mat[], /*<<*/ float r_, float p_, float y_ );
void   rotate_angle_axis_mtx44f( float mat[], /*<<*/ float th_deg, float x, float y, float z );
void   translate_mtx44f( float mat[], /*<<*/ float dx, float dy, float dz );
void   set_position_mtx44f( float mat[], float x, float y, float z );
void   invert_homog( float mat[] ); // Use the special inversion form for homogeneous coords
void   scale_mtx44f( float mat[], float Sx, float Sy, float Sz );
void   mat4lookAt(float mat[16] , float Ex,float Ey,float Ez , float Cx,float Cy,float Cz , float Ux,float Uy,float Uz);
void   mat3normalMatrix(float mat[16],float inv[9]);
void   mat4normalMatrix(float mat[16],float inv[16]);
void   mat4ortho(float mat[],float left,float right,float bottom,float top,float zNear,float zFar);
void   mat4perspective(float mat[],float fovy,float asp,float zNear,float zFar);
void   print_mtx44f( const char* text, float m[16] );
void   mat3print(const char* text,float m[9]);
