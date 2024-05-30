// Adapted from code by Song Ho Ahn (song.ahn@gmail.com)
////////// INIT ////////////////////////////////////////////////////////////////////////////////////

#include "geometry.h"
#include "matrix4x4f.h"



////////// VERTEX ARRAY OBJECTS (NESTED) ///////////////////////////////////////////////////////////

VNCT_f* make_VNCT_f( uint Ntri_ ){
    // Allocate the VBO at heap
    uint    arrSize = sizeof( float ) * 9 * Ntri_;
    uint    texSize = sizeof( float ) * 6 * Ntri_;
    VNCT_f* rtnVBO  = (VNCT_f*) malloc( sizeof( VNCT_f ) );
    /// Geo Info ///
    rtnVBO->Ntri  = Ntri_;
    rtnVBO->V     = (float*) malloc( arrSize );
    rtnVBO->N     = (float*) malloc( arrSize );
    rtnVBO->bufID = 0;
    rtnVBO->arSiz = arrSize;
    /// Color Info ///
    rtnVBO->p_clr = false; // Flag: Are vertex colors being used?
    rtnVBO->C     = (float*) malloc( arrSize );
    /// Texture Info ///
    rtnVBO->p_tex = false; // Flag: Is a texture being used?
    rtnVBO->T     = (float*) malloc( texSize );
    rtnVBO->txSiz = texSize;
    rtnVBO->texID = 0;
    /// Pose & Scale ///
    rtnVBO->relPose = make_identity_mtx44f();
    rtnVBO->ownPose = make_identity_mtx44f();
    rtnVBO->scale   = make_vec4f( 1.0f, 1.0f, 1.0f );
    rtnVBO->totPose = make_identity_mtx44f();
    /// Composite VBO ///
    rtnVBO->Nprt  = 0;
    rtnVBO->parts = NULL;
    /// Return ///
    return rtnVBO;
}


void allocate_N_VBO_VNCT_parts( VNCT_f* vbo, uint N ){
    // Make space for `N` sub-part pointers
    vbo->Nprt  = N;
    vbo->parts = (void**) malloc( N * sizeof( VNCT_f* ) );
}


void set_texture( VNCT_f* vbo, const char* path ){
    // Load texture at GPU and get its handle
    vbo->texID = LoadTexBMP( path );
    vbo->p_tex = true;
}


VNCT_f* get_part_i( VNCT_f* vbo, uint i ){
    // Fetch sub-VBO, NOTE: This is to wrap the need to cast the null pointer
    if( i < vbo->Nprt )  return (VNCT_f*) vbo->parts[i];
    else /*----------*/  return NULL;
}


void delete_VNCT_f( VNCT_f* vbo ){
    // Erase the VBO (and parts) at heap and the GPU
    if((vbo->bufID) != 0){  glDeleteBuffersARB(1, &(vbo->bufID));  }
    free( vbo->V );
    free( vbo->N );
    free( vbo->C );
    free( vbo->T );
    free( vbo->relPose );
    free( vbo->ownPose );
    for( uint i = 0; i < vbo->Nprt; ++i ){   delete_VNCT_f( (VNCT_f*) (vbo->parts[i]) );  }
    free( vbo );
}


void load_VNC_from_full_arrays( VNCT_f* vbo, /*<<*/ const float* Vsto, const float* Nsto, const float* Csto ){
    // Copy {V,N,C} from the specified arrays
    memcpy( vbo->V, Vsto, vbo->arSiz ); // Copy vertices
    memcpy( vbo->N, Nsto, vbo->arSiz ); // Copy normals
    memcpy( vbo->C, Csto, vbo->arSiz ); // Copy colors
    vbo->p_clr = true;
}


void load_VNT_from_full_arrays( VNCT_f* vbo, /*<<*/ const float* Vsto, const float* Nsto, const float* Tsto ){
    // Copy {V,N,T} from the specified arrays
    memcpy( vbo->V, Vsto, vbo->arSiz ); // Copy vertices
    memcpy( vbo->N, Nsto, vbo->arSiz ); // Copy normals
    memcpy( vbo->T, Tsto, vbo->txSiz ); // Copy texture UV
    vbo->p_tex = true;
}


void allocate_and_load_VBO_VNC_at_GPU( VNCT_f* vbo ){
    // Fetch & set buffer ID, and make space on the GPU for the VBO
    glGenBuffersARB( 1, &(vbo->bufID) );
    glBindBufferARB( GL_ARRAY_BUFFER_ARB, vbo->bufID );
    glBufferDataARB( GL_ARRAY_BUFFER_ARB, 3*(vbo->arSiz), 0, GL_STATIC_DRAW_ARB );
    glBufferSubDataARB( GL_ARRAY_BUFFER_ARB, 0             , vbo->arSiz, vbo->V ); // copy vertices starting from 0 offest
    glBufferSubDataARB( GL_ARRAY_BUFFER_ARB, vbo->arSiz    , vbo->arSiz, vbo->N ); // copy normals after vertices
    glBufferSubDataARB( GL_ARRAY_BUFFER_ARB, 2*(vbo->arSiz), vbo->arSiz, vbo->C ); // copy colours after normals
    vbo->p_clr = true;
    for( uint i = 0; i < vbo->Nprt; ++i ){  allocate_and_load_VBO_VNC_at_GPU( get_part_i( vbo, i ) );  }
}


void allocate_and_load_VBO_VNT_at_GPU( VNCT_f* vbo ){
    // Fetch & set buffer ID, and make space on the GPU for the VBO
    glGenBuffersARB( 1, &(vbo->bufID) );
    glBindBufferARB( GL_ARRAY_BUFFER_ARB, vbo->bufID );
    glBufferDataARB( GL_ARRAY_BUFFER_ARB, 3*(vbo->arSiz), 0, GL_STATIC_DRAW_ARB );
    glBufferSubDataARB( GL_ARRAY_BUFFER_ARB, 0             , vbo->arSiz, vbo->V ); // copy vertices starting from 0 offest
    glBufferSubDataARB( GL_ARRAY_BUFFER_ARB, vbo->arSiz    , vbo->arSiz, vbo->N ); // copy normals after vertices
    glBufferSubDataARB( GL_ARRAY_BUFFER_ARB, 2*(vbo->arSiz), vbo->txSiz, vbo->T ); // copy UV after normals
    vbo->p_tex = true;
    for( uint i = 0; i < vbo->Nprt; ++i ){  allocate_and_load_VBO_VNT_at_GPU( get_part_i( vbo, i ) );  }
}


void update_total_pose( VNCT_f* vbo ){
    // Compose relative, ownship, and scale transformations into `totPose`, relative to parent frame
    identity_mtx44f( vbo->totPose );
    mult_mtx44f( vbo->totPose, vbo->relPose );
    mult_mtx44f( vbo->totPose, vbo->ownPose );
    scale_mtx44f( vbo->totPose, vbo->scale.x, vbo->scale.y, vbo->scale.z );
}


void calc_total_pose_part_i( float* mat, /*<<*/ VNCT_f* vbo, uint i ){
    // Get the total distal pose at `i` and store it in `mat`
    identity_mtx44f( mat );
    update_total_pose( vbo );
    update_total_pose( get_part_i( vbo, i ) );
    mult_mtx44f( mat, vbo->totPose );
    mult_mtx44f( mat, get_part_i( vbo, i )->totPose );
}


void draw_recur_VNC_f( VNCT_f* vbo ){
    // Inner draw function for `VNCT_f`, NOTE: This function assumes GPU is set to draw VBO
    ulong arrSize = vbo->arSiz;
    ulong dblSize = arrSize*2;

    update_total_pose( vbo );

    glMatrixMode( GL_MODELVIEW );
    glPushMatrix();
    glMultMatrixf( vbo->totPose );
    
    // bind VBOs with IDs and set the buffer offsets of the bound VBOs
    // When buffer object is bound with its ID, all pointers in gl*Pointer()
    // are treated as offset instead of real pointer.
    glBindBufferARB( GL_ARRAY_BUFFER_ARB, vbo->bufID );

    // before draw, specify vertex and index arrays with their offsets
    glVertexPointer( 3, GL_FLOAT, 0, 0               );
    glNormalPointer(    GL_FLOAT, 0, (void*) arrSize );
    glColorPointer(  3, GL_FLOAT, 0, (void*) dblSize );

    glDrawArrays( GL_TRIANGLES, 0, 3*(vbo->Ntri) );

    // it is good idea to release VBOs with ID 0 after use.
    // Once bound with 0, all pointers in gl*Pointer() behave as real
    // pointer, so, normal vertex array operations are re-activated
    glBindBufferARB( GL_ARRAY_BUFFER_ARB, 0 );

    for( uint i = 0; i < vbo->Nprt; ++i ){  draw_recur_VNC_f( (VNCT_f*) vbo->parts[i] );  }

    glPopMatrix();
}


void draw_VNC_f( VNCT_f* vbo ){
    // Draw using "VBO Method" (See Song Ho Ahn code)
    // printf( "About to draw VBO ...\n" );
    
    // enable vertex arrays
    glEnableClientState( GL_VERTEX_ARRAY );
    glEnableClientState( GL_NORMAL_ARRAY );
    glEnableClientState( GL_COLOR_ARRAY  );

    draw_recur_VNC_f( vbo );

    // disable vertex arrays
    glDisableClientState( GL_VERTEX_ARRAY );  
    glDisableClientState( GL_NORMAL_ARRAY );
    glDisableClientState( GL_COLOR_ARRAY  );
}


void draw_recur_VNT_f( VNCT_f* vbo ){
    // Inner draw function for `VNCT_f`, NOTE: This function assumes GPU is set to draw VBO
    ulong arrSize = vbo->arSiz;
    ulong dblSize = arrSize*2;

    update_total_pose( vbo );

    glColor4f( 1,1,1,1 ); // Set all color channels to full
    glEnable( GL_TEXTURE_2D );
    glBindTexture( GL_TEXTURE_2D , vbo->texID );

    glMatrixMode( GL_MODELVIEW );
    glPushMatrix();
    glMultMatrixf( vbo->totPose );
    
    // bind VBOs with IDs and set the buffer offsets of the bound VBOs
    // When buffer object is bound with its ID, all pointers in gl*Pointer()
    // are treated as offset instead of real pointer.
    glBindBufferARB( GL_ARRAY_BUFFER_ARB, vbo->bufID );

    // before draw, specify vertex and index arrays with their offsets
    glVertexPointer(   3, GL_FLOAT, 0, 0               );
    glNormalPointer(      GL_FLOAT, 0, (void*) arrSize );
    glTexCoordPointer( 2, GL_FLOAT, 0, (void*) dblSize );

    glDrawArrays( GL_TRIANGLES, 0, 3*(vbo->Ntri) );

    // it is good idea to release VBOs with ID 0 after use.
    // Once bound with 0, all pointers in gl*Pointer() behave as real
    // pointer, so, normal vertex array operations are re-activated
    glBindBufferARB( GL_ARRAY_BUFFER_ARB, 0 );

    for( uint i = 0; i < vbo->Nprt; ++i ){  draw_recur_VNT_f( (VNCT_f*) vbo->parts[i] );  }

    glPopMatrix();
}


void draw_VNT_f( VNCT_f* vbo ){
    // Draw using "VBO Method" (See Song Ho Ahn code)
    // printf( "About to draw VBO ...\n" );
    
    // enable vertex arrays
    glEnableClientState( GL_VERTEX_ARRAY        );
    glEnableClientState( GL_NORMAL_ARRAY        );
    glEnableClientState( GL_TEXTURE_COORD_ARRAY );

    draw_recur_VNT_f( vbo );

    // disable vertex arrays
    glDisableClientState( GL_VERTEX_ARRAY        );  
    glDisableClientState( GL_NORMAL_ARRAY        );
    glDisableClientState( GL_TEXTURE_COORD_ARRAY );
}


vec4f get_posn( VNCT_f* vbo ){
    // Get the position components of the homogeneous coordinates as a vector
    return make_vec4f( vbo->totPose[12], vbo->totPose[13], vbo->totPose[14] );
}


void set_posn( VNCT_f* vbo, const vec4f posn ){
    // Set the position components of the homogeneous coordinates
    vbo->ownPose[12] = posn.x;
    vbo->ownPose[13] = posn.y;
    vbo->ownPose[14] = posn.z;
}


void translate( VNCT_f* vbo, const vec4f delta ){
    // Increment the position components of the homogeneous coordinates by the associated `delta` components
    vbo->ownPose[12] += delta.x;
    vbo->ownPose[13] += delta.y;
    vbo->ownPose[14] += delta.z;
}


void rotate_angle_axis_rad( VNCT_f* vbo, float angle_rad, const vec4f axis ){
    // Rotate the object by `angle_rad` about `axis`
    float op1[16];  identity_mtx44f( op1 );
    rotate_angle_axis_mtx44f( op1, angle_rad*180/M_PI, axis.x, axis.y, axis.z );
    // print_mtx44f( "Rotation Matrix:", op1 );
    mult_mtx44f( op1, vbo->ownPose );
    copy_mtx44f( vbo->ownPose, op1 );
}


void rotate_RPY_vehicle( VNCT_f* vbo, float r_, float p_, float y_ ){
    // Increment the world Roll, Pitch, Yaw of the model
    // NOTE: This is for airplanes that move forward in their own Z and have a wingspan across X
    float op1[16];  identity_mtx44f( op1 );
    R_RPY_vehicle_mtx44f( op1, r_, p_, y_ );
    mult_mtx44f( op1, vbo->ownPose );
    copy_mtx44f( vbo->ownPose, op1 );
}


void thrust_Z_vehicle( VNCT_f* vbo, float dZ ){
    // Move in the local Z direction by `dZ` 
    vec4f disp = make_vec4f( 0.0f, 0.0f, dZ );
    // disp = mult_mtx44f_vec4f( vbo->ownPose, disp );
    translate( vbo, disp );
}



////////// CONSTRUCTION ////////////////////////////////////////////////////////////////////////////

VNCT_f* VBO_from_TriNet_solid_color( TriNet* net, const vec4f color ){
    // Get a VBO from a `TriNet`
    // NOTE: This function assumes that the normals of the `net` are populated
    uint /*-*/ Nrows = net->Ntri;
    VNCT_f* rtnVBO = make_VNCT_f( net->Ntri );
    vec4f /**/ v0, v1, v2, n0, n1, n2;
    // 1. For every triangle / row
    for( uint i = 0; i < Nrows; ++i ){
        // 2. Fetch vertices and normals
        v0 = net->V[ net->F[i].v0 ];
        v1 = net->V[ net->F[i].v1 ];
        v2 = net->V[ net->F[i].v2 ];
        n0 = net->N[ net->F[i].v0 ];
        n1 = net->N[ net->F[i].v1 ];
        n2 = net->N[ net->F[i].v2 ];
        // 3. Load vertices
        rtnVBO->V[ i*9   ] = v0.x;
        rtnVBO->V[ i*9+1 ] = v0.y;
        rtnVBO->V[ i*9+2 ] = v0.z;
        rtnVBO->V[ i*9+3 ] = v1.x;
        rtnVBO->V[ i*9+4 ] = v1.y;
        rtnVBO->V[ i*9+5 ] = v1.z;
        rtnVBO->V[ i*9+6 ] = v2.x;
        rtnVBO->V[ i*9+7 ] = v2.y;
        rtnVBO->V[ i*9+8 ] = v2.z;
        // 3. Load normals
        rtnVBO->N[ i*9   ] = n0.x;
        rtnVBO->N[ i*9+1 ] = n0.y;
        rtnVBO->N[ i*9+2 ] = n0.z;
        rtnVBO->N[ i*9+3 ] = n1.x;
        rtnVBO->N[ i*9+4 ] = n1.y;
        rtnVBO->N[ i*9+5 ] = n1.z;
        rtnVBO->N[ i*9+6 ] = n2.x;
        rtnVBO->N[ i*9+7 ] = n2.y;
        rtnVBO->N[ i*9+8 ] = n2.z;
        // 4. Load colors
        rtnVBO->C[ i*9   ] = color.r;
        rtnVBO->C[ i*9+1 ] = color.g;
        rtnVBO->C[ i*9+2 ] = color.b;
        rtnVBO->C[ i*9+3 ] = color.r;
        rtnVBO->C[ i*9+4 ] = color.g;
        rtnVBO->C[ i*9+5 ] = color.b;
        rtnVBO->C[ i*9+6 ] = color.r;
        rtnVBO->C[ i*9+7 ] = color.g;
        rtnVBO->C[ i*9+8 ] = color.b;
    }
    return rtnVBO;
}


VNCT_f* VBO_from_TriNet_solid_color_transformed( TriNet* net, const vec4f color, const float* xfrm ){
    // Get a VBO from a `TriNet` with `xfrm` applied to all vertices and normals
    // NOTE: This function assumes that the normals of the `net` are populated
    uint /*-*/ Nrows = net->Ntri;
    VNCT_f* rtnVBO = make_VNCT_f( net->Ntri );
    float /**/ rota[16];  
    vec4f /**/ v0, v1, v2, n0, n1, n2;
    copy_mtx44f( rota, xfrm );
    set_position_mtx44f( rota, 0.0f, 0.0f, 0.0f );
    // 1. For every triangle / row
    for( uint i = 0; i < Nrows; ++i ){
        // 2. Fetch vertices and normals
        v0 = mult_mtx44f_vec4f( xfrm, net->V[ net->F[i].v0 ] );
        v1 = mult_mtx44f_vec4f( xfrm, net->V[ net->F[i].v1 ] );
        v2 = mult_mtx44f_vec4f( xfrm, net->V[ net->F[i].v2 ] );
        n0 = mult_mtx44f_vec4f( rota, net->N[ net->F[i].v0 ] );
        n1 = mult_mtx44f_vec4f( rota, net->N[ net->F[i].v1 ] );
        n2 = mult_mtx44f_vec4f( rota, net->N[ net->F[i].v2 ] );
        // 3. Load vertices
        rtnVBO->V[ i*9   ] = v0.x;
        rtnVBO->V[ i*9+1 ] = v0.y;
        rtnVBO->V[ i*9+2 ] = v0.z;
        rtnVBO->V[ i*9+3 ] = v1.x;
        rtnVBO->V[ i*9+4 ] = v1.y;
        rtnVBO->V[ i*9+5 ] = v1.z;
        rtnVBO->V[ i*9+6 ] = v2.x;
        rtnVBO->V[ i*9+7 ] = v2.y;
        rtnVBO->V[ i*9+8 ] = v2.z;
        // 3. Load normals
        rtnVBO->N[ i*9   ] = n0.x;
        rtnVBO->N[ i*9+1 ] = n0.y;
        rtnVBO->N[ i*9+2 ] = n0.z;
        rtnVBO->N[ i*9+3 ] = n1.x;
        rtnVBO->N[ i*9+4 ] = n1.y;
        rtnVBO->N[ i*9+5 ] = n1.z;
        rtnVBO->N[ i*9+6 ] = n2.x;
        rtnVBO->N[ i*9+7 ] = n2.y;
        rtnVBO->N[ i*9+8 ] = n2.z;
        // 4. Load colors
        rtnVBO->C[ i*9   ] = color.r;
        rtnVBO->C[ i*9+1 ] = color.g;
        rtnVBO->C[ i*9+2 ] = color.b;
        rtnVBO->C[ i*9+3 ] = color.r;
        rtnVBO->C[ i*9+4 ] = color.g;
        rtnVBO->C[ i*9+5 ] = color.b;
        rtnVBO->C[ i*9+6 ] = color.r;
        rtnVBO->C[ i*9+7 ] = color.g;
        rtnVBO->C[ i*9+8 ] = color.b;
    }
    return rtnVBO;
}


////////// SPECIFIC VBO ////////////////////////////////////////////////////////////////////////////

///// Cube ////////////////////////////////////////////////////////////////

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
const float cubeV[]  = { 1, 1, 1,  -1, 1, 1,  -1,-1, 1,      // v0-v1-v2 (front)
                        -1,-1, 1,   1,-1, 1,   1, 1, 1,      // v2-v3-v0

                         1, 1, 1,   1,-1, 1,   1,-1,-1,      // v0-v3-v4 (right)
                         1,-1,-1,   1, 1,-1,   1, 1, 1,      // v4-v5-v0

                         1, 1, 1,   1, 1,-1,  -1, 1,-1,      // v0-v5-v6 (top)
                        -1, 1,-1,  -1, 1, 1,   1, 1, 1,      // v6-v1-v0

                        -1, 1, 1,  -1, 1,-1,  -1,-1,-1,      // v1-v6-v7 (left)
                        -1,-1,-1,  -1,-1, 1,  -1, 1, 1,      // v7-v2-v1

                        -1,-1,-1,   1,-1,-1,   1,-1, 1,      // v7-v4-v3 (bottom)
                         1,-1, 1,  -1,-1, 1,  -1,-1,-1,      // v3-v2-v7

                         1,-1,-1,  -1,-1,-1,  -1, 1,-1,      // v4-v7-v6 (back)
                        -1, 1,-1,   1, 1,-1,   1,-1,-1 };    // v6-v5-v4

// normal array
const float cubeN[] = { 0, 0, 1,   0, 0, 1,   0, 0, 1,      // v0-v1-v2 (front)
                        0, 0, 1,   0, 0, 1,   0, 0, 1,      // v2-v3-v0

                        1, 0, 0,   1, 0, 0,   1, 0, 0,      // v0-v3-v4 (right)
                        1, 0, 0,   1, 0, 0,   1, 0, 0,      // v4-v5-v0

                        0, 1, 0,   0, 1, 0,   0, 1, 0,      // v0-v5-v6 (top)
                        0, 1, 0,   0, 1, 0,   0, 1, 0,      // v6-v1-v0

                       -1, 0, 0,  -1, 0, 0,  -1, 0, 0,      // v1-v6-v7 (left)
                       -1, 0, 0,  -1, 0, 0,  -1, 0, 0,      // v7-v2-v1

                        0,-1, 0,   0,-1, 0,   0,-1, 0,      // v7-v4-v3 (bottom)
                        0,-1, 0,   0,-1, 0,   0,-1, 0,      // v3-v2-v7

                        0, 0,-1,   0, 0,-1,   0, 0,-1,      // v4-v7-v6 (back)
                        0, 0,-1,   0, 0,-1,   0, 0,-1 };    // v6-v5-v4

// color array
const float cubeC[] = { 1, 1, 1,   1, 1, 0,   1, 0, 0,      // v0-v1-v2 (front)
                        1, 0, 0,   1, 0, 1,   1, 1, 1,      // v2-v3-v0

                        1, 1, 1,   1, 0, 1,   0, 0, 1,      // v0-v3-v4 (right)
                        0, 0, 1,   0, 1, 1,   1, 1, 1,      // v4-v5-v0

                        1, 1, 1,   0, 1, 1,   0, 1, 0,      // v0-v5-v6 (top)
                        0, 1, 0,   1, 1, 0,   1, 1, 1,      // v6-v1-v0

                        1, 1, 0,   0, 1, 0,   0, 0, 0,      // v1-v6-v7 (left)
                        0, 0, 0,   1, 0, 0,   1, 1, 0,      // v7-v2-v1

                        0, 0, 0,   0, 0, 1,   1, 0, 1,      // v7-v4-v3 (bottom)
                        1, 0, 1,   1, 0, 0,   0, 0, 0,      // v3-v2-v7

                        0, 0, 1,   0, 0, 0,   0, 1, 0,      // v4-v7-v6 (back)
                        0, 1, 0,   0, 1, 1,   0, 0, 1 };    // v6-v5-v4


VNCT_f* colorspace_cube_VNC_f( void ){
    // Make a colorful cube from the static array data
    printf( "About to allocate cube ...\n" );
    VNCT_f* rtnVBO = make_VNCT_f( 12 );
    printf( "About to populate cube ...\n" );
    load_VNC_from_full_arrays( rtnVBO, cubeV, cubeN, cubeC );
    return rtnVBO;
}


///// Tetrahedron /////////////////////////////////////////////////////////

VNCT_f* tetrahedron_VNC_f( float radius, const vec4f color ){
    // Construct a tetrahedron VBO with flat-shaded normals and one solid color
    TriNet* tetNet = create_tetra_mesh_only( radius );
    VNCT_f* rtnVBO = VBO_from_TriNet_solid_color( tetNet, color );
    delete_net( tetNet );
    return rtnVBO;
}


VNCT_f* tetrahedron_transformed_VNC_f( float radius, const vec4f color, const float* xfrm ){
    // Construct a tetrahedron VBO with flat-shaded normals and one solid color, with all vectors transformed
    TriNet* tetNet = create_tetra_mesh_only( radius );
    VNCT_f* rtnVBO = VBO_from_TriNet_solid_color_transformed( tetNet, color, xfrm );
    delete_net( tetNet );
    return rtnVBO;
}


///// Triangular Prism ////////////////////////////////////////////////////

VNCT_f* triprism_transformed_VNC_f( float height, float triRad, const vec4f color, const float* xfrm ){
    // Construct a triangular prism VBO with flat-shaded normals and one solid color, with all vectors transformed
    TriNet* priNet = create_triprism_mesh_only( height, triRad );
    VNCT_f* rtnVBO = VBO_from_TriNet_solid_color_transformed( priNet, color, xfrm );
    delete_net( priNet );
    return rtnVBO;
}


///// Cube ////////////////////////////////////////////////////////////////

VNCT_f* cube_VNC_f( float sideLen, const vec4f color ){
    // Construct a cube VBO with flat-shaded normals and one solid color
    TriNet* cubNet = create_cube_mesh_only( sideLen );
    VNCT_f* rtnVBO = VBO_from_TriNet_solid_color( cubNet, color );
    delete_net( cubNet );
    return rtnVBO;
}


///// Octahedron //////////////////////////////////////////////////////////

VNCT_f* octahedron_VNC_f( float cornerWidth, float height, const vec4f color ){
    // Construct a cube VBO with flat-shaded normals and one solid color
    TriNet* octNet = create_octahedron_mesh_only( cornerWidth, height );
    VNCT_f* rtnVBO = VBO_from_TriNet_solid_color( octNet, color );
    delete_net( octNet );
    return rtnVBO;
}


///// Icosahedron /////////////////////////////////////////////////////////

VNCT_f* icosahedron_VNC_f( float radius, const vec4f color ){
    // Construct a icosahedron VBO with flat-shaded normals and one solid color
    TriNet* icsNet = create_icos_mesh_only( radius );
    VNCT_f* rtnVBO = VBO_from_TriNet_solid_color( icsNet, color );
    delete_net( icsNet );
    return rtnVBO;
}


///// Icosphere ///////////////////////////////////////////////////////////

VNCT_f* icosphere_VNC_f( float radius, uint div, const vec4f color ){
    // Construct a icosphere VBO with flat-shaded normals and one solid color
    TriNet* sphNet = create_icosphere_mesh_only( radius, div );
    VNCT_f* rtnVBO = VBO_from_TriNet_solid_color( sphNet, color );
    delete_net( sphNet );
    return rtnVBO;
}



////////// OTHER OBJECTS ///////////////////////////////////////////////////////////////////////////

///// Planar Surface //////////////////////////////////////////////////////

VNCT_f* plane_XY_VNC_f( float xLen, float yLen, uint xDiv, uint yDiv, const vec4f color ){
    // Construct a icosphere VBO with flat-shaded normals and one solid color
    TriNet* plnNet = create_plane_XY_mesh_only( xLen, yLen, xDiv, yDiv );
    VNCT_f* rtnVBO = VBO_from_TriNet_solid_color( plnNet, color );
    delete_net( plnNet );
    return rtnVBO;
}