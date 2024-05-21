// Adapted from code by Song Ho Ahn (song.ahn@gmail.com)
////////// INIT ////////////////////////////////////////////////////////////////////////////////////

#include "geometry.h"
#include "matrix4x4f.h"



////////// VERTEX ARRAY OBJECTS (NESTED) ///////////////////////////////////////////////////////////

VAO_VNC_f* make_VAO_VNC_f( uint Ntri_ ){
    // Allocate the VAO at heap
    uint /*-*/ arrSize = sizeof( float ) * 9 * Ntri_;
    VAO_VNC_f* rtnVAO  = (VAO_VNC_f*) malloc( sizeof( VAO_VNC_f ) );
    /// Geo Info ///
    rtnVAO->Ntri  = Ntri_;
    rtnVAO->V     = (float*) malloc( arrSize );
    rtnVAO->N     = (float*) malloc( arrSize );
    rtnVAO->C     = (float*) malloc( arrSize );
    rtnVAO->bufID = 0;
    rtnVAO->arSiz = arrSize;
    /// Pose & Scale ///
    rtnVAO->relPose = make_identity_mtx44f();
    rtnVAO->ownPose = make_identity_mtx44f();
    rtnVAO->scale   = make_vec4f( 1.0f, 1.0f, 1.0f );
    rtnVAO->totPose = make_identity_mtx44f();
    /// Composite VAO ///
    rtnVAO->Nprt  = 0;
    rtnVAO->parts = NULL;
    /// Return ///
    return rtnVAO;
}


void allocate_N_VAO_VNC_parts( VAO_VNC_f* vao, uint N ){
    // Make space for `N` sub-part pointers
    vao->Nprt  = N;
    vao->parts = (void**) malloc( N * sizeof( VAO_VNC_f* ) );
}


VAO_VNC_f* get_part_i( VAO_VNC_f* vao, uint i ){
    // Fetch sub-VAO, NOTE: This is to wrap the need to cast the null pointer
    if( i < vao->Nprt )  return (VAO_VNC_f*) vao->parts[i];
    else /*----------*/  return NULL;
}


void delete_VAO_VNC_f( VAO_VNC_f* vao ){
    // Erase the VAO (and parts) at heap and the GPU
    if((vao->bufID) != 0){  glDeleteBuffersARB(1, &(vao->bufID));  }
    free( vao->V );
    free( vao->N );
    free( vao->C );
    free( vao->relPose );
    free( vao->ownPose );
    for( uint i = 0; i < vao->Nprt; ++i ){   delete_VAO_VNC_f( (VAO_VNC_f*) (vao->parts[i]) );  }
    free( vao );
}


void load_VAO_VNC_from_full_arrays( VAO_VNC_f* vao, /*<<*/ const float* Vsto, const float* Nsto, const float* Csto ){
    // Copy {V,N,C} from the specified arrays
    memcpy( vao->V, Vsto, vao->arSiz ); // Copy vertices
    memcpy( vao->N, Nsto, vao->arSiz ); // Copy normals
    memcpy( vao->C, Csto, vao->arSiz ); // Copy colors
}


void allocate_and_load_VAO_VNC_at_GPU( VAO_VNC_f* vao ){
    // Fetch & set buffer ID, and make space on the GPU for the VAO
    glGenBuffersARB( 1, &(vao->bufID) );
    glBindBufferARB( GL_ARRAY_BUFFER_ARB, vao->bufID );
    glBufferDataARB( GL_ARRAY_BUFFER_ARB, 3*(vao->arSiz), 0, GL_STATIC_DRAW_ARB );
    glBufferSubDataARB( GL_ARRAY_BUFFER_ARB, 0             , vao->arSiz, vao->V ); // copy vertices starting from 0 offest
    glBufferSubDataARB( GL_ARRAY_BUFFER_ARB, vao->arSiz    , vao->arSiz, vao->N ); // copy normals after vertices
    glBufferSubDataARB( GL_ARRAY_BUFFER_ARB, 2*(vao->arSiz), vao->arSiz, vao->C ); // copy colours after normals
    for( uint i = 0; i < vao->Nprt; ++i ){  allocate_and_load_VAO_VNC_at_GPU( get_part_i( vao, i ) );  }
}


void update_total_pose( VAO_VNC_f* vao ){
    // Compose relative, ownship, and scale transformations into `totPose`, relative to parent frame
    identity_mtx44f( vao->totPose );
    mult_mtx44f( vao->totPose, vao->relPose );
    mult_mtx44f( vao->totPose, vao->ownPose );
    scale_mtx44f( vao->totPose, vao->scale.x, vao->scale.y, vao->scale.z );
}


void calc_total_pose_part_i( float* mat, /*<<*/ VAO_VNC_f* vao, uint i ){
    // Get the total distal pose at `i` and store it in `mat`
    identity_mtx44f( mat );
    update_total_pose( vao );
    update_total_pose( get_part_i( vao, i ) );
    mult_mtx44f( mat, vao->totPose );
    mult_mtx44f( mat, get_part_i( vao, i )->totPose );
}


void draw_recur_VAO_VNC_f( VAO_VNC_f* vao ){
    // Inner draw function for `VAO_VNC_f`, NOTE: This function assumes GPU is set to draw VAO
    ulong arrSize = vao->arSiz;
    ulong dblSize = arrSize*2;

    update_total_pose( vao );

    glMatrixMode( GL_MODELVIEW );
    glPushMatrix();
    glMultMatrixf( vao->totPose );
    
    // bind VBOs with IDs and set the buffer offsets of the bound VBOs
    // When buffer object is bound with its ID, all pointers in gl*Pointer()
    // are treated as offset instead of real pointer.
    glBindBufferARB( GL_ARRAY_BUFFER_ARB, vao->bufID );

    // before draw, specify vertex and index arrays with their offsets
    glVertexPointer( 3, GL_FLOAT, 0, 0               );
    glNormalPointer(    GL_FLOAT, 0, (void*) arrSize );
    glColorPointer(  3, GL_FLOAT, 0, (void*) dblSize );

    glDrawArrays( GL_TRIANGLES, 0, 3*(vao->Ntri) );

    // it is good idea to release VBOs with ID 0 after use.
    // Once bound with 0, all pointers in gl*Pointer() behave as real
    // pointer, so, normal vertex array operations are re-activated
    glBindBufferARB( GL_ARRAY_BUFFER_ARB, 0 );

    for( uint i = 0; i < vao->Nprt; ++i ){  draw_recur_VAO_VNC_f( (VAO_VNC_f*) vao->parts[i] );  }

    glPopMatrix();
}


void draw_VAO_VNC_f( VAO_VNC_f* vao ){
    // Draw using "VBO Method" (See Song Ho Ahn code)
    // printf( "About to draw VAO ...\n" );
    
    // enable vertex arrays
    glEnableClientState( GL_VERTEX_ARRAY );
    glEnableClientState( GL_NORMAL_ARRAY );
    glEnableClientState( GL_COLOR_ARRAY  );

    draw_recur_VAO_VNC_f( vao );

    // disable vertex arrays
    glDisableClientState( GL_VERTEX_ARRAY );  
    glDisableClientState( GL_NORMAL_ARRAY );
    glDisableClientState( GL_COLOR_ARRAY  );
}


vec4f get_posn( VAO_VNC_f* vao ){
    // Get the position components of the homogeneous coordinates as a vector
    return make_vec4f( vao->totPose[12], vao->totPose[13], vao->totPose[14] );
}


void set_posn( VAO_VNC_f* vao, const vec4f posn ){
    // Set the position components of the homogeneous coordinates
    vao->ownPose[12] = posn.x;
    vao->ownPose[13] = posn.y;
    vao->ownPose[14] = posn.z;
}


void translate( VAO_VNC_f* vao, const vec4f delta ){
    // Increment the position components of the homogeneous coordinates by the associated `delta` components
    vao->ownPose[12] += delta.x;
    vao->ownPose[13] += delta.y;
    vao->ownPose[14] += delta.z;
}


void rotate_angle_axis_rad( VAO_VNC_f* vao, float angle_rad, const vec4f axis ){
    // Rotate the object by `angle_rad` about `axis`
    float op1[16];  identity_mtx44f( op1 );
    rotate_angle_axis_mtx44f( op1, angle_rad*180/M_PI, axis.x, axis.y, axis.z );
    // print_mtx44f( "Rotation Matrix:", op1 );
    mult_mtx44f( op1, vao->ownPose );
    copy_mtx44f( vao->ownPose, op1 );
}


void rotate_RPY_vehicle( VAO_VNC_f* vao, float r_, float p_, float y_ ){
    // Increment the world Roll, Pitch, Yaw of the model
    // NOTE: This is for airplanes that move forward in their own Z and have a wingspan across X
    float op1[16];  identity_mtx44f( op1 );
    R_RPY_vehicle_mtx44f( op1, r_, p_, y_ );
    mult_mtx44f( op1, vao->ownPose );
    copy_mtx44f( vao->ownPose, op1 );
}


void thrust_Z_vehicle( VAO_VNC_f* vao, float dZ ){
    // Move in the local Z direction by `dZ` 
    vec4f disp = make_vec4f( 0.0f, 0.0f, dZ );
    disp = mult_mtx44f_vec4f( vao->ownPose, disp );
    translate( vao, disp );
}



////////// CONSTRUCTION ////////////////////////////////////////////////////////////////////////////

VAO_VNC_f* VAO_from_TriNet_solid_color( TriNet* net, const vec4f color ){
    // Get a VAO from a `TriNet`
    // NOTE: This function assumes that the normals of the `net` are populated
    uint /*-*/ Nrows = net->Ntri;
    VAO_VNC_f* rtnVAO = make_VAO_VNC_f( net->Ntri );
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
        rtnVAO->V[ i*9   ] = v0.x;
        rtnVAO->V[ i*9+1 ] = v0.y;
        rtnVAO->V[ i*9+2 ] = v0.z;
        rtnVAO->V[ i*9+3 ] = v1.x;
        rtnVAO->V[ i*9+4 ] = v1.y;
        rtnVAO->V[ i*9+5 ] = v1.z;
        rtnVAO->V[ i*9+6 ] = v2.x;
        rtnVAO->V[ i*9+7 ] = v2.y;
        rtnVAO->V[ i*9+8 ] = v2.z;
        // 3. Load normals
        rtnVAO->N[ i*9   ] = n0.x;
        rtnVAO->N[ i*9+1 ] = n0.y;
        rtnVAO->N[ i*9+2 ] = n0.z;
        rtnVAO->N[ i*9+3 ] = n1.x;
        rtnVAO->N[ i*9+4 ] = n1.y;
        rtnVAO->N[ i*9+5 ] = n1.z;
        rtnVAO->N[ i*9+6 ] = n2.x;
        rtnVAO->N[ i*9+7 ] = n2.y;
        rtnVAO->N[ i*9+8 ] = n2.z;
        // 4. Load colors
        rtnVAO->C[ i*9   ] = color.r;
        rtnVAO->C[ i*9+1 ] = color.g;
        rtnVAO->C[ i*9+2 ] = color.b;
        rtnVAO->C[ i*9+3 ] = color.r;
        rtnVAO->C[ i*9+4 ] = color.g;
        rtnVAO->C[ i*9+5 ] = color.b;
        rtnVAO->C[ i*9+6 ] = color.r;
        rtnVAO->C[ i*9+7 ] = color.g;
        rtnVAO->C[ i*9+8 ] = color.b;
    }
    return rtnVAO;
}


VAO_VNC_f* VAO_from_TriNet_solid_color_transformed( TriNet* net, const vec4f color, const float* xfrm ){
    // Get a VAO from a `TriNet` with `xfrm` applied to all vertices and normals
    // NOTE: This function assumes that the normals of the `net` are populated
    uint /*-*/ Nrows = net->Ntri;
    VAO_VNC_f* rtnVAO = make_VAO_VNC_f( net->Ntri );
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
        rtnVAO->V[ i*9   ] = v0.x;
        rtnVAO->V[ i*9+1 ] = v0.y;
        rtnVAO->V[ i*9+2 ] = v0.z;
        rtnVAO->V[ i*9+3 ] = v1.x;
        rtnVAO->V[ i*9+4 ] = v1.y;
        rtnVAO->V[ i*9+5 ] = v1.z;
        rtnVAO->V[ i*9+6 ] = v2.x;
        rtnVAO->V[ i*9+7 ] = v2.y;
        rtnVAO->V[ i*9+8 ] = v2.z;
        // 3. Load normals
        rtnVAO->N[ i*9   ] = n0.x;
        rtnVAO->N[ i*9+1 ] = n0.y;
        rtnVAO->N[ i*9+2 ] = n0.z;
        rtnVAO->N[ i*9+3 ] = n1.x;
        rtnVAO->N[ i*9+4 ] = n1.y;
        rtnVAO->N[ i*9+5 ] = n1.z;
        rtnVAO->N[ i*9+6 ] = n2.x;
        rtnVAO->N[ i*9+7 ] = n2.y;
        rtnVAO->N[ i*9+8 ] = n2.z;
        // 4. Load colors
        rtnVAO->C[ i*9   ] = color.r;
        rtnVAO->C[ i*9+1 ] = color.g;
        rtnVAO->C[ i*9+2 ] = color.b;
        rtnVAO->C[ i*9+3 ] = color.r;
        rtnVAO->C[ i*9+4 ] = color.g;
        rtnVAO->C[ i*9+5 ] = color.b;
        rtnVAO->C[ i*9+6 ] = color.r;
        rtnVAO->C[ i*9+7 ] = color.g;
        rtnVAO->C[ i*9+8 ] = color.b;
    }
    return rtnVAO;
}


////////// SPECIFIC VAO ////////////////////////////////////////////////////////////////////////////

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


VAO_VNC_f* colorspace_cube_VAO_VNC_f( void ){
    // Make a colorful cube from the static array data
    printf( "About to allocate cube ...\n" );
    VAO_VNC_f* rtnVAO = make_VAO_VNC_f( 12 );
    printf( "About to populate cube ...\n" );
    load_VAO_VNC_from_full_arrays( rtnVAO, cubeV, cubeN, cubeC );
    return rtnVAO;
}


///// Tetrahedron /////////////////////////////////////////////////////////

VAO_VNC_f* tetrahedron_VAO_VNC_f( float radius, const vec4f color ){
    // Construct a tetrahedron VAO with flat-shaded normals and one solid color
    TriNet*    tetNet = create_tetra_mesh_only( radius );
    VAO_VNC_f* rtnVAO = VAO_from_TriNet_solid_color( tetNet, color );
    delete_net( tetNet );
    return rtnVAO;
}


VAO_VNC_f* tetrahedron_transformed_VAO_VNC_f( float radius, const vec4f color, const float* xfrm ){
    // Construct a tetrahedron VAO with flat-shaded normals and one solid color, with all vectors transformed
    TriNet*    tetNet = create_tetra_mesh_only( radius );
    VAO_VNC_f* rtnVAO = VAO_from_TriNet_solid_color_transformed( tetNet, color, xfrm );
    delete_net( tetNet );
    return rtnVAO;
}


///// Triangular Prism ////////////////////////////////////////////////////

VAO_VNC_f* triprism_transformed_VAO_VNC_f( float height, float triRad, const vec4f color, const float* xfrm ){
    // Construct a triangular prism VAO with flat-shaded normals and one solid color, with all vectors transformed
    TriNet*    priNet = create_triprism_mesh_only( height, triRad );
    VAO_VNC_f* rtnVAO = VAO_from_TriNet_solid_color_transformed( priNet, color, xfrm );
    delete_net( priNet );
    return rtnVAO;
}


///// Cube ////////////////////////////////////////////////////////////////

VAO_VNC_f* cube_VAO_VNC_f( float sideLen, const vec4f color ){
    // Construct a cube VAO with flat-shaded normals and one solid color
    TriNet*    cubNet = create_cube_mesh_only( sideLen );
    VAO_VNC_f* rtnVAO = VAO_from_TriNet_solid_color( cubNet, color );
    delete_net( cubNet );
    return rtnVAO;
}



///// Icosahedron /////////////////////////////////////////////////////////

VAO_VNC_f* icosahedron_VAO_VNC_f( float radius, const vec4f color ){
    // Construct a icosahedron VAO with flat-shaded normals and one solid color
    TriNet*    icsNet = create_icos_mesh_only( radius );
    VAO_VNC_f* rtnVAO = VAO_from_TriNet_solid_color( icsNet, color );
    delete_net( icsNet );
    return rtnVAO;
}


///// Icosphere ///////////////////////////////////////////////////////////

VAO_VNC_f* icosphere_VAO_VNC_f( float radius, uint div, const vec4f color ){
    // Construct a icosphere VAO with flat-shaded normals and one solid color
    TriNet*    sphNet = create_icosphere_mesh_only( radius, div );
    VAO_VNC_f* rtnVAO = VAO_from_TriNet_solid_color( sphNet, color );
    delete_net( sphNet );
    return rtnVAO;
}



////////// OTHER OBJECTS ///////////////////////////////////////////////////////////////////////////

///// Planar Surface //////////////////////////////////////////////////////

VAO_VNC_f* plane_XY_VAO_VNC_f( float xLen, float yLen, uint xDiv, uint yDiv, const vec4f color ){
    // Construct a icosphere VAO with flat-shaded normals and one solid color
    TriNet*    plnNet = create_plane_XY_mesh_only( xLen, yLen, xDiv, yDiv );
    VAO_VNC_f* rtnVAO = VAO_from_TriNet_solid_color( plnNet, color );
    delete_net( plnNet );
    return rtnVAO;
}