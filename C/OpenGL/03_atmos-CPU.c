// gcc -std=gnu17 -O3 -Wall 03_atmos-CPU.c -lglut -lGLU -lGL -lm -o atmos.out


////////// INIT ////////////////////////////////////////////////////////////////////////////////////

#include "TriNet.h"


////////// SETTINGS ////////////////////////////////////////////////////////////////////////////////
const float _SCALE = 10.0; //- Scale Dimension



////////// HELPER FUNCTIONS ////////////////////////////////////////////////////////////////////////


////////// PROJECT STRUCTS /////////////////////////////////////////////////////////////////////////


///// Triangular Cell /////////////////////////////////////////////////////

typedef struct{
    // Holds particle info for one triangular cell

    // Bookkeeping //
    uint  Nmax; // ---- Max number of particles that can occupy this cell
    uint  insrtDex; //- Index for next insert
    uint  lost; // ---- Count of missing particles
    vec2f accel; // --- Per-frame change in velocity
    float speedLim; //- Max speed of any particle
    uint  ID; // ------ Triangle index in the mesh associated with this cell
    vec3u neighbors; // Local connectivity
    
    // Geometry //
    vec2f v1_2f; //- Vertex 1 in local frame
    vec2f v2_2f; //- Vertex 2 in local frame
    vec3f origin; // Origin in parent frame
    vec3f xBasis; // X direction in parent frame
    vec3f yBasis; // Y direction in parent frame
    
    // Update && Painting @ Heap //
    matx_Nx4f* prtLocVel; // Local position and velocity of each particle
    uint* /**/ triDices; //- Cell membership of each particle
    matx_Nx3f* pntGlbPos; // 3D position of each drawn point
    // uint* /**/ dstDices; // Destination cell IDs
    // matx_Nx6f* pntState; // Particles to be transferred

}TriCell;


TriCell* alloc_cell( uint prtclMax_ ){
    // Allocate mem for a `TriNet` with `Ntri_` faces and `Nvrt_` vertices (shared vertices allowed)
    TriCell* rtnStruct   = malloc( sizeof( *rtnStruct ) ); 
    rtnStruct->Nmax /**/ = prtclMax_;
    rtnStruct->prtLocVel = matrix_new_Nx4f( prtclMax_ );
    rtnStruct->triDices  = malloc( sizeof( uint ) * prtclMax_ );
    rtnStruct->pntGlbPos = matrix_new_Nx3f( prtclMax_ );
    // rtnStruct->dstDices  = malloc( sizeof( uint ) * ((uint) prtclMax_/4) );
    // rtnStruct->pntState  = matrix_new_Nx6f( (uint) prtclMax_/4 );
    return rtnStruct;
}


void delete_cell( TriCell* cell ){
    // Free mem for a `TriCell` 
    free( cell->prtLocVel );
    free( cell->triDices );
    free( cell->pntGlbPos );
    free( cell );
}


void init_cell( TriCell* cell, uint Nadd, float dimLim, uint id, const vec3u* neighbors_, 
                const vec2f* accel, float speedLim_ ){
    // Populate particles with zero velocity, Set speed limit
    
    // 0. Init
    uint actAdd = min_uint( cell->Nmax, Nadd );
    uint i /**/ = 0;
    
    // 1. Set static state
    cell->insrtDex = actAdd; // ------------------- Index for next insert
    cell->lost     = 0; // ------------------------ Count of missing particles
    set_vec2f( &(cell->accel), accel ); // -------- Per-frame change in velocity
    cell->speedLim = fabsf( speedLim_ ); // ------- Max speed of any particle
    cell->ID /*-*/ = id; // ----------------------- Triangle index in the mesh associated with this cell
    set_vec3u( &(cell->neighbors), neighbors_ ); // Local connectivity
    
    // 2. Set dynamic state: Particle position, speed, and membership
    for( ; i < Nadd; ++i ){
        load_row_from_4f( cell->prtLocVel, i, randf()*dimLim, randf()*dimLim, 0.0f, 0.0f );
        cell->triDices[i] = id;
    }
    for( ; i < cell->Nmax; ++i ){
        load_row_from_4f( cell->prtLocVel, i, 0.0f, 0.0f, 0.0f, 0.0f );
        cell->triDices[i] = UINT32_MAX;
    }
}


void set_cell_geo( TriCell* cell, const vec3f* v0_3f, const vec3f* v1_3f, const vec3f* v2_3f ){
    // Construct the local coordinate frame && Locate the vertices of the 2D cell
    vec3f xSide;  
    vec3f n;  
    vec3f v1delta;  
    vec3f v2delta;  
    // Calc local reference frame //
    sub_vec3f( &xSide, v1_3f, v0_3f );
    unit_vec3f( &(cell->xBasis), &xSide );
    get_CCW_tri_norm( &n, v0_3f, v1_3f, v2_3f );
    cross_vec3f( &(cell->yBasis), &n, &(cell->xBasis) );
    set_vec3f( &(cell->origin), v0_3f );
    // Calc planar reference frame //
    sub_vec3f( &v1delta, v1_3f, &(cell->origin) );
    sub_vec3f( &v2delta, v2_3f, &(cell->origin) );
    cell->v1_2f[0] = dot_vec3f( &(cell->xBasis), &v1delta );
    cell->v1_2f[1] = dot_vec3f( &(cell->yBasis), &v1delta );
    cell->v2_2f[0] = dot_vec3f( &(cell->xBasis), &v2delta );
    cell->v2_2f[1] = dot_vec3f( &(cell->yBasis), &v2delta );
    // printf( "%u: ", cell->ID );  print_vec3f( cell->origin );  print_vec3f( cell->xBasis );  print_vec3f( cell->yBasis );  nl();
}


void advance_particles( TriCell* cell ){
    // Perform one tick of the simulation
    float pX    = 0.0f;
    float pY    = 0.0f;
    float vX    = 0.0f;
    float vY    = 0.0f;
    uint  curID = cell->ID;
    float spdLm = cell->speedLim;

    // printf( "### %u ###", cell->ID );  nl();  

    // 1. For every particle
    for( uint i = 0; i < cell->Nmax; ++i ){
        // 2. If particle belongs to this cell, Then load data and perform updates
        if(cell->triDices[i] == curID){
            // 3. Load particle position
            pX = (*cell->prtLocVel)[i][0];
            pY = (*cell->prtLocVel)[i][1];
            // 4. Load particle velocity
            vX = (*cell->prtLocVel)[i][2];
            vY = (*cell->prtLocVel)[i][3];

            // printf("[%f,%f,%f,%f] --> ",pX, pY, vX, vY);

            // 5. Accelerate
            vX += cell->accel[0];
            vY += cell->accel[1];

            // 6. Apply per-direction speed limit
            vX /= fmaxf(fabsf(vX)/spdLm, 1.0);
            vY /= fmaxf(fabsf(vY)/spdLm, 1.0);

            // 7. Move particle
            pX += vX;
            pY += vY;

            // printf("[%f,%f,%f,%f]",pX, pY, vX, vY);  nl();

            // 8. Store updated particle info
            load_row_from_4f( cell->prtLocVel, i, pX, pY, vX, vY );
        }
    }

    // nl();
}


void project_particles_to_points( TriCell* cell ){
    // Lift 2D particles in this cell to 3D points ready to draw
    uint  curID = cell->ID;
    uint  j     = 0;
    float pX    = 0.0f;
    float pY    = 0.0f;
    vec3f xDelta = {0.0f,0.0f,0.0f};
    vec3f yDelta = {0.0f,0.0f,0.0f};
    vec3f posGlb = {0.0f,0.0f,0.0f};

    // printf( "### %u ###", cell->ID );  nl();  

    // 1. For every member particle, Calc 3D position for painting
    for( uint i = 0; i < cell->Nmax; ++i ){
        // 2. If particle belongs to this cell, Then calc its 3D position
        if(cell->triDices[i] == curID){
            // 3. Load particle position
            pX = (*cell->prtLocVel)[i][0];
            pY = (*cell->prtLocVel)[i][1];
            // 4. Calc 3D position
            scale_vec3f( &xDelta, &(cell->xBasis), pX );
            scale_vec3f( &yDelta, &(cell->yBasis), pY );
            add3_vec3f( &posGlb, &(cell->origin), &xDelta, &yDelta );
            
            // printf("[%f,%f] --> ",pX,pY); print_vec3f( cell->origin );  print_vec3f( xDelta );  print_vec3f( yDelta );  
            // printf( " --> " );  print_vec3f( posGlb );  nl();
            // 5. Store 3D position
            load_row_from_vec3f( cell->pntGlbPos, j, &posGlb );
            ++j;
        }
    }

    // nl();

    // 3. Clear out all rows not representing a member particle // NOT NEEDED?
    // for( ; j < cell->Nmax; ++j ){
    // 	load_row_from_3f( cell->pntGlbPos, j, 0.0f, 0.0f, 0.0f );
    // }
}


void draw_cell_points( TriCell* cell, vec3f pntColor ){
    // Draw all currently active particles belonging to this cell
    uint curID = cell->ID;
    project_particles_to_points( cell );
    glColor3f( pntColor[0] , pntColor[1] , pntColor[2] );
    glBegin( GL_POINTS );
    for( uint i = 0; i < cell->Nmax; ++i ){  
        if( cell->triDices[i] == curID ){  send_row_to_glVtx3f( cell->pntGlbPos, i );  }  
    }
    glEnd();
}


void rot_90deg_about_orig( vec2f* r, /*<<*/ const vec2f* v ){
    // Rotate `v` 90 degrees CCW about the origin, R^2
    // https://www.khanacademy.org/math/geometry/hs-geo-transformations/hs-geo-rotations/a/rotating-shapes
    (*r)[0] = -(*v)[1];
    (*r)[1] =  (*v)[0];
}


bool p_pnt_positive_angle_from_seg( const vec2f* pnt, const vec2f* segOrg, const vec2f* segEnd ){
    // Return true if the `pnt` is on the left of a segment defined bottom --to-> top, `segOrg` --to-> `segEnd `
    vec2f diffEnd   = {0.0f,0.0f};  sub_vec2f( &diffEnd, segEnd, segOrg );
    vec2f diffPnt   = {0.0f,0.0f};  sub_vec2f( &diffEnd, pnt   , segOrg );
    vec2f pointLeft = {0.0f,0.0f};  rot_90deg_about_orig( &pointLeft, &diffEnd );
    return (dot_vec2f( &diffPnt, &pointLeft ) >= 0.0f);
}


void determine_particle_exits( TriCell* cell ){
    // Detect if any currently active particles have departed from the cell
    vec2f posn_i = {0.0f,0.0f};
    vec2f zero2f = {0.0f,0.0f};
    uint curID = cell->ID;
    for( uint i = 0; i < cell->Nmax; ++i ){ 
        if( cell->triDices[i] == curID ){  
            posn_i[0] = (*cell->prtLocVel)[i][0];
            posn_i[1] = (*cell->prtLocVel)[i][1];
            // Check Neighbor 0 //
            if( posn_i[1] < 0.0f ){
                cell->triDices[i] = cell->neighbors[0];
                continue;
            }
            // Check Neighbor 1 //
            if( !p_pnt_positive_angle_from_seg( &posn_i, &(cell->v1_2f), &(cell->v2_2f) ) ){
                cell->triDices[i] = cell->neighbors[1];
                continue;
            }
            // Check Neighbor 2 //
            if( !p_pnt_positive_angle_from_seg( &posn_i, &(cell->v2_2f), &zero2f ) ){
                cell->triDices[i] = cell->neighbors[2];
                continue;
            }
        }
    }
}


void lift_pnt_2D_to_3D( vec3f* pnt3f, /*<<*/ vec2f* pnt2f, 
                        const vec3f* origin, const vec3f* xBasis, const vec3f* yBasis ){
    // Project the local 2D point to the global 3D frame
    vec3f xLocal;  scale_vec3f( &xLocal, xBasis, (*pnt2f)[0] );
    vec3f yLocal;  scale_vec3f( &yLocal, yBasis, (*pnt2f)[1] );
    /*---------*/  add3_vec3f( pnt3f, origin, &xLocal, &yLocal );
}


void project_pnt_3D_to_2D( vec2f* pnt2f, /*<<*/ vec3f* pnt3f,
                           const vec3f* origin, const vec3f* xBasis, const vec3f* yBasis ){
    // Project the global 3D point to the local 2D frame
    vec3f difPnt; sub_vec3f( &difPnt, pnt3f, origin );
    (*pnt2f)[0] = dot_vec3f( &difPnt, xBasis );
    (*pnt2f)[1] = dot_vec3f( &difPnt, yBasis );
}


void transfer_particles( TriCell* recvCell, /*<<*/ TriCell* sendCell ){
    // FIXME, START HERE: PACKAGE DEPARTURE STATE FOR CAPTURE BY NEIGHBOR CELL
    // FIXME: INTERPRET DEPARTURE STATE IN LOCAL REFERENCE FRAME

    vec2f postn2f_i = {0.0f,0.0f};
    vec2f veloc2f_i = {0.0f,0.0f};
    vec3f postn3f_i = {0.0f,0.0f,0.0f};
    vec3f veloc3f_i = {0.0f,0.0f,0.0f};
    vec2f postn2f_j = {0.0f,0.0f};
    vec2f veloc2f_j = {0.0f,0.0f};
    
    // 1. For every departed particle bound for `recvCell`, do ...
    for( uint i = 0; i < sendCell->Nmax; ++i ){ 
        if(sendCell->triDices[i] == recvCell->ID){
            // 2. Capture 3D state
            postn2f_i[0] = (*sendCell->prtLocVel)[i][0]; // pX
            postn2f_i[1] = (*sendCell->prtLocVel)[i][1]; // pY
            veloc2f_i[0] = (*sendCell->prtLocVel)[i][2]; // vX
            veloc2f_i[1] = (*sendCell->prtLocVel)[i][3]; // vY
            lift_pnt_2D_to_3D( &postn3f_i, &postn2f_i, 
                               &(sendCell->origin), &(sendCell->xBasis), &(sendCell->yBasis) );
            // FIXME, START HERE: PROJECT ON TO REF FRAME OF RECV CELL
            // FIXME: COUNT IF LOST @ RECV
            // FIXME: ADVANCE RECV INSERT INDEX (WRAPPED)
        }
    }
}

void backfill_particles( TriCell* cell ){
    // FIXME: ITERATE ALL ROWS, BACKFILL UP TO LOST, ASSUME DEPARTURES HAVE BEEN HANDLED
}



///// Particle Atmosphere /////////////////////////////////////////////////

typedef struct{
    // Holds net and cell data for a toy atmosphere with cells containing particles that move in a planar fashion
    uint /**/ Ncell; // -- Number of cells in this atmosphere
    uint /**/ NmaxCell; // Max number of particles any one cell can have
    TriNet*   net; // ---- Geometric information for atmosphere
    TriCell** cells; // -- Array of pointers to cells containing particles
}Atmos;


Atmos* alloc_atmos( uint Ncells_, uint prtclMax_ ){
    // Allocate memory for a toy atmosphere
    Atmos* rtnStruct    = malloc( sizeof( *rtnStruct ) ); 
    rtnStruct->Ncell    = Ncells_;
    rtnStruct->NmaxCell = prtclMax_;
    rtnStruct->cells    = malloc( sizeof( TriCell* ) * Ncells_ ); 
    for( uint i = 0; i < Ncells_; ++i ){
        rtnStruct->cells[i] = alloc_cell( prtclMax_ );
    }
    return rtnStruct;
}


void delete_atmos( Atmos* atmos ){
    // Free memory for a toy atmosphere
    for( uint i = 0; i < atmos->Ncell; ++i ){
        delete_cell( atmos->cells[i] );
    }
    free( atmos->cells );
    delete_net( atmos->net );
    free( atmos );
}


void init_atmos( Atmos* atmos, TriNet* filledNet, uint Nadd, float speedLim_, float accelLim_ ){
    // Set mesh, Allocate cells, Init cells, and Setup cell geometries
    vec3f v0     = {0.0f,0.0f,0.0f};
    vec3f v1     = {0.0f,0.0f,0.0f};
    vec3f v2     = {0.0f,0.0f,0.0f};
    vec3u nghbrs = {0,0,0};
    vec2f acl_i  = {0.0f,0.0f};
    // 1. Set mesh
    atmos->net = filledNet;
    // 2. For every cell, Setup ...
    for( uint i = 0; i < atmos->Ncell; ++i ){
        // 3. Allocate cell
        atmos->cells[i] = alloc_cell( atmos->NmaxCell );
        // 4. Calc params
        load_vec3f_from_row( &v0    , atmos->net->V, (*atmos->net->F)[i][0] );
        load_vec3f_from_row( &v1    , atmos->net->V, (*atmos->net->F)[i][1] );
        load_vec3f_from_row( &v2    , atmos->net->V, (*atmos->net->F)[i][2] );
        load_vec3u_from_row( &nghbrs, atmos->net->A, i                      );
        acl_i[0] = randf_range( -accelLim_, accelLim_ );
        acl_i[1] = randf_range( -accelLim_, accelLim_ );
        // 5. Init cell
        init_cell( atmos->cells[i], Nadd, diff_vec3f( &v0, &v1 ), i, &nghbrs, &acl_i, speedLim_ );
        // 6. Setup cell geometry
        set_cell_geo( atmos->cells[i], &v0, &v1, &v2 );
        // print_vec3f( v0 );  print_vec3f( v1 );  print_vec3f( v2 );  nl();
    }
}


void tick_atmos( Atmos* atmos ){
    // 2. For every cell, Step
    for( uint i = 0; i < atmos->Ncell; ++i ){  advance_particles( atmos->cells[i] );  }
}


void draw_all_cells( Atmos* atmos, vec3f pntColor ){
    // Draw the current particles in every cell
    for( uint i = 0; i < atmos->Ncell; ++i ){  draw_cell_points( atmos->cells[i], pntColor );  }
}


///// Icosahedron /////////////////////////////////////////////////////////

void populate_icos_vertices_and_faces( matx_Nx3f* V, matx_Nx3u* F, float radius ){
    // Load geometry for an icosahedron onto matrices `V` and `F` 
    /// Calc req'd constants ///
    float sqrt5 = (float) sqrt( 5.0 ); // ----------------------------------- Square root of 5
    float phi   = (float)( 1.0 + sqrt5 ) * 0.5; // ------------------------- The Golden Ratio
    float ratio = (float)sqrt( 10.0 + ( 2.0 * sqrt5 ) ) / ( 4.0 * phi ); // ratio of edge length to radius
    float a     = ( radius / ratio ) * 0.5;
    float b     = ( radius / ratio ) / ( 2.0f * phi );
    /// Load Vertices ///
    // Assume `V` already allocated for *12* vertices
    load_row_from_3f( V, 0,  0, b,-a ); 
    load_row_from_3f( V, 1,  b, a, 0 );
    load_row_from_3f( V, 2, -b, a, 0 );
    load_row_from_3f( V, 3,  0, b, a );
    load_row_from_3f( V, 4,  0,-b, a );
    load_row_from_3f( V, 5, -a, 0, b );
    load_row_from_3f( V, 6,  0,-b,-a );
    load_row_from_3f( V, 7,  a, 0,-b );
    load_row_from_3f( V, 8,  a, 0, b );
    load_row_from_3f( V, 9, -a, 0,-b );
    load_row_from_3f( V,10,  b,-a, 0 );
    load_row_from_3f( V,11, -b,-a, 0 );
    /// Load Faces ///
    // Assume `F` already allocated for *20* faces
    load_row_from_3u( F, 0,  2, 1, 0 );
    load_row_from_3u( F, 1,  1, 2, 3 );
    load_row_from_3u( F, 2,  5, 4, 3 );
    load_row_from_3u( F, 3,  4, 8, 3 );
    load_row_from_3u( F, 4,  7, 6, 0 );
    load_row_from_3u( F, 5,  6, 9, 0 );
    load_row_from_3u( F, 6, 11,10, 4 );
    load_row_from_3u( F, 7, 10,11, 6 );
    load_row_from_3u( F, 8,  9, 5, 2 );
    load_row_from_3u( F, 9,  5, 9,11 );
    load_row_from_3u( F,10,  8, 7, 1 );
    load_row_from_3u( F,11,  7, 8,10 );
    load_row_from_3u( F,12,  2, 5, 3 );
    load_row_from_3u( F,13,  8, 1, 3 );
    load_row_from_3u( F,14,  9, 2, 0 );
    load_row_from_3u( F,15,  1, 7, 0 );
    load_row_from_3u( F,16, 11, 9, 6 );
    load_row_from_3u( F,17,  7,10, 6 );
    load_row_from_3u( F,18,  5,11, 4 );
    load_row_from_3u( F,19, 10, 8, 4 );
}


TriNet* create_icos_mesh_only( float radius ){
    // Create an regular icosahedron (*without* unfolded net data)
    /// Allocate ///
    TriNet* icosNet = alloc_net( 20, 12 );
    /// Vertices and Faces ///
    populate_icos_vertices_and_faces( icosNet->V, icosNet->F, radius );
    /// Normals ///
    N_from_VF( icosNet->Ntri, icosNet->V, icosNet->F, icosNet->N );
    /// Return ///
    return icosNet;
}


TriNet* create_icos_VFNA( float radius ){
    // Create an regular icosahedron (*without* unfolded net data)
    /// Allocate ///
    TriNet* icosNet = alloc_net( 20, 12 );
    /// Vertices and Faces ///
    populate_icos_vertices_and_faces( icosNet->V, icosNet->F, radius );
    /// Normals ///
    N_from_VF( icosNet->Ntri, icosNet->V, icosNet->F, icosNet->N );
    /// Advacency ///
    populate_net_connectivity( icosNet, 0.005 );
    /// Return ///
    return icosNet;
}


///// Icos Atmos //////////////////////////////////////////////////////////

Atmos* create_icos_atmos( float radius, uint prtclMax_, uint Nadd, float speedLim_, float accelLim_ ){
    // Allocate and initialize a toy atmosphere based on an icosahedron
    TriNet* icosNet   = create_icos_VFNA( radius );
    Atmos*  rtnStruct = alloc_atmos( icosNet->Ntri, prtclMax_ );
    init_atmos( rtnStruct, icosNet, Nadd, speedLim_, accelLim_ );
    return rtnStruct;
}


////////// WINDOW STATE & VIEW PROJECTION //////////////////////////////////////////////////////////
float w2h /**/ =  0.0f; // Aspect ratio
int   fov /**/ = 55; // -- Field of view (for perspective)
float lastTime = 0.0f;


static void Project(){
    // Set projection
    // Adapted from code provided by Willem A. (Vlakkies) Schreüder  
    // NOTE: This function assumes that aspect rario will be computed by 'resize'
    
    //  Tell OpenGL we want to manipulate the projection matrix
    glMatrixMode( GL_PROJECTION );
    //  Undo previous transformations
    glLoadIdentity();
    
    gluPerspective( (double) fov , // -- Field of view angle, in degrees, in the y direction.
                    (double) w2h , // -- Aspect ratio , the field of view in the x direction. Ratio of x (width) to y (height).
                    (double) _SCALE/4.0 , //- Specifies the distance from the viewer to the near clipping plane (always positive).
                    (double) 4.0*_SCALE ); // Specifies the distance from the viewer to the far clipping plane (always positive).
    
    // Switch back to manipulating the model matrix
    glMatrixMode( GL_MODELVIEW );
    // Undo previous transformations
    glLoadIdentity();
}



////////// GEOMETRY & CAMERA ///////////////////////////////////////////////////////////////////////
Camera3D cam = { {4.0f, 2.0f, 2.0f}, {0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 1.0f} };
Atmos*   simpleAtmos;



////////// SIMULATION LOOP /////////////////////////////////////////////////////////////////////////

void tick(){
    // Simulation updates in between repaints
    tick_atmos( simpleAtmos );

    //  Tell GLUT it is necessary to redisplay the scene
    glutPostRedisplay();
}



////////// RENDERING LOOP //////////////////////////////////////////////////////////////////////////

float heartbeat( float targetFPS ){
    // Attempt to maintain framerate no greater than target. (Period is rounded down to next ms)
    float currTime   = 0.0f;
    float framTime   = 0.0f;
    float target_ms  = 1000.0f / targetFPS;
    static float FPS = 0.0f;
    framTime = (float) glutGet( GLUT_ELAPSED_TIME ) - lastTime;
    if( framTime < target_ms ){ sleep_ms( (long) (target_ms - framTime) );  }
    currTime = (float) glutGet( GLUT_ELAPSED_TIME );
    FPS = (1000.0f / (currTime - lastTime)) * 0.125f + FPS * 0.875f; // Filter for readable number
    lastTime = currTime;
    return FPS;
}


void display(){
    // Display the scene
    // Adapted from code provided by Willem Schreuder
    
    vec3f icsClr = {1.0f,1.0f,1.0f};
    vec3f atmClr = {0.0f,1.0f,0.0f};
    vec3f center = {0.0f,0.0f,0.0f};
    vec3f sphClr = {0.0, 14.0f/255.0f, 214.0f/255.0f};
    float FPS;

    //  Clear the image
    glClearDepth( 1.0f );
    glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );
    //  Reset previous transforms to the identity matrix
    glLoadIdentity();
    
    // Set view 
    look( cam );

    draw_sphere( center, 1.55f, sphClr );
    draw_net_wireframe( simpleAtmos->net, icsClr );
    draw_all_cells( simpleAtmos, atmClr );

    
    FPS = heartbeat( 60.0 );
    // Display status
    if( FPS < 40.0f ){
        glColor3f( 1.0f, 0.0f, 0.0f ); // Text Red
    }else if( FPS < 50.0f ){
        glColor3f( 249/255.0f, 255/255.0f, 99/255.0f ); // Text Yellow
    }else{
        glColor3f( 45/255.0f, 1.0f, 45/255.0f ); // Text Green
    }
    glWindowPos2i( 5 , 5 ); // Next raster operation relative to lower lefthand corner of the window
    Print( "FPS %f", FPS );

    //  Flush and swap
    glFlush();
    glutSwapBuffers();
}



////////// WINDOW STATE ////////////////////////////////////////////////////////////////////////////

void reshape( int width , int height ){
    // GLUT calls this routine when the window is resized
    // Calc the aspect ratio: width to the height of the window
    w2h = ( height > 0 ) ? (float) width / height : 1;
    // Set the viewport to the entire window
    glViewport( 0 , 0 , width , height );
    // Set projection
    Project();
}


int main( int argc , char* argv[] ){
    init_rand();
    
    
    // icos = create_icos_VFNA( 2.00 );
    simpleAtmos = create_icos_atmos( 2.00, 128, 64, 0.00125, 0.00006 );
    
    //  Initialize GLUT and process user parameters
    glutInit( &argc , argv );
    lastTime = (float) glutGet( GLUT_ELAPSED_TIME );
    
    //  Request 500 x 500 pixel window
    glutInitWindowSize( 1000 , 750 );
    
    //  Create the window
    glutCreateWindow( "LOOK AT THIS GODDAMN PLANET" );

    // NOTE: Set modes AFTER the window / graphics context has been created!
    //  Request double buffered, true color window 
    glutInitDisplayMode( GLUT_RGB | GLUT_DOUBLE | GLUT_DEPTH );
    // Enable z-testing at the full ranger
    glEnable( GL_DEPTH_TEST );
    glDepthRange( 0.0f , 1.0f );
    
    //  Tell GLUT to call "display" when the scene should be drawn
    glutDisplayFunc( display );

    // Tell GLUT to call "idle" when there is nothing else to do
    glutIdleFunc( tick );
    
    //  Tell GLUT to call "reshape" when the window is resized
    glutReshapeFunc( reshape );
    
    // //  Tell GLUT to call "special" when an arrow key is pressed
    // glutSpecialFunc( special );
    
    // //  Tell GLUT to call "key" when a key is pressed
    // glutKeyboardFunc( key );
    
    //  Pass control to GLUT so it can interact with the user
    glutMainLoop();
    
    // // Free memory
    // delete_net( icos );
    delete_atmos( simpleAtmos );
    
    //  Return code
    return 0;
}