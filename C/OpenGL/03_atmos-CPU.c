// gcc -std=gnu17 -O3 -Wall 03_atmos-CPU.c -lglut -lGLU -lGL -lm -o atmos.out


////////// INIT ////////////////////////////////////////////////////////////////////////////////////

#include "TriNet.h"
#include "OGL_Geo.h"


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
    uint  ID; // ------ Triangle index in the mesh associated with this cell
    vec3u neighbors; // Local connectivity

    // Wind Dynamics //
    float accelLim;
    vec2f accel; // --- Per-frame change in velocity
    float speedLim; //- Max speed of any particle
    float diffusProb; // Probability to exchange wind energy between cells
    float diffusRate; // Rate of wind exchange
    float pertrbProb; // Probability to change wind accel randomly
    float pertrbRate; // Rate to blend in perturbation
    
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


void determine_particle_exits( TriCell* cell ){
    // Detect if any currently active particles have departed from the cell
    // NOTE: Once per cell, Per timestep
    vec2f posn_i = {0.0f,0.0f};
    vec2f zero2f = {0.0f,0.0f};
    uint curID = cell->ID;
    for( uint i = 0; i < cell->Nmax; ++i ){ 
        if( cell->triDices[i] == curID ){  
            posn_i[0] = (*cell->prtLocVel)[i][0];
            posn_i[1] = (*cell->prtLocVel)[i][1];
            // Check Neighbor 0 //
            if( !p_pnt_positive_angle_from_seg( &posn_i, &zero2f, &(cell->v1_2f) ) ){
                cell->triDices[i] = cell->neighbors[0];
                // print_vec2f(posn_i);printf(" beyond ");print_vec2f(zero2f);printf("---");print_vec2f(cell->v1_2f);nl();
                continue;
            }
            // Check Neighbor 1 //
            if( !p_pnt_positive_angle_from_seg( &posn_i, &(cell->v1_2f), &(cell->v2_2f) ) ){
                cell->triDices[i] = cell->neighbors[1];
                // print_vec2f(posn_i);printf(" beyond ");print_vec2f(cell->v1_2f);printf("---");print_vec2f(cell->v2_2f);nl();
                continue;
            }
            // Check Neighbor 2 //
            if( !p_pnt_positive_angle_from_seg( &posn_i, &(cell->v2_2f), &zero2f ) ){
                cell->triDices[i] = cell->neighbors[2];
                // print_vec2f(posn_i);printf(" beyond ");print_vec2f(cell->v2_2f);printf("---");print_vec2f(zero2f);nl();
                continue;
            }
        }
    }
}


void generate_particle_at_row( TriCell* cell, uint row, /*<<*/ uint searchWidth ){
    // Generate a particle with a slight clumping tendency
    vec2f position = {0.0f,0.0f};
    vec2f posnDiff = {0.0f,0.0f};
    vec2f posn_i   = {0.0f,0.0f};
    vec2f velocity = {cell->accel[0]*2.0f, cell->accel[1]*2.0f};
    vec2f velo_i   = {0.0f,0.0f};
    uint  accum    = 0;
    uint  qDex     = 0;
    uint  curID    = cell->ID;
    float dimLim   = diff_vec2f( &(cell->v1_2f), &(cell->v2_2f) );
    float stretch  = dimLim / 4.0f;
    position[0] = randf()*dimLim;
    position[1] = randf()*dimLim;
    for( uint i = 0; i < searchWidth; ++i ){
        qDex = (uint) (randf() * (cell->Nmax));
        if( cell->triDices[ qDex ] == curID ){
            ++accum;

            posn_i[0] = (*cell->prtLocVel)[ qDex ][0];
            posn_i[1] = (*cell->prtLocVel)[ qDex ][1];
            sub_vec2f( &posn_i  , &posn_i  , &position );
            add_vec2f( &posnDiff, &posnDiff, &posn_i   );

            velo_i[0] = (*cell->prtLocVel)[ qDex ][2];
            velo_i[1] = (*cell->prtLocVel)[ qDex ][3];
            add_vec2f( &velocity, &velocity, &velo_i );
        }
    }
    if( accum ){
        div_vec2f( &posnDiff, &posnDiff, (float) accum );
        scale_vec2f( &posnDiff, &posnDiff, stretch / norm_vec2f( &posnDiff ) );
        add_vec2f( &position, &position, &posnDiff );

        div_vec2f( &velocity, &velocity, (float) accum );
    }
    load_row_from_4f( cell->prtLocVel, row, position[0], position[1], velocity[0], velocity[1] );
}


void init_cell( TriCell* cell, uint id, const vec3u* neighbors_, 
                float accelLim_, float speedLim_, float diffusProb_, float diffusRate_, float pertrbProb_, float pertrbRate_ ){
    // Populate particles with zero velocity, Set speed limit
    
    // 0. Init
    
    vec2f acl = {0.0f,0.0f};
    acl[0] = randf_range( -accelLim_, accelLim_ );
    acl[1] = randf_range( -accelLim_, accelLim_ );

    // 1. Set static state
    
    cell->lost     = 0; // ------------------------ Count of missing particles
    set_vec2f( &(cell->accel), &acl ); // -------- Per-frame change in velocity
    cell->speedLim = fabsf( speedLim_ ); // ------- Max speed of any particle
    cell->ID /*-*/ = id; // ----------------------- Triangle index in the mesh associated with this cell
    set_vec3u( &(cell->neighbors), neighbors_ ); // Local connectivity
    cell->diffusProb = diffusProb_;
    cell->diffusRate = diffusRate_;
    cell->pertrbProb = pertrbProb_;
    cell->pertrbRate = pertrbRate_;
}


void set_cell_geo( TriCell* cell, uint Nadd, const vec3f* v0_3f, const vec3f* v1_3f, const vec3f* v2_3f ){
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
    cell->v1_2f[1] = 0.0f;
    cell->v2_2f[0] = dot_vec3f( &(cell->xBasis), &v2delta );
    cell->v2_2f[1] = dot_vec3f( &(cell->yBasis), &v2delta );
    
    uint actAdd = min_uint( cell->Nmax, Nadd );
    uint i /**/ = 0;
    cell->insrtDex = actAdd; // ------------------- Index for next insert
    // 2. Set dynamic state: Particle position, speed, and membership
    for( ; i < actAdd; ++i ){
        generate_particle_at_row( cell, i, 16 );
        cell->triDices[i] = cell->ID;
    }
    for( ; i < cell->Nmax; ++i ){
        load_row_from_4f( cell->prtLocVel, i, 0.0f, 0.0f, 0.0f, 0.0f );
        cell->triDices[i] = UINT32_MAX;
    }
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

            // 8. Store updated particle info
            load_row_from_4f( cell->prtLocVel, i, pX, pY, vX, vY );
        }
    }

    // determine_particle_exits( cell );
}


void project_particles_to_points( TriCell* cell ){
    // Lift 2D particles in this cell to 3D points ready to draw
    uint  curID = cell->ID;
    // uint  j     = 0;
    float pX    = 0.0f;
    float pY    = 0.0f;
    vec3f xDelta = {0.0f,0.0f,0.0f};
    vec3f yDelta = {0.0f,0.0f,0.0f};
    vec3f posGlb = {0.0f,0.0f,0.0f};

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
            // 5. Store 3D position
            load_row_from_vec3f( cell->pntGlbPos, i, &posGlb );
            // ++j;
        }
    }
}


void draw_cell_points( TriCell* cell, vec3f pntColor ){
    // Draw all currently active particles belonging to this cell
    uint curID = cell->ID;
    project_particles_to_points( cell );
    // glColor3f( pntColor[0] , pntColor[1] , pntColor[2] );
    // glBegin( GL_POINTS );
    for( uint i = 0; i < cell->Nmax; ++i ){  
        if( cell->triDices[i] == curID ){  send_row_to_glVtx3f( cell->pntGlbPos, i );  }  
        // if( cell->triDices[i] == 10 ){  send_row_to_glVtx3f( cell->pntGlbPos, i );  }  
    }
    // glEnd();
}


void transfer_particles( TriCell* recvCell, /*<<*/ TriCell* sendCell ){
    // Transfer all particles departing `sendCell` that are bound for `recvCell`
    // NOTE: 3X per cell, Per timestep

    // Outgoing State //
    vec2f postn2f_i = {0.0f,0.0f};
    vec2f veloc2f_i = {0.0f,0.0f};
    // Global State //
    vec3f postn3f = {0.0f,0.0f,0.0f};
    vec3f veloc3f = {0.0f,0.0f,0.0f};
    // Incoming State //
    vec2f postn2f_j = {0.0f,0.0f};
    vec2f veloc2f_j = {0.0f,0.0f};
    
    // 1. For every departed particle bound for `recvCell`, do ...
    for( uint i = 0; i < sendCell->Nmax; ++i ){ 
        if(sendCell->triDices[i] == recvCell->ID){
            // 2. Count the departed particle
            ++(sendCell->lost);
            // 2. Capture 3D state
            postn2f_i[0] = (*sendCell->prtLocVel)[i][0]; // pX
            postn2f_i[1] = (*sendCell->prtLocVel)[i][1]; // pY
            veloc2f_i[0] = (*sendCell->prtLocVel)[i][2]; // vX
            veloc2f_i[1] = (*sendCell->prtLocVel)[i][3]; // vY
            lift_pnt_2D_to_3D( &postn3f, &postn2f_i, &(sendCell->origin), &(sendCell->xBasis), &(sendCell->yBasis) );
            lift_vec_2D_to_3D( &veloc3f, &veloc2f_i, &(sendCell->xBasis), &(sendCell->yBasis) );
            // 3. Project 3D state onto neighboring cell
            project_pnt_3D_to_2D( &postn2f_j, &postn3f, &(recvCell->origin), &(recvCell->xBasis), &(recvCell->yBasis) );
            project_vec_3D_to_2D( &veloc2f_j, &veloc3f, &(recvCell->xBasis), &(recvCell->yBasis) );
            // 4. Load 2D state at insert point
            load_row_from_4f( recvCell->prtLocVel, recvCell->insrtDex, 
                              postn2f_j[0], postn2f_j[1], veloc2f_j[0], veloc2f_j[1] );
            // 5. If active, Then count overwrite, Else set particle active
            if( recvCell->triDices[ recvCell->insrtDex ] == recvCell->ID ){  ++(recvCell->lost);  }
            // 5. Else activated, count gain
            else{  
                recvCell->triDices[ recvCell->insrtDex ] = recvCell->ID;  
                if( recvCell->lost ){  --(recvCell->lost);  }
            }
            // 6. Advance the insertion index, making sure to wrap
            recvCell->insrtDex = (recvCell->insrtDex + 1) % (recvCell->Nmax);
            // 7. Deactivate departed particle
            sendCell->triDices[i] = UINT32_MAX;
        }
    }
}

void backfill_particles( TriCell* cell ){
    // Attempt to backfill lost particles, Assume that departures have been handled
    // float dimLim = diff_vec2f( &(cell->v1_2f), &(cell->v2_2f) );
    // float vX, vY;
    uint added = 0;
    uint addLim = (uint)(1.0f * (cell->lost) / 128.0f);
    for( uint i = 0; ((i < cell->Nmax ) && (added < addLim)); ++i ){ 
        if( cell->triDices[i] == UINT32_MAX ){  

            // vX = (*cell->prtLocVel)[ (i+1) % (cell->Nmax) ][2];
            // vY = (*cell->prtLocVel)[ (i+1) % (cell->Nmax) ][3];
            // load_row_from_4f( cell->prtLocVel, i, randf()*dimLim, randf()*dimLim, vX, vY );
            generate_particle_at_row( cell, i, 8 );
            ++added;

            cell->triDices[i] = cell->ID;
            --(cell->lost);
            cell->insrtDex = (i+1) % (cell->Nmax);
        }
    }
}


void cell_flow_interaction( TriCell* recvCell, /*<<*/ TriCell* sendCell ){
    // Perturb and diffuse per-cell wind acceleration 
    // WARNING: IF A CELL ACCEL EVER REACHES ZERO THE ATMOSPHERE ERASES ITSELF!
    vec2f acl /**/ = {0.0f,0.0f};
    vec2f tempSend = {0.0f,0.0f};
    vec2f tempRecv = {0.0f,0.0f};
    float prevLen  = 0.0f;
    float epsilon  = 0.000075;
    acl[0]  = randf_range( -(sendCell->accelLim), sendCell->accelLim );
    acl[1]  = randf_range( -(sendCell->accelLim), sendCell->accelLim );
    if( randf() <= sendCell->pertrbProb ){
        // prevLen = norm_vec2f( &(sendCell->accel) );    
        blend_vec2f( &(sendCell->accel), &acl, sendCell->pertrbRate, &(sendCell->accel), 1.0f-(sendCell->pertrbRate)  );
        if( norm_vec2f( &(sendCell->accel) ) < epsilon ){
            set_vec2f( &(sendCell->accel), &acl );
        }
    }
    if( randf() <= sendCell->diffusProb ){
        set_vec2f( &tempSend, &(sendCell->accel) );
        set_vec2f( &tempRecv, &(recvCell->accel) );
        
        prevLen = norm_vec2f( &tempSend );
        blend_vec2f( &(sendCell->accel), 
                     &tempRecv, sendCell->diffusRate, 
                     &tempSend, 1.0f-(sendCell->diffusRate)  );
        if( norm_vec2f( &(sendCell->accel) ) < epsilon ){
            // set_vec2f( &(sendCell->accel), &acl );
            stretch_to_len_vec2f( &(sendCell->accel), &(sendCell->accel), randf_range( prevLen, 10.0*epsilon ) );
        }

        prevLen = norm_vec2f( &tempRecv );
        blend_vec2f( &(recvCell->accel), 
                     &tempSend, recvCell->diffusRate, 
                     &tempRecv, 1.0f-(recvCell->diffusRate)  );
        if( norm_vec2f( &(recvCell->accel) ) < epsilon ){
            // acl[0]  = randf_range( -(recvCell->accelLim), recvCell->accelLim );
            // acl[1]  = randf_range( -(recvCell->accelLim), recvCell->accelLim );
            // set_vec2f( &(recvCell->accel), &acl );
            stretch_to_len_vec2f( &(sendCell->accel), &(sendCell->accel), randf_range( prevLen, 10.0*epsilon ) );
        }
    }
    if( randf() <= (sendCell->pertrbProb)*4.0f ){
        prevLen = norm_vec2f( &(recvCell->accel) );
        stretch_to_len_vec2f( &(recvCell->accel), &(recvCell->accel), randf_range( prevLen, 10.0*epsilon ) );
        prevLen = norm_vec2f( &(sendCell->accel) );
        stretch_to_len_vec2f( &(sendCell->accel), &(sendCell->accel), randf_range( prevLen, 10.0*epsilon ) );
    }
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


void init_atmos( Atmos* atmos, TriNet* filledNet, uint Nadd, 
                 float speedLim_, float accelLim_, float diffusProb_, float diffusRate_, float pertrbProb_, float pertrbRate_ ){
    // Set mesh, Allocate cells, Init cells, and Setup cell geometries
    vec3f v0     = {0.0f,0.0f,0.0f};
    vec3f v1     = {0.0f,0.0f,0.0f};
    vec3f v2     = {0.0f,0.0f,0.0f};
    vec3u nghbrs = {0,0,0};
    // vec2f acl_i  = {0.0f,0.0f};
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
        // 5. Init cell
        init_cell( atmos->cells[i], i, &nghbrs, 
                   accelLim_, speedLim_, diffusProb_, diffusRate_, pertrbProb_, pertrbRate_ );
        // 6. Setup cell geometry
        set_cell_geo( atmos->cells[i], Nadd, &v0, &v1, &v2 );
        // print_vec3f( v0 );  print_vec3f( v1 );  print_vec3f( v2 );  nl();
    }
}


void perform_all_transfers_from_i( Atmos* atmos, uint i ){
    // Transfer particles and flows between cell `i` and its neighbors
    uint n_j = 0;
    for( uint j = 0; j < 3; ++j ){
        n_j = atmos->cells[i]->neighbors[j];
        transfer_particles( atmos->cells[n_j], atmos->cells[i] );
        cell_flow_interaction( atmos->cells[n_j], atmos->cells[i] );
    }
}


void tick_atmos( Atmos* atmos ){
    // 2. For every cell, Step
    for( uint i = 0; i < atmos->Ncell; ++i ){  
        backfill_particles( atmos->cells[i] );
        advance_particles( atmos->cells[i] );  
        perform_all_transfers_from_i( atmos, i );
        determine_particle_exits( atmos->cells[i] );
    }
}


void naturalize_flows( Atmos* atmos, uint Nflo, uint Nsim ){
    // Attempt to even out awkward flows & clouds
    uint n_j  = 0;
    uint Nact = max_uint( Nflo, Nsim );
    // 0. For the max number of runs, Do ...
    for( uint k = 0; k < Nact; ++k ){  
        // 1. For `Nflo` iterations, Even out flows to reduce opposing fronts
        for( uint i = 0; ((k < Nflo)&&(i < atmos->Ncell)); ++i ){  
            for( uint j = 0; j < 3; ++j ){
                n_j = atmos->cells[i]->neighbors[j];
                transfer_particles( atmos->cells[n_j], atmos->cells[i] );
                cell_flow_interaction( atmos->cells[n_j], atmos->cells[i] );
            }
        }
        // 1. For `Nsim` iterations, Advance sim to shear and dissipate init triangular clouds
        if(k < Nsim){  tick_atmos( atmos );  }
    }
}


void draw_all_cells( Atmos* atmos, vec3f pntColor ){
    // Draw the current particles in every cell
    glColor3f( pntColor[0] , pntColor[1] , pntColor[2] );
    glBegin( GL_POINTS );
    for( uint i = 0; i < atmos->Ncell; ++i ){  draw_cell_points( atmos->cells[i], pntColor );  }
    glEnd();
}


///// Spherical Atmos /////////////////////////////////////////////////////

Atmos* create_icosphere_atmos( float radius, uint div, uint prtclMax_, uint Nadd, float speedLim_, 
                               float accelLim_, float diffusProb_, float diffusRate_, float pertrbProb_, float pertrbRate_ ){
    // Allocate and initialize a toy atmosphere based on an icosahedron
    TriNet* sphrNet   = create_icosphere_VFNA( radius, div );
    Atmos*  rtnStruct = alloc_atmos( sphrNet->Ntri, prtclMax_ );
    init_atmos( rtnStruct, sphrNet, Nadd, speedLim_, accelLim_, diffusProb_, diffusRate_, pertrbProb_, pertrbRate_ );
    return rtnStruct;
}



////////// PROGRAM SETTINGS ////////////////////////////////////////////////////////////////////////
float _SPHERE_RADIUS =   2.15f;
float _ATMOS_RADIUS  =   2.25f;
uint  _ICOS_SUBDIVID =   6;
uint  _MAX_PRT_CELL  = 300; 
uint  _INIT_PRT_CELL = 200; 
float _SPEED_LIMIT   =   0.0075;
float _ACCEL_LIMIT   =   0.00020;
float _DIFFUS_PROB   = 1.0f/2000.0f;
float _DIFFUS_RATE   = 0.0625;
float _PERTURB_PROB  = 1.0f/25.0f;
float _PERTURB_RATE  = 0.75;
uint  _N_ATMOS_NATR  = 25;
uint  _N_WARM_UP     = 65;


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
    
    // vec3f icsClr = {1.0f,1.0f,1.0f};
    vec3f atmClr = {1.0f,1.0f,1.0f};
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

    draw_sphere( center, _SPHERE_RADIUS, sphClr );
    // draw_net_wireframe( simpleAtmos->net, icsClr );
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



////////// WINDOW RESIZING /////////////////////////////////////////////////////////////////////////

void reshape( int width , int height ){
    // GLUT calls this routine when the window is resized
    // Calc the aspect ratio: width to the height of the window
    w2h = ( height > 0 ) ? (float) width / height : 1;
    // Set the viewport to the entire window
    glViewport( 0 , 0 , width , height );
    // Set projection
    Project();
}


////////// MAIN ////////////////////////////////////////////////////////////////////////////////////

int main( int argc , char* argv[] ){
    init_rand();
    
    // icos = create_icos_VFNA( 2.00 );
    // simpleAtmos = create_icos_atmos( 2.00, 1024, 512, 0.00125, 0.00006 );
    simpleAtmos = create_icosphere_atmos( 
        _ATMOS_RADIUS, _ICOS_SUBDIVID, _MAX_PRT_CELL, _INIT_PRT_CELL, 
        _SPEED_LIMIT, _ACCEL_LIMIT, _DIFFUS_PROB, _DIFFUS_RATE, _PERTURB_PROB, _PERTURB_RATE
    );
    naturalize_flows( simpleAtmos, _N_ATMOS_NATR, _N_WARM_UP );


    // p_net_faces_outward_convex( 
    //     simpleAtmos->net->Ntri, 
    //     simpleAtmos->net->Nvrt, 
    //     simpleAtmos->net->V, 
    //     simpleAtmos->net->F, 
    //     simpleAtmos->net->N 
    // );
    
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