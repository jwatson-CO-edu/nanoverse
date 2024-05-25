////////// INIT ////////////////////////////////////////////////////////////////////////////////////

#include "geometry.h"
#include "matrix4x4f.h"



////////// PROGRAM SETTINGS ////////////////////////////////////////////////////////////////////////

/// View Settings ///
const float _DRAW_DIST_MIN = 50.0f / 16.0f; // Scale Dimension
const float _DRAW_DIST_MAX = 50.0f * 16.0f; // Scale Dimension
const int   _FOV_DEG    =   55; // - Field of view (for perspective)
const float _TARGET_FPS =   60.0f; // Desired framerate
const int   _WINDOW_W   = 1200;
const int   _WINDOW_H   =  900;

/// Model Settings ///
const float _TNK_BODY_SCL = 1.0f;
const vec4f _TNK_BODY_CLR = {0.0f, 1.0f, 0.0f, 1.0f};
const float _TNK_WHL_SCL  = 0.25f;
const vec4f _TNK_WHL_CLR  = {0.0f , 0.0f , 1.0f, 1.0f};
const vec4f _TNK_GUN_CLR  = {0.50f, 0.75f, 1.0f, 1.0f};
const float _GRID_UNIT    =  1.0f;
const uint  _N_UNIT /*-*/ = 50;
const vec4f _GRID_CLR     = {0.5f, 0.5f, 0.5f, 1.0f};
const float _GRID_THICC   = 3.0f;

/// Movement Settings ///
const float _FRM_DISP = 0.05; // Units to move per frame
const float _FRM_TURN = 0.05; // Radians to move per pixel of mouse movement
const float _FWK_DISP = _FRM_DISP/2.0f; // Units to move per frame


/// Environment Settings ///
const int   _MOON_EMISS = 50;
const float _MOON_SHINY =  1.0f;

////////// PROGRAM STRUCTS /////////////////////////////////////////////////////////////////////////

///// Light Source ////////////////////////////////////////////////////////

typedef struct{
    // Light source for default Phong shading
    uint  ID; // ----- GL light source enum
    vec4f position; // Position in the world frame
    vec4f ambient;
	vec4f diffuse;
	vec4f specular;
}LightSource;


LightSource* make_white_light_source( const vec4f posn, uint sourcEnum, 
                                      int ambientPrcnt, int diffusePrcnt, int specularPrcnt ){
    // White light source
    LightSource* lite = (LightSource*) malloc( sizeof( LightSource ) );
    lite->ID /*-*/ = sourcEnum;
    lite->position = posn;
    lite->ambient  = make_vec4f( 0.01f*ambientPrcnt  , 0.01f*ambientPrcnt , 0.01f*ambientPrcnt  );
    lite->diffuse  = make_vec4f( 0.01f*diffusePrcnt  , 0.01f*diffusePrcnt , 0.01f*diffusePrcnt  );
	lite->specular = make_vec4f( 0.01f*specularPrcnt , 0.01f*specularPrcnt, 0.01f*specularPrcnt );
    return lite;
}


void illuminate_with_source( LightSource* lite ){
    // Use this `lite` in the scene
    float Position[] = { lite->position.x, lite->position.y, lite->position.z, lite->position.w };
    float Ambient[]  = { lite->ambient.r , lite->ambient.g , lite->ambient.b , lite->ambient.a  };
	float Diffuse[]  = { lite->diffuse.r , lite->diffuse.g , lite->diffuse.b , lite->diffuse.a  };
	float Specular[] = { lite->specular.r, lite->specular.g, lite->specular.b, lite->specular.a };
    // glColorMaterial( GL_FRONT_AND_BACK , GL_AMBIENT_AND_DIFFUSE );
	// glEnable( GL_COLOR_MATERIAL );
	//  Enable light 0
	glEnable( lite->ID );
	//  Set ambient, diffuse, specular components and position of light 0
    glLightfv( lite->ID, GL_POSITION , Position );
	glLightfv( lite->ID, GL_AMBIENT  , Ambient  );
	glLightfv( lite->ID, GL_DIFFUSE  , Diffuse  );
	glLightfv( lite->ID, GL_SPECULAR , Specular );

    // glDisable( lite->ID );
}


///// The Moon ////////////////////////////////////////////////////////////

typedef struct{
   char* name;
   char* surf;
   uint  surftex;
   float R;
   vec4f loc;
   float th;
   float ph;
}Planet;


static void Vertex( int th, int ph ){
    // Draw vertex in polar coordinates
    // Author: Willem A. (Vlakkies) Schreüder, https://www.prinmath.com/
    double x = Cos(th)*Cos(ph);
    double y = Sin(th)*Cos(ph);
    double z =         Sin(ph);
    glNormal3d(x,y,z);
    glTexCoord2d(th/360.0,ph/180.0+0.5);
    glVertex3d(x,y,z);
}


void draw_Planet( Planet* plnt ){
    // Draw planet
    // Author: Willem A. (Vlakkies) Schreüder, https://www.prinmath.com/
    float xfrm[16];  identity_mtx44f( xfrm );
    translate_mtx44f( xfrm, plnt->loc.x, plnt->loc.y, plnt->loc.z );
    scale_mtx44f( xfrm, plnt->R, plnt->R, plnt->R );
    rotate_z_mtx44f( xfrm, plnt->th );
    rotate_y_mtx44f( xfrm, plnt->ph );

    glMatrixMode( GL_MODELVIEW );
    glPushMatrix();
    glMultMatrixf( xfrm );

    //  Set texture
    glEnable( GL_TEXTURE_2D );
    glBindTexture( GL_TEXTURE_2D, plnt->surftex );
    //  Latitude bands
    glColor3f( 1, 1, 1 );
    for( int ph = -90; ph < 90; ph += 5 ){
        glBegin( GL_QUAD_STRIP );
        for( int th = 0; th <= 360; th += 5 ){
            Vertex( th, ph   );
            Vertex( th, ph+5 );
        }
        glEnd();
    }
    glDisable( GL_TEXTURE_2D );
    glPopMatrix();
}


Planet* make_Moon( char* texPath, float R_, const vec4f loc_, float th_, float ph_ ){
    Planet* moon = (Planet*) malloc( sizeof( Planet ) );
    moon->surf    = texPath;
    moon->surftex = LoadTexBMP( texPath );
    moon->R /*-*/ = R_;
    moon->loc     = loc_;
    moon->th /**/ = th_;
    moon->ph /**/ = ph_;
    return moon;
}


void draw_Moon( Planet* texMoon, int emission, float shiny ){
    // Like drawing a planet, but shiny
    // Adapted from code by: Willem A. (Vlakkies) Schreüder, https://www.prinmath.com/
    float xfrm[16];  identity_mtx44f( xfrm );
    translate_mtx44f( xfrm, texMoon->loc.x, texMoon->loc.y, texMoon->loc.z );
    scale_mtx44f( xfrm, texMoon->R, texMoon->R, texMoon->R );
    rotate_z_mtx44f( xfrm, texMoon->th );
    rotate_y_mtx44f( xfrm, texMoon->ph );

    glMatrixMode( GL_MODELVIEW );
    glPushMatrix();
    glMultMatrixf( xfrm );

    // float yellow[] = { 1.0f , 1.0f , 0.0f , 1.0f };
	float Emission[] = { 0.01f*emission, 0.01f*emission, 0.01f*emission, 1.0f };

    glColor3f( 1 , 1 , 1 );
    float shininess = shiny<0 ? 0 : pow( 2.0, shiny );
	glMaterialf( GL_FRONT_AND_BACK , GL_SHININESS , shininess );
	glMaterialfv( GL_FRONT_AND_BACK , GL_EMISSION , Emission );

    //  Set texture
    glEnable( GL_TEXTURE_2D );
    glBindTexture( GL_TEXTURE_2D, texMoon->surftex );
    //  Latitude bands
    glColor3f( 1, 1, 1 );
    for( int ph = -90; ph < 90; ph += 5 ){
        glBegin( GL_QUAD_STRIP );
        for( int th = 0; th <= 360; th += 5 ){
            Vertex( th, ph+5 );
            Vertex( th, ph   );
        }
        glEnd();
    }
    glDisable( GL_TEXTURE_2D );
    glPopMatrix();
}

///// Firework ////////////////////////////////////////////////////////////

typedef struct{
    // A bright projectile that explodes even more brightly, with particles!
    VAO_VNC_f* shell; // shell geometry
    vec4f /**/ color;
}Firework;


Firework* make_Firework( float width, const vec4f bodyClr ){
    // Alloc and construct firework shell
    Firework* rtnFw = (Firework*) malloc( sizeof( Firework ) );
    rtnFw->shell = octahedron_VAO_VNC_f( width, 2.0f*width, bodyClr );
    rtnFw->color = bodyClr;
    allocate_and_load_VAO_VNC_at_GPU( rtnFw->shell );
    return rtnFw;
}


///// TetraTank_mk0 ///////////////////////////////////////////////////////

typedef struct{
    // Tetrahedral vehicle that rolls on 3 icosahedra and translates in any direction on X-Y plane
    VAO_VNC_f* body;
    float*     w0pose; 
    float*     w1pose; 
    float*     w2pose; 
    float*     gnPose; 
    float /**/ tiltAng;
}TetraTank_mk0;


TetraTank_mk0* make_TetraTank_mk0( float bodyRad_m, const vec4f bodyClr, float wheelRad_m, const vec4f wheelClr ){
    // Alloc and construct tank geometry
    TetraTank_mk0* rtnTank = (TetraTank_mk0*) malloc( sizeof( TetraTank_mk0 ) );
    // Body //
    TriNet* tetNet = create_tetra_mesh_only( bodyRad_m );
    vec4f v0 /**/ = tetNet->V[0];
    vec4f v1 /**/ = tetNet->V[1];
    vec4f v2 /**/ = tetNet->V[2];
    vec4f v3 /**/ = tetNet->V[3];
    vec4f segCntr = seg_center( v0, v1 );
    float rotAngl = angle_between_vec4f( sub_vec4f( v2, segCntr ), make_vec4f( 0.0f, 0.0f, 1.0f ) );
    float op2[16];
    float op3[16];
    Rx_mtx44f( op2, -(M_PI/2.0f-rotAngl) );
    rtnTank->body = VAO_from_TriNet_solid_color_transformed( tetNet, bodyClr, op2 );
    translate_mtx44f( rtnTank->body->relPose, 0.0f, 0.0f, bodyRad_m );
    allocate_N_VAO_VNC_parts( rtnTank->body, 4 );
    // Wheels //
    for( ubyte i = 0; i < 3; ++i ){
        rtnTank->body->parts[i] = (void*) icosahedron_VAO_VNC_f( wheelRad_m, wheelClr );
    }
    rtnTank->w0pose = get_part_i( rtnTank->body, 0 )->ownPose; 
    rtnTank->w1pose = get_part_i( rtnTank->body, 1 )->ownPose; 
    rtnTank->w2pose = get_part_i( rtnTank->body, 2 )->ownPose;
    v0 = stretch_to_len_vec4f( v0, bodyRad_m + wheelRad_m*2.0f );
    v1 = stretch_to_len_vec4f( v1, bodyRad_m + wheelRad_m*2.0f );
    v2 = stretch_to_len_vec4f( v2, bodyRad_m + wheelRad_m*2.0f );
    mult_mtx44f( get_part_i( rtnTank->body, 0 )->relPose, op2 );
    mult_mtx44f( get_part_i( rtnTank->body, 1 )->relPose, op2 );
    mult_mtx44f( get_part_i( rtnTank->body, 2 )->relPose, op2 );
    translate_mtx44f( get_part_i( rtnTank->body, 0 )->relPose, v0.x, v0.y, v0.z );
    translate_mtx44f( get_part_i( rtnTank->body, 1 )->relPose, v1.x, v1.y, v1.z );
    translate_mtx44f( get_part_i( rtnTank->body, 2 )->relPose, v2.x, v2.y, v2.z );
    rtnTank->w0pose = get_part_i( rtnTank->body, 0 )->ownPose;
    rtnTank->w1pose = get_part_i( rtnTank->body, 1 )->ownPose;
    rtnTank->w2pose = get_part_i( rtnTank->body, 2 )->ownPose;

    // Turret //
    identity_mtx44f( op3 );
    rotate_x_mtx44f( op3, M_PI-rotAngl ); //+rotAngl );
    rotate_z_mtx44f( op3, M_PI/6.0f );
    
    rtnTank->body->parts[3] = (void*) triprism_transformed_VAO_VNC_f( bodyRad_m, wheelRad_m, _TNK_GUN_CLR, op3 );
    v3 = stretch_to_len_vec4f( v3, bodyRad_m + wheelRad_m*2.0f );
    mult_mtx44f( get_part_i( rtnTank->body, 3 )->relPose, op2 );
    translate_mtx44f( get_part_i( rtnTank->body, 3 )->relPose, v3.x, v3.y, v3.z );
    rtnTank->gnPose = get_part_i( rtnTank->body, 3 )->ownPose;
    
    // Alloc All @ GPU //
    allocate_and_load_VAO_VNC_at_GPU( rtnTank->body );
    rtnTank->tiltAng = rotAngl;
    // Cleanup && Return //
    delete_net( tetNet );
    return rtnTank;
}


void set_gunsight( TetraTank_mk0* tank, Camera3D* camera ){
    // Look down the barrel of tetratank
    float xfrm[16];
    float ofst = 0.5f;
    float stbk = 6.0f;
    vec4f eye = make_vec4f(  ofst*_TNK_BODY_SCL,  stbk*_TNK_BODY_SCL, -ofst*_TNK_BODY_SCL );
    vec4f luk = make_vec4f(  ofst*_TNK_BODY_SCL, -stbk*_TNK_BODY_SCL, -ofst*_TNK_BODY_SCL );
    vec4f upC = make_vec4f(  0.0f, 0.0f, 1.0f );
    calc_total_pose_part_i( xfrm, tank->body, 3 );
    // rotate_x_mtx44f( xfrm, M_PI+(tank->tiltAng) );
    rotate_x_mtx44f( xfrm, -M_PI/2.0f-(tank->tiltAng) );
    // camera->eyeLoc = add_vec4f( mult_mtx44f_vec4f( xfrm, eye ), cen );
    // camera->lookPt = add_vec4f( mult_mtx44f_vec4f( xfrm, luk ), cen );
    camera->eyeLoc = mult_mtx44f_vec4f( xfrm, eye );
    camera->lookPt = mult_mtx44f_vec4f( xfrm, luk );
    camera->upVctr = upC;
}


void align_firework_at_muzzle( TetraTank_mk0* tank, Firework* proj ){
    // Prepare to fire `proj` Z-wise from `tank` barrel
    float xfrm[16];
    calc_total_pose_part_i( xfrm, tank->body, 3 );
    // copy_mtx44f( get_part_i( tank->body, 3 )->relPose, xfrm );
    translate_mtx44f( xfrm, 0.0f, 3.0f*_TNK_WHL_SCL, _TNK_BODY_SCL );
    rotate_x_mtx44f( xfrm, -5.0f*M_PI/4.0 );
    copy_mtx44f( proj->shell->relPose, xfrm );
}


void roll_wheels_for_rel_body_move( TetraTank_mk0* tank, const vec4f relMov, float zTurn_rad ){
    // Rotate the wheels as if they convey the tank by relative `relMov` through rolling contact
    vec4f /**/ trnAxis;
    vec4f /**/ relAxis  = cross_vec4f( make_vec4f( 0.0f, 0.0f, 1.0f ), relMov );
    float /**/ roll_deg = norm_vec4f( relMov )/_TNK_WHL_SCL * 180.0f/M_PI;
    vec4f /**/ whlAxis;
    float /**/ matWhl[16];
    VAO_VNC_f* whl = NULL;
    for( ubyte i = 0; i < 3; ++i ){
        whl = get_part_i( tank->body, i );
        update_total_pose( whl );
        copy_mtx44f( matWhl, whl->ownPose );
        invert_homog( matWhl );

        trnAxis = unit_vec4f( get_posn_mtx44f( whl->relPose ) );
        // trnAxis.z = 0.0f;
        whlAxis = mult_mtx44f_vec4f( matWhl, trnAxis );
        // trnAxis = cross_vec4f( make_vec4f( 0.0f, 0.0f, 1.0f ), trnAxis );
        rotate_angle_axis_mtx44f( 
            whl->ownPose, 
            -zTurn_rad * 180.0f/M_PI * (_TNK_BODY_SCL+_TNK_WHL_SCL)/_TNK_WHL_SCL, 
            whlAxis.x, whlAxis.y, whlAxis.z 
        );
        
        whlAxis = mult_mtx44f_vec4f( matWhl, relAxis );
        rotate_angle_axis_mtx44f( whl->ownPose, roll_deg, whlAxis.x, whlAxis.y, whlAxis.z );
    }
}


////////// PROGRAM STATE ///////////////////////////////////////////////////////////////////////////

/// Geometry ///
TetraTank_mk0* tank     = NULL;
Firework* /**/ frwk     = NULL;
bool /*-----*/ fwActive = false;
VAO_VNC_f*     grnd     = NULL;
Planet* /*--*/ moon     = NULL;
LightSource*   lite     = NULL;
Camera3D /*-*/ cam /**/ = { {4.0f, 2.0f, 2.0f, 1.0f}, {0.0f, 0.0f, 0.0f, 1.0f}, {0.0f, 0.0f, 1.0f, 1.0f} };

/// Interaction ///
bool upArrw = false;
bool dnArrw = false;
bool rtArrw = false;
bool lfArrw = false;
bool space  = false;
bool tab    = false;
int  xLast  = INT32_MAX, xDelta = 0;
int  yLast  = INT32_MAX, yDelta = 0;



////////// WINDOW & VIEW STATE /////////////////////////////////////////////////////////////////////
float w2h = 0.0f; // Aspect ratio


static void project(){
	// Set projection
	// Adapted from code provided by Willem A. (Vlakkies) Schreüder  
	// NOTE: This function assumes that aspect rario will be computed by 'resize'
	// 1. Tell OpenGL we want to manipulate the projection matrix
	glMatrixMode( GL_PROJECTION );
	//  Undo previous transformations
	glLoadIdentity();
	gluPerspective( _FOV_DEG , // ------ Field of view angle, in degrees, in the y direction.
					w2h , // ----------- Aspect ratio , the field of view in the x direction. Ratio of x (width) to y (height).
					_DRAW_DIST_MIN , //- Specifies the distance from the viewer to the near clipping plane (always positive).
					_DRAW_DIST_MAX ); // Specifies the distance from the viewer to the far clipping plane (always positive).
	// 2. Switch back to manipulating the model matrix
	glMatrixMode( GL_MODELVIEW );
	// 3. Undo previous transformations
	glLoadIdentity();
}


void reshape( int width , int height ){
	// GLUT calls this routine when the window is resized
    // Adapted from code provided by Willem A. (Vlakkies) Schreüder  
	// 1. Calc the aspect ratio: width to the height of the window
	w2h = ( height > 0 ) ? (float) width / height : 1;
	// 2. Set the viewport to the entire window
	glViewport( 0 , 0 , width , height );
	// 3. Set projection
	project();
}


void capture_pointer( int x, int y ){
    int pad = 50;
    if( (x < pad) || (y < pad) || (x > (_WINDOW_W-pad)) || (y > (_WINDOW_H-pad)) )
        glutWarpPointer( _WINDOW_W/2, _WINDOW_H/2 ); // Retain cursor at center while space not pressed
}



////////// RENDERING ///////////////////////////////////////////////////////////////////////////////

void display(){
    // Refresh display

    //  Erase the window and the depth buffer
    glClearDepth( 1.0f );
    glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT | GL_STENCIL_BUFFER_BIT );
    glEnable( GL_DEPTH_TEST );
    glLoadIdentity();

    ///// DRAW LOOP BEGIN /////////////////////////////////////////////////
    
	//  Enable lighting , From this point until 'glDisable' lighting is applied
	glEnable( GL_LIGHTING );
    glColorMaterial( GL_FRONT_AND_BACK , GL_AMBIENT_AND_DIFFUSE );
	glEnable( GL_COLOR_MATERIAL );

    set_gunsight( tank, &cam );
    look( cam ); // ------------------ NOTE: Look THEN illuminate!
    illuminate_with_source( lite ); // NOTE: Illumination THEN transformation!

    // draw_Planet( moon );
    draw_Moon( moon, _MOON_EMISS, _MOON_SHINY );

    // draw_grid_org_XY( _GRID_UNIT, _N_UNIT, _N_UNIT, _GRID_THICC, _GRID_CLR );

    draw_VAO_VNC_f( grnd );

    draw_VAO_VNC_f( tank->body );

    if( fwActive )  draw_VAO_VNC_f( frwk->shell );

        
    glDisable( GL_LIGHTING );
    ///// DRAW LOOP END ///////////////////////////////////////////////////

    //  Display parameters
    glDisable( GL_DEPTH_TEST );
    glWindowPos2i( 5, 5 );
    glColor3f( 1.0f, 1.0f, 1.0f );
    Print( "FPS=%f", heartbeat_FPS( _TARGET_FPS ) );

    // Check for errors, Flush, and swap
	ErrCheck( "display" );
	glFlush();
	glutSwapBuffers();

}



////////// INTERACTION /////////////////////////////////////////////////////////////////////////////

void special_dn( int key, int x, int y ){
    // GLUT calls this routine when an arrow key is pressed
    if( key == GLUT_KEY_UP    )  upArrw = true;
    if( key == GLUT_KEY_DOWN  )  dnArrw = true;
    if( key == GLUT_KEY_RIGHT )  rtArrw = true;
    if( key == GLUT_KEY_LEFT  )  lfArrw = true;
}

void key_dn( ubyte key, int x, int y ){
    // GLUT calls this routine when a letter key is pressed
    if( key == 'w' )  upArrw = true;
    if( key == 's' )  dnArrw = true;
    if( key == 'd' )  rtArrw = true;
    if( key == 'a' )  lfArrw = true;
    if( key == 32  ){ // [Spacebar]
        space = true;
        glutSetCursor( GLUT_CURSOR_INHERIT );
    }
    if( (key == 9) && (!fwActive) ){  tab = true;  } // [Tab]
}

void special_up( int key, int x, int y ){
    // GLUT calls this routine when an arrow key is released
    if( key == GLUT_KEY_UP    )  upArrw = false;
    if( key == GLUT_KEY_DOWN  )  dnArrw = false;
    if( key == GLUT_KEY_RIGHT )  rtArrw = false;
    if( key == GLUT_KEY_LEFT  )  lfArrw = false;
}

void key_up( ubyte key, int x, int y ){
    // GLUT calls this routine when a letter key is released
    if( key == 'w' )  upArrw = false;
    if( key == 's' )  dnArrw = false;
    if( key == 'd' )  rtArrw = false;
    if( key == 'a' )  lfArrw = false;
    if( key == 32  ){ // [Spacebar]
        space = false;
        glutSetCursor( GLUT_CURSOR_NONE );
    }
    // if( key == 9 ){  tab = false;  } // [Tab]
}

void mouse_move( int x, int y ){
    // Mouse activity callback
    if( xLast < INT32_MAX ){
        xDelta = x - xLast;
        yDelta = y - yLast;    
    }
    xLast = x;
    yLast = y;
    if( !space )  capture_pointer( x, y );
}

////////// SIMULATION //////////////////////////////////////////////////////////////////////////////

void tick(){
    // Background work
    vec4f totMove   = make_0_vec4f();
    float turnAngle = -1.0f * xDelta * _FRM_TURN;
    float op1[16];  

    if( !space ){ // Disable driving while space is held down
        Rz_mtx44f( op1, turnAngle );
        mult_mtx44f( tank->body->ownPose, op1 );

        if( upArrw )  totMove = add_vec4f( totMove, make_vec4f(  0.0f,  1.0f, 0.0f ) );
        if( dnArrw )  totMove = add_vec4f( totMove, make_vec4f(  0.0f, -1.0f, 0.0f ) );
        if( rtArrw )  totMove = add_vec4f( totMove, make_vec4f(  1.0f,  0.0f, 0.0f ) );
        if( lfArrw )  totMove = add_vec4f( totMove, make_vec4f( -1.0f,  0.0f, 0.0f ) );

        totMove = stretch_to_len_vec4f( totMove, _FRM_DISP );

        identity_mtx44f( op1 );
        translate_mtx44f( op1, totMove.x, totMove.y, 0.0f );
        mult_mtx44f( tank->body->ownPose, op1 );
        
        roll_wheels_for_rel_body_move( tank, totMove, turnAngle );

        turnAngle = -1.0f * yDelta * _FRM_TURN / 2.0f;
        Rx_mtx44f( op1, turnAngle );
        mult_mtx44f( tank->gnPose, op1 );

        if( fwActive )  thrust_Z_vehicle( frwk->shell, -_FWK_DISP );

        if( tab ){
            align_firework_at_muzzle( tank, frwk );
            fwActive = true;
            tab /**/ = false;
        }
    }

    // Track only RELATIVE mouse movement!
    xDelta = 0;
    yDelta = 0;
    
    // 3. Set projection
	project();
    // Tell GLUT it is necessary to redisplay the scene
	glutPostRedisplay();
}




////////// MAIN ////////////////////////////////////////////////////////////////////////////////////


int main( int argc, char* argv[] ){

    init_rand();

    ///// Initialize GLUT /////////////////////////////////////////////////

    glutInit( &argc , argv );
    // initGL();

    // Request window with size specified in pixels
    glutInitWindowSize( _WINDOW_W, _WINDOW_H );

    // Create the window
    glutCreateWindow( "Vertex Array Object (VAO) Test" );

    // NOTE: Set modes AFTER the window / graphics context has been created!
    // Request double buffered, true color window 
    glutInitDisplayMode( GLUT_RGB | GLUT_DOUBLE | GLUT_DEPTH );
    
    glEnable( GL_CULL_FACE );
    //  OpenGL should normalize normal vectors
	glEnable( GL_NORMALIZE );
    glDepthRange( 0.0f , 1.0f ); 
    glutSetCursor( GLUT_CURSOR_NONE ); // Hide the cursor while in the window
    glutWarpPointer( _WINDOW_W/2, _WINDOW_H/2 );


    ///// Initialize Geometry /////////////////////////////////////////////
    vec4f gClr = make_vec4f( 31.0f/255.0f, 120.0f/255.0f, 55.0f/255.0f );
    vec4f fClr = make_vec4f( 1.0, 0.0f, 0.0f );
    vec4f mLoc = make_vec4f( 50.0f, 0.0f, 50.0f );
    tank = make_TetraTank_mk0( _TNK_BODY_SCL, _TNK_BODY_CLR, _TNK_WHL_SCL, _TNK_WHL_CLR );
    grnd = plane_XY_VAO_VNC_f( 2.0f*_GRID_UNIT*_N_UNIT, 2.0f*_GRID_UNIT*_N_UNIT, _N_UNIT, _N_UNIT, gClr );
    allocate_and_load_VAO_VNC_at_GPU( grnd );
    moon = make_Moon( "resources/moon.bmp", 4.0f, mLoc, 0.0f, -M_PI/4.0f );
    lite = make_white_light_source( stretch_to_len_vec4f( mLoc, 10.0f ), GL_LIGHT0, 5, 50, 100 );
    frwk = make_Firework( _TNK_WHL_SCL*2.0f, fClr );

    ///// Initialize GLUT Callbacks ///////////////////////////////////////

    //  Tell GLUT to call "display" when the scene should be drawn
    glutDisplayFunc( display );

    // Tell GLUT to call "idle" when there is nothing else to do
    glutIdleFunc( tick );
    
    //  Tell GLUT to call "reshape" when the window is resized
    glutReshapeFunc( reshape );
    
    //  Tell GLUT to call "special" when an arrow key is pressed or released
    glutSpecialFunc( special_dn );
    glutSpecialUpFunc( special_up );

    //  Tell GLUT to call "mouse" when mouse input arrives
    // glutMouseFunc( mouse ); // Clicks
    glutPassiveMotionFunc( mouse_move ); // Movement
    
    // //  Tell GLUT to call "key" when a key is pressed or released
    glutKeyboardFunc( key_dn );
    glutKeyboardUpFunc( key_up );

    ///// GO ///// GO ///// GO ////////////////////////////////////////////
    
    // Pass control to GLUT so it can interact with the user
    glutMainLoop();
    
    
    ///// Free Memory /////////////////////////////////////////////////////
    printf( "Cleanup!\n" );
    
    delete_VAO_VNC_f( tank->body );

    printf( "\n### DONE ###\n\n" );
    //  Return code
    return 0;
}