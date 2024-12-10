////////// INIT ////////////////////////////////////////////////////////////////////////////////////
#include "toolbox.hpp"

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


void draw_Planet( Planet* plnt ){
    // Draw planet
    // Adapted from work by: Willem A. (Vlakkies) Schreüder, https://www.prinmath.com/
    mat4f xfrm = identity_mtx44f();
    xfrm = translate( xfrm, vec3f{plnt->loc.x, plnt->loc.y, plnt->loc.z} );
    xfrm = scale( xfrm, vec3f{plnt->R, plnt->R, plnt->R} );
    xfrm = rotate( xfrm, plnt->th, vec3f{0.0f, 0.0f, 1.0f} );
    xfrm = rotate( xfrm, plnt->ph, vec3f{0.0f, 1.0f, 0.0f} );

    glMatrixMode( GL_MODELVIEW );
    glPushMatrix();
    glMultMatrixf( value_ptr( xfrm ) );

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
    mat4f xfrm = identity_mtx44f();
    xfrm = translate( xfrm, vec3f{texMoon->loc.x, texMoon->loc.y, texMoon->loc.z} );
    xfrm = scale( xfrm, vec3f{texMoon->R, texMoon->R, texMoon->R} );
    xfrm = rotate( xfrm, texMoon->th, vec3f{0.0f, 0.0f, 1.0f} );
    xfrm = rotate( xfrm, texMoon->ph, vec3f{0.0f, 1.0f, 0.0f} );

    glMatrixMode( GL_MODELVIEW );
    glPushMatrix();
    glMultMatrixf( value_ptr( xfrm ) );

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