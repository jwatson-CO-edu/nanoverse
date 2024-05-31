////////// INIT ////////////////////////////////////////////////////////////////////////////////////

#include "geometry.h"
#include "matrix4x4f.h"



////////// PROGRAM SETTINGS ////////////////////////////////////////////////////////////////////////

/// View Settings ///
const float _DRAW_DIST_MIN = 10.0f / 16.0f; // Scale Dimension
const float _DRAW_DIST_MAX = 10.0f * 16.0f; // Scale Dimension
const int   _FOV_DEG    =   55; // - Field of view (for perspective)
const float _TARGET_FPS =   60.0f; // Desired framerate
const int   _WINDOW_W   = 1200;
const int   _WINDOW_H   =  900;



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


///// Firework ////////////////////////////////////////////////////////////

typedef struct{
    // A bright projectile that explodes even more brightly, with particles!
    VNCT_f* shell; // Shell geometry
    vec4f   color; // Shell color
    uint    Nsprk; // Number of sparkles
    uint    mode; //- 0: Shell, 1: Detonation
    uint    sprkShdr; // Particle system shader
    uint    xpldShdr; // Explosion animation shader
    // FIXME, START HERE: HANDLES FOR GPU PARTICLES BUFFERS
    // float*  posn; //- Sparkle position
    // float*  colr; //- Sparkle color, NOTE: Sparkle liveness is tracked by fading color
}Firework;

Firework* make_Firework( float width, const vec4f bodyClr, uint Nsprk_ ){
    // Setup the firework struct for rendering
    Firework* rtnFw = (Firework*) malloc( sizeof( Firework ) );
    /// CPU-Side ///
    rtnFw->shell = octahedron_VNC_f( width, 2.0f*width, bodyClr );
    rtnFw->color = bodyClr;
    rtnFw->Nsprk    = Nsprk_;
    rtnFw->mode     = 0; // Begin in Shell Mode
    rtnFw->sprkShdr = 0; // TBD
    rtnFw->xpldShdr = 0; // TBD
    /// GPU-Side ///
    allocate_and_load_VAO_VNC_at_GPU( rtnFw->shell );
    // FIXME, START HERE: CREATE GPU BUFFERS FOR THE PARTICLES
    return rtnFw;
}