//  CSCIx239 library
//  Willem A. (Vlakkies) Schreuder
#include "toolbox.h"



////////// TRIGONOMETRY ////////////////////////////////////////////////////////////////////////////

// Cosine and Sine in degrees
// Author: Willem A. (Vlakkies) Schreüder  
double Cos( double x ){  return cos( (x) * 3.1415927 / 180 );  }
double Sin( double x ){  return sin( (x) * 3.1415927 / 180 );  }
double Tan( double x ){  return tan( (x) * 3.1415927 / 180 );  }
float  Cosf( float x ){  return cosf( (x) * 3.1415927f / 180.0f );  }
float  Sinf( float x ){  return sinf( (x) * 3.1415927f / 180.0f );  }
float  Tanf( float x ){  return tanf( (x) * 3.1415927f / 180.0f );  }
float  Atan2f( float y, float x ){  return (atan2f( y, x ) * 3.1415927f / 180.0f);  }



////////// MATH HELPERS ////////////////////////////////////////////////////////////////////////////
uint min_uint( uint x, uint y ){  return ((x) < (y)) ? (x) : (y);  } 
uint max_uint( uint x, uint y ){  return ((x) < (y)) ? (y) : (x);  } 


///// Random Numbers //////////////////////////////////////////////////////
void init_rand(){  srand( time( NULL ) );  }

float randf(){
    // Return a pseudo-random number between 0.0 and 1.0
    return  1.0f * rand() / RAND_MAX;
}

float randf_range( float lo, float hi ){
    // Return a pseudo-random number between `lo` and `hi`
    // NOTE: This function assumes `hi > lo`
    float span = hi - lo;
    return lo + span * randf();
}

int randi_range( int lo, int hi ){
    // Return a pseudo-random number between `lo` and `hi` (int)
    int span = hi - lo;
    return lo + (rand() % span);
}

uint randu_range( uint lo, uint hi ){
    // Return a pseudo-random number between `lo` and `hi` (uint)
    uint span = hi - lo;
    return lo + ((uint) rand() % span);
}

ubyte rand_ubyte(){
    // Return a pseudo-random unsigned byte
    return (ubyte) (randf()*256.0f);
}



////////// SIMULATION & TIMING /////////////////////////////////////////////////////////////////////

int sleep_ms( long msec ){
    // Pause main thread: implemented using nanosleep(), continuing the sleep if it is interrupted by a signal
    // Author: Neuron, https://stackoverflow.com/a/1157217
    struct timespec ts;
    int res;
    if( msec < 0 ){
        errno = EINVAL;
        return -1;
    }
    ts.tv_sec = msec / 1000;
    ts.tv_nsec = (msec % 1000) * 1000000;
    do{
        res = nanosleep( &ts, &ts );
    }while( res && errno == EINTR );
    return res;
}


float heartbeat_FPS( float targetFPS ){
    // Attempt to maintain framerate no greater than target. (Period is rounded down to next ms)
    float /*--*/ currTime  = 0.0f;
    float /*--*/ framTime  = 0.0f;
    float /*--*/ target_ms = 1000.0f / targetFPS;
    static float FPS /*-*/ = 0.0f;
    static float lastTime  = 0.0f;
    if( lastTime > 0.0f ){
        framTime = (float) glutGet( GLUT_ELAPSED_TIME ) - lastTime;
        if( framTime < target_ms ){ sleep_ms( (long) (target_ms - framTime) );  }
        currTime = (float) glutGet( GLUT_ELAPSED_TIME );
        FPS /**/ = (1000.0f / (currTime - lastTime)) * 0.125f + FPS * 0.875f; // Filter for readable number
        lastTime = currTime;
    }else{  lastTime = (float) glutGet( GLUT_ELAPSED_TIME );  }
    return FPS;
}



////////// CAMERA //////////////////////////////////////////////////////////////////////////////////

/// Methods ///
void look( const Camera3D camera ){
    // Set camera position, target, and orientation
    gluLookAt( (double) camera.eyeLoc.x, (double) camera.eyeLoc.y, (double) camera.eyeLoc.z,  
               (double) camera.lookPt.x, (double) camera.lookPt.y, (double) camera.lookPt.z,  
               (double) camera.upVctr.x, (double) camera.upVctr.y, (double) camera.upVctr.z );
}


void orbit_target_about_Z( Camera3D* camera, float delTheta_deg ){
    // Move the `camera` in a circular arc by `delTheta_deg` in the X-Y plane w/ `camera.lookPt` as the center
    vec4f vcDiff = sub_vec4f( camera->lookPt, camera->eyeLoc );
    float theta  = Atan2f( vcDiff.y, vcDiff.x ) + delTheta_deg;
    float radius = sqrtf( (vcDiff.y)*(vcDiff.y) + (vcDiff.x)*(vcDiff.x) );
    vcDiff.x = radius * Cosf( theta );
    vcDiff.y = radius * Sinf( theta );
    camera->eyeLoc = add_vec4f( camera->lookPt, vcDiff );
}



////////// OPENGL SYSTEM ///////////////////////////////////////////////////////////////////////////


void ErrCheck( const char* where ){
   // Check for OpenGL errors and print to stderr
   // Author: Willem A. (Vlakkies) Schreüder, https://www.prinmath.com/
   int err = glGetError();
   if (err) fprintf( stderr, "ERROR: %s [%s]\n", gluErrorString(err), where );
}


void Fatal( const char* format , ... ){
   // Print message to stderr and exit
   // Author: Willem A. (Vlakkies) Schreüder, https://www.prinmath.com/
   va_list args;
   va_start( args, format );
   vfprintf( stderr, format, args );
   va_end( args );
   exit(1);
}


void Print(const char* format , ...){
    // Print raster letters to the viewport
    // Author: Willem A. (Vlakkies) Schreüder, https://www.prinmath.com/
    char    buf[LEN];
    char*   ch = buf;
    va_list args;
    // 1. Turn the parameters into a character string
    va_start( args, format );
    vsnprintf( buf, LEN, format, args );
    va_end( args );
    // 2. Display the characters one at a time at the current raster position
    while( *ch ){  glutBitmapCharacter( GLUT_BITMAP_HELVETICA_18, *ch++ );  }
}


void Project( double fov, double asp, double dim ){
    // Set projection
    // Author: Willem A. (Vlakkies) Schreüder, https://www.prinmath.com/
    // 1. Tell OpenGL we want to manipulate the projection matrix
    glMatrixMode( GL_PROJECTION );
    // 2. Undo previous transformations
    glLoadIdentity();
    // 3. Perspective transformation
    if( fov )
        gluPerspective( fov, asp, dim/16, 16*dim );
    // 4. Orthogonal transformation
    else
        glOrtho( -asp*dim, asp*dim, -dim, +dim, -dim, +dim );
    //  Switch to manipulating the model matrix
    glMatrixMode( GL_MODELVIEW );
    //  Undo previous transformations
    glLoadIdentity();
}

////////// PRINTING HELPERS ////////////////////////////////////////////////////////////////////////
void nl( void ){  printf("\n");  } // Emit a newline to console