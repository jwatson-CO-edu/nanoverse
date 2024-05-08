////////// INIT ////////////////////////////////////////////////////////////////////////////////////
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
    }
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



////////// OPENGL SYSTEM ///////////////////////////////////////////////////////////////////////////

void ErrCheck( const char* where ){
    // Author: Willem A. Schreüder  
    int err = glGetError();
    if (err) fprintf(stderr,"ERROR: %s [%s]\n",gluErrorString(err),where);
}


void Fatal( const char* format , ... ){
    // Scream and Run
    // Author: Willem A. Schreüder  
    va_list args;
    va_start( args, format );
    vfprintf( stderr, format, args );
    va_end( args );
    exit(1);
}


void Project( double fov, double asp, double dim ){
    // Set the projection matrix for perspective
    // Adapted from code by Willem A. Schreüder  
    // 1. Tell OpenGL we want to manipulate the projection matrix
    glMatrixMode( GL_PROJECTION );
    // 2. Undo previous transformations
    glLoadIdentity();
    // 3. Perspective transformation
    gluPerspective( fov, asp, dim/16, 16*dim );
    // 4. Switch to manipulating the model matrix
    glMatrixMode( GL_MODELVIEW );
    // 5. Undo previous transformations
    glLoadIdentity();
}


void Print( const char* format, ... ){
    // Print raster letters to the screen
    // Author: Willem A. Schreüder  
    char    buf[LEN];
    char*   ch=buf;
    va_list args;
    //  Turn the parameters into a character string
    va_start( args, format );
    vsnprintf( buf, LEN, format, args );
    va_end( args );
    //  Display the characters one at a time at the current raster position
    while( *ch )  glutBitmapCharacter( GLUT_BITMAP_HELVETICA_18, *ch++ );
}



//
static void Reverse( void* x, const int n ){
    //  Reverse n bytes
    char* ch = (char*)x;
    for( int k = 0; k < n/2; k++ ){
        char tmp = ch[k];
        ch[k] = ch[n-1-k];
        ch[n-1-k] = tmp;
    }
}


uint LoadTexBMP( const char* file ){
    // Load texture from BMP file
    //  1. Open file
    FILE* f = fopen( file, "rb" );
    if( !f )  Fatal( "Cannot open file %s\n", file );
    //  2. Check image magic
    unsigned short magic;
    if( fread( &magic, 2, 1, f ) != 1 )  Fatal( "Cannot read magic from %s\n", file );
    if( magic != 0x4D42 && magic != 0x424D )  Fatal( "Image magic not BMP in %s\n", file );
    //  3. Read header
    unsigned int dx, dy, off, k; // Image dimensions, offset and compression
    unsigned short nbp, bpp;   // Planes and bits per pixel
    if( fseek( f, 8, SEEK_CUR )    || fread( &off, 4, 1, f ) != 1 ||
        fseek( f, 4, SEEK_CUR )    || fread( &dx , 4, 1, f ) != 1 || fread( &dy, 4, 1, f ) != 1 ||
        fread( &nbp, 2, 1, f) != 1 || fread( &bpp, 2, 1, f ) != 1 || fread( &k , 4, 1, f ) != 1 )
        Fatal( "Cannot read header from %s\n", file );
    //  4. Reverse bytes on big endian hardware (detected by backwards magic)
    if( magic == 0x424D ){
        Reverse( &off, 4 );
        Reverse( &dx , 4 );
        Reverse( &dy , 4 );
        Reverse( &nbp, 2 );
        Reverse( &bpp, 2 );
        Reverse( &k  , 4 );
    }
    //  5. Check image parameters
    unsigned int max;
    glGetIntegerv( GL_MAX_TEXTURE_SIZE, (int*) &max );
    if(dx<1 || dx>max) Fatal( "%s image width %d out of range 1-%d\n", file, dx, max );
    if(dy<1 || dy>max) Fatal( "%s image height %d out of range 1-%d\n", file, dy, max );
    if(nbp!=1) /*---*/ Fatal( "%s bit planes is not 1: %d\n", file, nbp );
    if(bpp!=24) /*--*/ Fatal( "%s bits per pixel is not 24: %d\n", file, bpp );
    if(k!=0) /*-----*/ Fatal( "%s compressed files not supported\n", file );
    //  6. Allocate image memory
    unsigned int size = 3*dx*dy;
    unsigned char* image = (unsigned char*) malloc( size );
    if( !image )  Fatal( "Cannot allocate %d bytes of memory for image %s\n", size, file );
    //  7. Seek to and read image
    if( fseek( f, off, SEEK_SET ) || fread( image, size, 1, f ) != 1 )  Fatal( "Error reading data from image %s\n", file );
    fclose( f );
    //  8. Reverse colors (BGR -> RGB)
    for( k = 0; k < size; k += 3 ){
        unsigned char temp = image[k];
        image[k]   = image[k+2];
        image[k+2] = temp;
    }
    //  9. Sanity check
    ErrCheck( "LoadTexBMP" );
    // 10. Generate 2D texture
    unsigned int texture;
    glGenTextures( 1, &texture );
    glBindTexture( GL_TEXTURE_2D, texture );
    // 11. Copy image
    glTexImage2D( GL_TEXTURE_2D, 0, GL_RGB, dx, dy, 0, GL_RGB, GL_UNSIGNED_BYTE, image );
    if( glGetError() )  Fatal( "Error in glTexImage2D %s %dx%d\n", file, dx, dy );
    // 12. Scale linearly when image size doesn't match
    glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR );
    glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR );
    // 13. Free image memory
    free( image );
    // N. Return texture name
    return texture;
}

////////// PRINTING HELPERS ////////////////////////////////////////////////////////////////////////
void nl( void ){  printf("\n");  } // Emit a newline to console