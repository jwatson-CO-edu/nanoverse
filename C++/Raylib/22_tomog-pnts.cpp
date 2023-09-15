// g++ 22_tomog-pnts.cpp -std=c++17 -lraylib -O3
// Building as an L-System


////////// INIT ////////////////////////////////////////////////////////////////////////////////////

///// Imports ////////////////////////////////////

/// Standard ///
#include <sys/stat.h>
#include <fstream>
using std::ifstream, std::ofstream;

/// Local ///
#include "rl_toybox.hpp"
#define RLIGHTS_IMPLEMENTATION
#include "rlights.h"

///// Aliases ////////////////////////////////////
typedef vector<vector<vector<float>>> vvvf;
typedef vector<float> /*-----------*/ vf;
typedef array<long,3> /*-----------*/ address;

////////// FILE OPERATIONS /////////////////////////////////////////////////////////////////////////

bool file_exists( const string& fName ){ 
    // Return true if the file exists , otherwise return false
    struct stat buf; 
    if( stat( fName.c_str() , &buf ) != -1 ){ return true; } else { return false; }
}

void fetch_next_ubyte( ifstream& file, ubyte* numVar ){
    // Fetch 1 byte from `buffer` and cast as an `int`
    if( !file.eof() && file.is_open() ){
        file.read( reinterpret_cast<char*>( numVar ), sizeof( ubyte ) );
    }
}

void fetch_next_ushort( ifstream& file, ushort* numVar ){
    // Fetch 1 byte from `buffer` and cast as an `int`
    if( !file.eof() && file.is_open() ){
        file.read( reinterpret_cast<char*>( numVar ), sizeof( ushort ) );
    }
}

void fetch_next_float( ifstream& file, float* numVar ){
    // Fetch 1 byte from `buffer` and cast as an `int`
    if( !file.eof() && file.is_open() ){
        file.read( reinterpret_cast<char*>( numVar ), sizeof( float ) );
    }
}

string fetch_ASCII_chars_as_string( ifstream& file, uint N ){
    // Read `N` ASCII chars from the file and return as a string
    string rtnStr;
    ubyte  byt;
    for( uint i = 0; i < N; ++i ){
        fetch_next_ubyte( file, &byt );
        rtnStr += ((char) byt );
    }
    return rtnStr;
}



////////// NUMPY ///////////////////////////////////////////////////////////////////////////////////

vvvf read_NPY_tomog( const string& fName ){
    // Read the contents of an NPY matrix created by the Numpy Python library
    // https://numpy.org/devdocs/reference/generated/numpy.lib.format.html
    // WARNING, 2023-09-13: This function is TAILOR-MADE for this application only and does NOT GENERALIZE to all NPY!
    ifstream npy; // --------- NPY file
    string   hdr; // --------- Magic string
    ubyte    byt; // --------- Byte value
    ushort   dBt; // --------- Short value
    string   dct; // --------- Array format
    ubyte    dim = 100; // --- Size of each dimension
    vvvf     rtnCube; // ----- Struct to return
    vvf /**/ slice; // ------- Inner struct
    vf /*-*/ row; // --------- Innermost struct
    float    val; // --------- Data point
    float    magMin =  1e6; // Min magnitude
    float    magMax = -1e6; // Max magnitude
    if( file_exists( fName ) ){
        npy = ifstream{ fName };
        // The first 6 bytes are a magic string: exactly \x93NUMPY
        hdr = fetch_ASCII_chars_as_string( npy, 6 );
        cout << "Header:    " << hdr << endl;
        cout << "Version:   ";
        // The next 1 byte is an unsigned byte: the major version number of the file format, e.g. \x01
        fetch_next_ubyte( npy, &byt ); 
        cout << ((uint) byt) << ".";
        // The next 1 byte is an unsigned byte: the minor version number of the file format, e.g. \x00
        fetch_next_ubyte( npy, &byt );
        cout << ((uint) byt) << endl;
        // The next 2 bytes form a little-endian unsigned short int: the length of the header data HEADER_LEN
        fetch_next_ushort( npy, &dBt );
        cout << "Dim Bytes: " << dBt << endl;
        // The next HEADER_LEN bytes form the header data describing the array’s format. 
        // It is an ASCII string which contains a Python literal expression of a dictionary.
        dct = fetch_ASCII_chars_as_string( npy, dBt );
        cout << "Dim Dict:  " << dct << endl; // {'descr': '<f4', 'fortran_order': False, 'shape': (100, 100, 100), } 
        // WARNING, 2023-09-13: Assume 100 x 100 x 100 of floats, Assume innermost dimension is contiguous
        // 6+4+118+100*100*100*4 = 4,000,128  -  An array of double would be 8MB but the file is not that big
        for( ubyte i = 0; i < dim; ++i ){
            slice.clear();
            for( ubyte j = 0; j < dim; ++j ){
                row.clear();
                for( ubyte k = 0; k < dim; ++k ){
                    fetch_next_float( npy, &val );
                    if( val < magMin )  magMin = val;
                    if( val > magMax )  magMax = val;
                    row.push_back( val );
                }
                slice.push_back( row );
            }
            rtnCube.push_back( slice );
        }
        cout << "Read a data cube of size " << rtnCube.size() << " x " << rtnCube[0].size() << " x " << rtnCube[0][0].size() << endl;
        cout << "Values are in range [" << magMin << ", " << magMax << "]" << endl;
    }else{  cout << "404: " << fName << " NOT found!" << endl;  }
    return rtnCube;
}
 
void render_tomog_cube( const vvvf& data, float magMin, float magMax, 
                        float scale, Color clrMin, Color clrMax ){
    // Render solar tomography data as a 3D array of dots, Value determines color and opacity
    // 2023-09-15: This rendering method produces an ugly moire, esp. w/ screencaps, How to fix?
    ubyte   dim    = 100; // -------------- Size of each dimension
    float   offset = dim * scale / 2.0f; // Places the center at <0,0,0>
    Vector3 nudge{ 0.0f, 0.0f, 0.05f }; //- Length of line segment producing a dot
    Vector3 cellCntr, dotBgn, dotEnd; // -- Cell coords in the world frame
    Color   cellClr; // ------------------- Voxel color
    float   cellVal, blendFrac; // -------- Voxel value and fraction along min-->max number line
    Color   ngtClr = BLUE; // ------------- Unused!
    // WARNING, 2023-09-13: Assume 100 x 100 x 100 of floats

    rlBegin( RL_LINES ); // Begin batch drawing

    // For each voxel in the datacube
    for( ubyte i = 0; i < dim; ++i ){
        for( ubyte j = 0; j < dim; ++j ){
            for( ubyte k = 0; k < dim; ++k ){
                
                // Fetch voxel datum
                cellVal = data[i][j][k];
                
                // If the voxel has a value within the given interval, then render it, Only draw sensible values!
                if( (cellVal >= magMin) && (cellVal <= magMax) ){
                    // Determine cell color && set, Assume higher values are hotter
                    blendFrac = (cellVal - magMin) / (magMax - magMin); // [0.0, 1.0] --> [lo, hi]
                    cellClr = {
                        (ubyte)(blendFrac*clrMax.r + (1.0f-blendFrac)*clrMin.r),
                        (ubyte)(blendFrac*clrMax.g + (1.0f-blendFrac)*clrMin.g),
                        (ubyte)(blendFrac*clrMax.b + (1.0f-blendFrac)*clrMin.b),
                        (ubyte)(blendFrac*clrMax.a + (1.0f-blendFrac)*clrMin.a)
                    };
                    rlColor4ub( cellClr.r, cellClr.g, cellClr.b, cellClr.a );

                    // Calc cell center && place dot
                    cellCntr = { i*scale-offset, j*scale-offset, -k*scale+offset }; // Is `data[0][0][0]` above or below the orbital plane?
                    dotBgn   = Vector3Add( cellCntr, nudge );
                    dotEnd   = Vector3Subtract( cellCntr, nudge );
                    rlVertex3f( dotBgn.x, dotBgn.y, dotBgn.z );
                    rlVertex3f( dotEnd.x, dotEnd.y, dotEnd.z );
                } // else don't draw!
            }
        }
    }
    rlEnd(); // End batch drawing
}

vector<address> get_spherical_window( float radius ){
    // Get a `vector` of the offset `address`es for a spherical window with `radius` (One unit = stride 1)
    vector<address> window;
    long /*------*/ halfLen = ceil( abs( radius ) + 1.0f );
    float /*-----*/ radSqr = radius * radius;
    // address /*---*/ elem;
    for( long i = -halfLen; i <= halfLen; ++i ){
        for( long j = -halfLen; j <= halfLen; ++j ){
            for( long k = -halfLen; k <= halfLen; ++k ){
                if( (1.0*i*i + 1.0*j*j + 1.0*k*k) <= radSqr ){  window.push_back( {i,j,k} );  }
            }
        }
    }
    return window;
}

float average_value_thresh_filter( const vvvf& data, const vector<address>& window, address addr, float thresh ){
    // Return the filtered value of `addr` within `data` given the `window` and the `thresh` value
    // 2023-09-15: Goal is to remove streaks without smoothing the data, Preserve structure where things are happening!

    // FIXME, START HERE: ACCEPT THE VALUE IF THE SPHERICAL NEIGHBORHOOD HAS AVERAGE THRESH VALUE OR GREATER, OTHERWISE ZERO
    long    dim = 100;
    float   Nps = 0.0;
    float   acc = 0.0f;
    address flt;
    bool    vld;

    for( address win : window ){
        flt = { addr[0]+win[0], addr[1]+win[1], addr[2]+win[2] };
        vld = (0 <= flt[0]) && (flt[0] < dim) &&
              (0 <= flt[1]) && (flt[1] < dim) &&
              (0 <= flt[2]) && (flt[2] < dim);
        if( vld ){  

            // FIXME: DO NOT ACCUM NEGATIVE VALUES!

            acc += data[ flt[0] ][ flt[1] ][ flt[2] ];  
            Nps += 1.0f;
        }
    }

    if( (acc/Nps) >= thresh )  return data[ addr[0] ][ addr[1] ][ addr[2] ];
    else /*----------------*/  return 0.0f;
}

////////// MAIN ////////////////////////////////////////////////////////////////////////////////////

int main(){

    ///// Raylib Init /////////////////////////////////////////////////////

    /// RNG Init ///
    rand_seed();

    /// Window Init ///
    InitWindow( 900, 900, "Solor Tomograph Test" );
    SetTargetFPS( 60 );

    ///// Create Objects //////////////////////////////////////////////////

    /// Create Camera ///
    Camera camera = Camera{
        Vector3{  12.0,  15.0,  15.0 }, // Position
        Vector3{   0.0,   0.0,   0.0 }, // Target
        Vector3{   0.0,   0.0,   1.0 }, // Up
        45.0, // ---------------------- FOV_y
        0 // -------------------------- Projection mode
    };

    vvvf data = read_NPY_tomog( "data/model.npy" );

    // FIXME: GET FILTERED DATACUBE

    Color clrMin = ORANGE;
    clrMin.a = 25;
    Color clrMax = YELLOW;

    ////////// RENDER LOOP /////////////////////////////////////////////////////////////////////////

    while( !WindowShouldClose() ){

        /// Begin Drawing ///
        BeginDrawing();
        BeginMode3D( camera );
        ClearBackground( BLACK );

        ///// DRAW LOOP ///////////////////////////////////////////////////

        // FIXME: DRAW FILTERED DATACUBE
        render_tomog_cube( data, 0.5f, 125.0f, 0.1f, clrMin, clrMax );

        ///// END DRAWING /////////////////////////////////////////////////

        /// End Drawing ///
        EndMode3D();
        EndDrawing();
    }

    return 0;
}