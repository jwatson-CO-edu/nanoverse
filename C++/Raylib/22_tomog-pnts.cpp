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
    ifstream npy;
    string   hdr;
    ubyte    byt;
    ushort   dBt;
    string   dct;
    ubyte    dim = 100;
    vvvf     rtnCube;
    vvf /**/ slice;
    vf /*-*/ row;
    float    val;
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
                    row.push_back( val );
                }
                slice.push_back( row );
            }
            rtnCube.push_back( slice );
        }
        cout << "Read a data cube of size " << rtnCube.size() << " x " << rtnCube[0].size() << " x " << rtnCube[0][0].size() << endl;

    }else{  cout << "404: " << fName << " NOT found!" << endl;  }
    return rtnCube;
}


// FIXME, START HERE: DISPLAY RAW DATA AS MONOCHROME POINTS ON 3D GRID, MAGNITUDE DETERMINES SIZE
 


////////// MAIN ////////////////////////////////////////////////////////////////////////////////////

int main(){

    vvvf data = read_NPY_tomog( "data/model.npy" );

    return 0;
}