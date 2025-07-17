#include <string>
using std::string;
#include <vector>
using std::vector;
#include <array>
using std::array;
#include <map>
using std::map;
#include <iostream>
using std::cout, std::endl;
#include <fitsio.h>

#define FITS_CACHE_ROWS 100

typedef vector<string> vstr;
typedef array<long,2>  addr;

void load_arr( addr coords, long* arr ){
    // Load a `std::array` into a C array
    for( size_t i = 0; i < coords.size(); ++i ){  arr[i] = coords[i];  }
}

class FITS_File { public:
    // Fetch and manage FITS data in a sane manner

    /// Helper Structs ///
    enum CState { CNULL, READY, STALE };
    struct CEntry {
        CState state;
        void*  row;
    };
    
    /// Data ///
    fitsfile* /*---------------*/ fptr /*-*/ = nullptr;
    void* /*-------------------*/ dataArr    = nullptr;
    array<CEntry,FITS_CACHE_ROWS> cache /**/ = {CEntry{CNULL,nullptr}};
    vector<char[ FLEN_CARD ]>     cards;
    int /*---------------------*/ status; 
    int /*---------------------*/ nkeys;
    string /*------------------*/ path;
    vstr /*--------------------*/ keys;
    int /*---------------------*/ datatype;
    int /*---------------------*/ Naxes;
    
    /// Null Values ///
    map<int,void*>   nullPxlVal;
    unsigned char    TBYTE_NULL     = 0;     
    signed char /**/ TSBYTE_NULL    = 0;    
    signed short     TSHORT_NULL    = 0;    
    unsigned short   TUSHORT_NULL   = 0;   
    signed int /*-*/ TINT_NULL /**/ = 0;      
    unsigned int     TUINT_NULL     = 0;     
    signed long /**/ TLONG_NULL     = 0;     
    signed long long TLONGLONG_NULL = 0; 
    unsigned long    TULONG_NULL    = 0;    
    float /*------*/ TFLOAT_NULL    = 0.0f;    
    double /*-----*/ TDOUBLE_NULL   = 0.0;  
    

    void setup_null_pixel_values(){
        // Set a null value for each datatype
        nullPxlVal[ TBYTE     ] = &TBYTE_NULL;
        nullPxlVal[ TSBYTE    ] = &TSBYTE_NULL;
        nullPxlVal[ TSHORT    ] = &TSHORT_NULL;
        nullPxlVal[ TUSHORT   ] = &TUSHORT_NULL;
        nullPxlVal[ TINT      ] = &TINT_NULL;
        nullPxlVal[ TUINT     ] = &TUINT_NULL;
        nullPxlVal[ TLONG     ] = &TLONG_NULL;
        nullPxlVal[ TLONGLONG ] = &TLONGLONG_NULL;
        nullPxlVal[ TULONG    ] = &TULONG_NULL;
        nullPxlVal[ TFLOAT    ] = &TFLOAT_NULL;
        nullPxlVal[ TDOUBLE   ] = &TDOUBLE_NULL;
    }

    size_t get_elem_size(){
        // Get the size in bytes of each element
        switch( datatype ){
            case TBYTE:     return sizeof( unsigned char );
            case TSBYTE:    return sizeof( signed char );
            case TSHORT:    return sizeof( signed short );
            case TUSHORT:   return sizeof( unsigned short );
            case TINT: /**/ return sizeof( signed int );
            case TUINT:     return sizeof( unsigned int );
            case TLONG:     return sizeof( signed long );
            case TLONGLONG: return sizeof( signed long long );
            case TULONG:    return sizeof( unsigned long );
            case TFLOAT:    return sizeof( float );
            case TDOUBLE:   return sizeof( double );
            default: /*--*/ return 0;
        }
    }
    
    void report_status( string prefix = "" ){  
        /* print any error messages */
        if( prefix.size() == 0 ){  cout << "Status: ";  }else{  cout << prefix << ": ";  }
        if( status ){  fits_report_error( stderr, status );  }  
        cout << endl;
    } 

    FITS_File( string path_ ){
        // Open FITS at path and init params
        status = 0; /* MUST initialize status */
        path   = path_;
        fits_open_file( &fptr, path.c_str(), READONLY, &status );
        fits_get_hdrspace( fptr, &nkeys, NULL, &status );
        cards.reserve( nkeys );
        fits_get_img_type( fptr, &datatype, &status );
        report_status( "Image Type" );
        fits_get_img_dim( fptr, &Naxes, &status );
        report_status( "Image Dims" );
        load_keys();
    }

    void load_keys(){
        // Load the string cards
        for( int ii = 1; ii <= nkeys; ii++ ){ 
            fits_read_record( fptr, ii, cards[ii-1], &status ); /* read keyword */
            keys[ii-1] = string( cards[ii-1] );
            printf( "key %i: %s\n", ii-1, cards[ii-1] );
        }
    }

    void close(){
        // Close the file
        fits_close_file( fptr, &status );
        report_status( "FILE CLOSE" );
    }

    bool malloc_arr( size_t Nelems ){
        // Make space for a bulk pixel fetch
        dataArr = malloc( get_elem_size() * Nelems );
        return (dataArr != NULL);
    }

    void fetch_pixel_data( addr coords, long nelements ){
        // FIXME: https://heasarc.gsfc.nasa.gov/docs/software/fitsio/quick/node9.html
        int p_hasNull = 0;
        long pxlAdr[2];
        load_arr( coords, pxlAdr );
        fits_read_pix( fptr, datatype, pxlAdr, 
                       nelements, nullPxlVal[ datatype ], 
                       dataArr, 
                       &p_hasNull, &status);
        if( p_hasNull ){  cout << "Some pixels are NULL!" << endl;  }
    }
};

int main( int argc, char *argv[] ){


    
}