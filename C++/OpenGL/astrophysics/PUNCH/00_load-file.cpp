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

typedef vector<string> vstr;
typedef array<long,2>  addr;

void load_arr( addr coords, long* arr ){
    for( size_t i = 0; i < coords.size(); ++i ){  arr[i] = coords[i];  }
}

class FITS_File { public:
    /// Data ///
    fitsfile* /*-----------*/ fptr    = nullptr;
    void* /*---------------*/ dataArr = nullptr;
    vector<char[ FLEN_CARD ]> cards;
    int /*-----------------*/ status; 
    int /*-----------------*/ nkeys;
    string /*--------------*/ path;
    vstr /*----------------*/ keys;
    int /*-----------------*/ datatype;
    int /*-----------------*/ Naxes;
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
    
    void report_status( string prefix = "" ){  
        /* print any error messages */
        if( prefix.size() == 0 ){  cout << "Status: ";  }else{  cout << prefix << ": ";  }
        if( status ){  fits_report_error( stderr, status );  }  
        cout << endl;
    } 

    FITS_File( string path_ ){
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
        for( int ii = 1; ii <= nkeys; ii++ ){ 
            fits_read_record( fptr, ii, cards[ii-1], &status ); /* read keyword */
            keys[ii-1] = string( cards[ii-1] );
            printf( "key %i: %s\n", ii-1, cards[ii-1] );
        }
    }

    void close(){
        fits_close_file( fptr, &status );
        report_status( "FILE CLOSE" );
    }

    void fetch_pixel_data( addr coords, long nelements ){
        // FIXME: https://heasarc.gsfc.nasa.gov/docs/software/fitsio/quick/node9.html
        long pxlAdr[2];
        load_arr( coords, pxlAdr );
        fits_read_pix( fptr, datatype, pxlAdr, 
                       nelements, nullPxlVal[ datatype ], 
                       void *array, 
                       int *anynul, int *status)
    }
};

int main( int argc, char *argv[] ){


    
}