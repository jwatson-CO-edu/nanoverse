// g++ -I/opt/fits/include -I/usr/local/include/opencv4 01_load-punch-params.cpp `pkg-config --cflags --libs opencv4` -L/opt/fits/lib -lcfitsio -lm

////////// INIT ////////////////////////////////////////////////////////////////////////////////////

///// Imports /////////////////////////////////////////////////////////////
/// Standard ///
#include <string>
using std::string, std::to_string;
#include <vector>
using std::vector;
#include <array>
using std::array;
#include <map>
using std::map;
#include <iostream>
using std::cout, std::endl;
#include <memory>
using std::shared_ptr;
#include <algorithm>
using std::min, std::max;

/// CFITSIO ///
#include <fitsio.h>
/// OpenCV ///
#include <opencv2/opencv.hpp>
using cv::Mat;

///// Defines /////////////////////////////////////////////////////////////
#define PUNCH_X_DIM 1024
#define PUNCH_Y_DIM 1024

///// Aliases /////////////////////////////////////////////////////////////
typedef vector<string> vstr;
typedef array<long,2>  addr;

template<typename T, size_t S>
std::ostream& operator<<( std::ostream& os , array<T,S> arr ){ 
	os << "[";
    for( size_t i = 0; i < S; ++i ){
        os << arr[i];
        if( i < (S-1) ){  os << ", "; }else{  os << ",";  }
    }
    os << "]";
	return os;
}


////////// HELPER FUNCTIONS ////////////////////////////////////////////////////////////////////////

void load_arr( addr coords, long* arr ){
    // Load a `std::array` into a C array
    arr[0] = coords[1];
    arr[1] = coords[0];
    // cout << "Loaded arr!: " << coords << endl;
}

double* alloc_dbbl_arr( size_t N ){
    double* rtnPtr = nullptr;
    rtnPtr = (double*) malloc( sizeof( double ) * N );
    return rtnPtr;
}

////////// FITS ////////////////////////////////////////////////////////////////////////////////////

class FITS_File { public:
    // Fetch and manage FITS data in a sane manner

    /// Data ///
    fitsfile*     fptr    = nullptr;
    void* /*---*/ dataArr = nullptr;
    int* /*---*/ axisDim = nullptr;
    long* pxlAdr;
    vector<char*> cards;
    int /*-----*/ status; 
    int /*-----*/ nkeys;
    string /*--*/ path;
    int /*-----*/ datatype;
    int /*-----*/ Naxes;
    bool /*----*/ verbose = true;
    
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
        if( verbose ){
            if( prefix.size() == 0 ){  cout << "Status: ";  }else{  cout << prefix << ": ";  }
            if( status ){  fits_report_error( stderr, status );  }else{ cout << "No error!";  }
            cout << endl;
        }
    } 


    void get_HDU_value( string key, int datatype_, void *value ){
        // Fetch a Header Data Unit value 
        char* comment = nullptr;
        fits_read_key( fptr, datatype_, key.c_str(),
                       value, comment, &status);
        if( verbose && (comment != nullptr) ){  cout << key << ", Comment: " << comment << endl;  } 
    }

    
    double float_HDU_as_double( string key ){
        // Fetch a Header Data Unit float and return as double
        float fltVal;
        get_HDU_value( key, TFLOAT, &fltVal );
        if( verbose ){  cout << "For \"" << key << "\", got " << fltVal << " --> " << (double) fltVal << endl;  }
        return (double) fltVal;
    }


    FITS_File( string path_ ){
        // Open FITS at path and init params
        status  = 0; /* MUST initialize status */
        path    = path_;
        pxlAdr  = (long*) malloc( sizeof( long ) * 2 );
        fits_open_file( &fptr, path.c_str(), READONLY, &status );
        fits_get_hdrspace( fptr, &nkeys, NULL, &status );
        cards.reserve( nkeys );
        fits_get_img_type( fptr, &datatype, &status );
        switch( datatype ){
            case -32:
                datatype = TFLOAT;
                break;
            default:
                // datatype = TBYTE;
                break;
        }
        cout << "Data Type: " << datatype << endl;
        report_status( "Image Type" );
        fits_get_img_dim( fptr, &Naxes, &status );
        if( verbose ){  
            display_keys();  
            cout << "There are " << Naxes << " axes." << endl;
        }
        axisDim = (int*) malloc( sizeof( long ) * Naxes );
        for( int i = 0; i < Naxes; ++i ){
            string key = "NAXIS" + to_string(i+1);
            get_HDU_value( key, TINT, &axisDim[i] );
            if( verbose ){  cout << key << " = " << axisDim[i] << ", ";  }
        }
        cout << endl;
        report_status( "Image Dims" );
    }


    ~FITS_File(){  
        // Free all dyn mem
        close(); // Frees the pointer
        if( dataArr ){  free( dataArr );  }
        if( axisDim ){  free( axisDim );  }
        for( char* card : cards ){  if( card ){  free( card );  }  }
    }


    void display_keys(){
        // Load the string cards
        for( int ii = 0; ii < nkeys; ii++ ){ 
            cards[ii] = (char*) malloc( sizeof( char ) * FLEN_CARD );
            if( cards[ii] == NULL ){  break;  }
            fits_read_record( fptr, ii, cards[ii], &status ); /* read keyword */
            printf( "key %i: %s\n", ii, cards[ii] );
        }
        report_status( "Obtained HDU Keys" );
    }


    // Get the extent of the dimension
    long get_dim( int index ){  if( index < Naxes ) return (long) axisDim[ index ]; else return 0;  }


    void close(){
        // Close the file
        fits_close_file( fptr, &status );
        report_status( "FILE CLOSE" );
    }

    
    bool malloc_arr( size_t Nelems ){
        // Make space for a bulk pixel fetch
        if( dataArr ){  free( dataArr );  }
        dataArr = malloc( get_elem_size() * Nelems );
        return (bool) dataArr;
    }

    
    void* fetch_pixel_data( addr coords, long nelements ){
        // Wrapper to fetch pixel data from the FITS, return a pointer to the array
        int p_hasNull = 0;
        load_arr( coords, pxlAdr );
        if( !malloc_arr( nelements ) ){  return nullptr;  }
        fits_read_pix( fptr, datatype, pxlAdr, 
                       nelements, nullPxlVal[ datatype ], 
                       dataArr, 
                       &p_hasNull, &status);
        if( p_hasNull ){  cout << "Some pixels are NULL!" << endl;  }
        // free( pxlAdr );
        return dataArr;
    }

    void* fetch_row( long row ){
        // Get data from an entire row
        if( (row > 0) && (row <= get_dim(0)) ){
            return fetch_pixel_data( {row, 1}, (long) get_dim(1) );
        }else{  return nullptr;  }
    }
};
typedef shared_ptr<FITS_File> FitsPtr;



////////// CUSTOM DATA FRAME ///////////////////////////////////////////////////////////////////////

class Corona_Data_FITS { public:
    // Container for 3D CME reconstuction data and params
    FitsPtr dataFileFITS = nullptr;
    double  xRefPxlVal;
    double  yRefPxlVal;
    double  xRadPerPxl;
    double  yRadPerPxl;
    double  dToSun;
    double  radSun;
    double  obsLat;
    double  obsLng;
    Mat     img;
    Mat     shw;

    Corona_Data_FITS( string fitsPath ){
        // Load all params relevent to 3D CME reconstruction
        dataFileFITS  = FitsPtr{ new FITS_File{ fitsPath } };    
        xRefPxlVal    = dataFileFITS->float_HDU_as_double( "CRVAL1" );    
        yRefPxlVal    = dataFileFITS->float_HDU_as_double( "CRVAL2" );    
        xRadPerPxl    = dataFileFITS->float_HDU_as_double( "CDELT1" );    
        yRadPerPxl    = dataFileFITS->float_HDU_as_double( "CDELT2" );    
        cout << (int) dataFileFITS->get_dim(0) << ", " << (int) dataFileFITS->get_dim(1) << endl;
        img /*-----*/ = Mat( cv::Size( (int) dataFileFITS->get_dim(0), (int) dataFileFITS->get_dim(1) ), CV_32F, cv::Scalar(0.0) );
        shw /*-----*/ = Mat( cv::Size( (int) dataFileFITS->get_dim(0), (int) dataFileFITS->get_dim(1) ), CV_8U , cv::Scalar(0) );
        long  xDim  = dataFileFITS->get_dim(0);
        long  yDim  = dataFileFITS->get_dim(1);
        float* row_i = nullptr;
        float valMin =  1e9f;
        float valMax = -1e9f;
        float span;
        for( long i = 1; i <= xDim; i++ ){
            row_i = (float*) dataFileFITS->fetch_row(i);
            // cout << row_i << endl;
            for( long j = 1; j <= yDim; j++ ){  
                // cout << "[" << i << ", " << j << "]: ";
                img.at<float>(i-1,j-1) = row_i[j-1]; 
                valMin = min( row_i[j-1], valMin );
                valMax = max( row_i[j-1], valMax );
            }
        }
        cout << "Value Range: [" << valMin << ", " << valMax << "]" << endl;
        span = valMax - valMin;
        for( long i = 1; i <= xDim; i++ ){
            for( long j = 1; j <= yDim; j++ ){  
                shw.at<unsigned char>(i-1,j-1) = static_cast<unsigned char>((img.at<float>(i-1,j-1)-valMin)/span*255.0);
                // cout << shw.at<unsigned char>(i-1,j-1) << ", ";
            }
        }
        cv::imwrite( "TestImg.png", shw );
    }
};



////////// MAIN ////////////////////////////////////////////////////////////////////////////////////
int main( int argc, char *argv[] ){
    string    fPath = "../data/cme0_dcmer_0000_bang_0000_pB/stepnum_005.fits";
    // FITS_File fFile{ fPath };
    Corona_Data_FITS fFile{ fPath };
}