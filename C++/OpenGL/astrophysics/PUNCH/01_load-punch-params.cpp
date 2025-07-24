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
using std::min, std::max, std::pow, std::sqrt;
#include <filesystem>
using std::filesystem::directory_iterator;
#include <cmath>
using std::asin;

/// CFITSIO ///
#include <fitsio.h>
/// OpenCV ///
#include <opencv2/opencv.hpp>
using cv::Mat;

/// Graphics Language Math ////
#include <glm/glm.hpp>
using glm::vec2, glm::vec3, glm::vec4, glm::mat4;
#include <glm/gtc/matrix_transform.hpp>
using glm::rotate;

///// Defines /////////////////////////////////////////////////////////////
#define PUNCH_X_DIM 1024
#define PUNCH_Y_DIM 1024

///// Aliases /////////////////////////////////////////////////////////////
typedef vector<string> vstr;
typedef array<long,2>  addr;
typedef vec2 /*-----*/ vec2f;
typedef vec3 /*-----*/ vec3f;
typedef vec4 /*-----*/ vec4f;
typedef mat4 /*-----*/ mat4f;

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
    arr[0] = coords[1]+1; // FITS is 1-indexed!
    arr[1] = coords[0]+1; // FITS is column-major!
}


double* alloc_dbbl_arr( size_t N ){
    double* rtnPtr = nullptr;
    rtnPtr = (double*) malloc( sizeof( double ) * N );
    return rtnPtr;
}


vector<string> split_string_on_char( string input, char ch ){
    // Return a vector of strings found in `input` separated by whitespace
    vector<string> rtnWords;
    string /*---*/ currWord;
    char /*-----*/ currChar;
    input += ch; // Separator hack

    for( char& currChar : input ){
        if( currChar == ch ){
            if( currWord.length() > 0 )  rtnWords.push_back( currWord );
            currWord = "";
        }else{
            currWord += currChar;
        }
    }
    return rtnWords; 
}


vector<string> list_files_at_path( string path, bool sortAlpha = true ){
    // List all the files found at a path
    vector<string> rtnNams;
    string /*---*/ path_i;
    for (const auto & entry : directory_iterator( path ) ){  
        path_i = entry.path().string();
        rtnNams.push_back( path_i );  
    }
    if( sortAlpha )  std::sort( rtnNams.begin(),rtnNams.end() );
    return rtnNams;
}


string to_upper( string input ){
    // Return a version of the string that is upper case
    string output;
    for( char& c : input ){ output += toupper( c ); }
    return output;
}


bool file_has_ext( string path, string ext ){
    // Return true if a file has a 
    vector<string> parts = split_string_on_char( path, '.' );
    // cout << "There are " << parts.size() << " segments!" << endl;
    return (to_upper( parts[ parts.size()-1 ] ) == to_upper( ext ));
}


vector<string> list_files_at_path_w_ext( string path, string ext, bool sortAlpha = true ){
    vector<string> allPaths = list_files_at_path( path, sortAlpha );
    vector<string> rtnPaths;
    for( string fPath : allPaths ){
        if( file_has_ext( fPath, ext ) )
            rtnPaths.push_back( string( fPath ) );
    }
    return rtnPaths;
}


mat4f R_x( float theta ){
    mat4f rtnMtx = mat4f{ 1.0 };
    rotate( rtnMtx, theta, vec3f{ 1.0, 0.0, 0.0 } );
    return rtnMtx;
}


mat4f R_y( float theta ){
    mat4f rtnMtx = mat4f{ 1.0 };
    rotate( rtnMtx, theta, vec3f{ 0.0, 1.0, 0.0 } );
    return rtnMtx;
}


mat4f R_z( float theta ){
    mat4f rtnMtx = mat4f{ 1.0 };
    rotate( rtnMtx, theta, vec3f{ 0.0, 0.0, 1.0 } );
    return rtnMtx;
}



////////// FITS ////////////////////////////////////////////////////////////////////////////////////

class FITS_File { public:
    // Fetch and manage FITS data in a sane manner

    /// Data ///
    fitsfile*     fptr    = nullptr;
    void* /*---*/ dataArr = nullptr;
    int* /*----*/ axisDim = nullptr;
    long* /*---*/ pxlAdr;
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


    float float_HDU( string key ){
        // Fetch a Header Data Unit float and return as double
        float fltVal;
        get_HDU_value( key, TFLOAT, &fltVal );
        if( verbose ){  cout << "For \"" << key << "\", got " << fltVal << " --> " << (double) fltVal << endl;  }
        return fltVal;
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
        return dataArr;
    }

    void* fetch_row( long row ){
        // Get data from an entire row
        if( (row > -1) && (row < get_dim(0)) ){
            return fetch_pixel_data( {row, 0}, (long) get_dim(1) );
        }else{  return nullptr;  }
    }
};
typedef shared_ptr<FITS_File> FitsPtr;




////////// CUSTOM DATA FRAME ///////////////////////////////////////////////////////////////////////

class Corona_Data_FITS { public:
    // Container for 3D CME reconstuction data and params
    FitsPtr dataFileFITS = nullptr; // Pointer to the FITS wrapper
    float xRefPxlVal; // ----------- Reference Pixel Value, X
    float yRefPxlVal; // ----------- Reference Pixel Value, Y
    float xRadPerPxl; // ----------- Radians per Pixel, X
    float yRadPerPxl; // ----------- Radians per Pixel, Y
    float dToSun; // --------------- Distance to Sun
    float radSun; // --------------- Radius of Sun
    float obsLat; // --------------- Observer Latitude
    float obsLng; // --------------- Observer Longitude
    Mat   img; // ------------------ Unpacked Image Data
    Mat   shw; // ------------------ Debug Image Data (Greyscale, Offset & Scaled)

    Corona_Data_FITS( string fitsPath ){
        // Load all params relevent to 3D CME reconstruction
        dataFileFITS = FitsPtr{ new FITS_File{ fitsPath } };    
        xRefPxlVal   = dataFileFITS->float_HDU( "CRVAL1" );    
        yRefPxlVal   = dataFileFITS->float_HDU( "CRVAL2" );    
        xRadPerPxl   = dataFileFITS->float_HDU( "CDELT1" );    
        yRadPerPxl   = dataFileFITS->float_HDU( "CDELT2" );    
        img /*----*/ = Mat( cv::Size( (int) dataFileFITS->get_dim(0), (int) dataFileFITS->get_dim(1) ), CV_32F, cv::Scalar(0.0) );
        shw /*----*/ = Mat( cv::Size( (int) dataFileFITS->get_dim(0), (int) dataFileFITS->get_dim(1) ), CV_8U , cv::Scalar(0)   );
        long  xDim   = dataFileFITS->get_dim(0);
        long  yDim   = dataFileFITS->get_dim(1);
        float* row_i = nullptr;
        float valMin =  1e9f;
        float valMax = -1e9f;
        float span   =  0.0f;
        for( long i = 0; i < xDim; i++ ){
            row_i = (float*) dataFileFITS->fetch_row(i);
            for( long j = 0; j < yDim; j++ ){  
                img.at<float>(i,j) = row_i[j]; 
                valMin = min( row_i[j], valMin );
                valMax = max( row_i[j], valMax );
            }
        }
        /// Debug Info! ///
        cout << "Value Range: [" << valMin << ", " << valMax << "]" << endl;
        span = valMax - valMin;
        for( long i = 0; i < xDim; i++ ){
            for( long j = 0; j < yDim; j++ ){  
                shw.at<unsigned char>(i,j) = static_cast<unsigned char>((img.at<float>(i,j)-valMin)/span*255.0);
            }
        }
        vstr   pathParts = split_string_on_char( fitsPath    , '.' );
        /*--*/ pathParts = split_string_on_char( pathParts[0], '/' );
        string pngPath;   
        if( pathParts.size() >= 2 )
            pngPath = "output/" + pathParts[ pathParts.size()-2 ] + "+++" + pathParts[ pathParts.size()-1 ] + ".png";
        else if( pathParts.size() >= 1 )
            pngPath = "output/" + pathParts[ pathParts.size()-1 ] + ".png";
        else
            pngPath = "output/Test.png";
        cv::imwrite( pngPath, shw );
    }
};
typedef shared_ptr<Corona_Data_FITS> CMEPtr;


class SolnPair{ public:
    Mat zetaPlus;
    Mat zetaMinus;

    SolnPair( int width, int height, int depth ){
        zetaPlus  = Mat( {width, height, depth}, CV_32F, cv::Scalar(0.0) );
        zetaMinus = Mat( {width, height, depth}, CV_32F, cv::Scalar(0.0) );
    }
};


SolnPair calc_coords( string totalPath, string polarPath ){
    // Calculate 3D coordinates from polarized data
    Corona_Data_FITS tB_data{ totalPath };
    Corona_Data_FITS pB_data{ polarPath };
    long  xDim = tB_data.dataFileFITS->get_dim(0);
    long  yDim = tB_data.dataFileFITS->get_dim(1);
    long  hDim = yDim / 2;
    float pxFromCenter  = 0.0f;
    float epsilon = 0.0f;
    float pB_over_tB = 0.0;
    float bRatio /**/  = 0.0f;
    float dSun = tB_data.dataFileFITS->float_HDU( "OBS_R0" );  
    float angOffset = tB_data.xRefPxlVal;
    float radPerPxl = tB_data.xRadPerPxl;

    float zetaPlus  = 0.0f;
    float zetaMinus = 0.0f;
    float Op_sky /*---*/ = 0.0f;
    float Ad_oos /*---*/ = 0.0f;
    float dThom /*---*/ = 0.0f;
    float dPxlPlus /*---*/ = 0.0f;
    float dPxlMinus /*---*/ = 0.0f;
    float dTilt /*---*/ = 0.0f;

    vec4f ertLoc{ 0.0f, 0.0f, 0.0f, 1.0f }; // Place the Earth at the origin
    vec4f sunLoc{ 0.0f, 0.0f, dSun, 1.0f }; // Place the Sun along the Z-axis

    mat4f xRot;
    mat4f yRot;

    vec4f posnPlus;
    vec4f posnMinus;

    SolnPair rtnPair{ xDim, yDim, 3 };

    for( long i = 0; i < xDim; i++ ){
        for( long j = 0; j < yDim; j++ ){  
            pB_over_tB   = pB_data.img.at<float>(i,j) / tB_data.img.at<float>(i,j);
            pxFromCenter = sqrt( pow( abs( i-hDim ), 2.0f ) + pow( abs( j-hDim ), 2.0f ) );
            epsilon /**/ = pxFromCenter * radPerPxl;
            dThom = dSun * cos( epsilon );
            bRatio /*-*/ = sqrt( (1.0f - pB_over_tB) / (1.0f + pB_over_tB) );
            zetaPlus     = epsilon + asin(  bRatio );
            zetaMinus    = epsilon + asin( -bRatio );
            xRot /*---*/ = R_x( j * radPerPxl + angOffset );
            yRot /*---*/ = R_y( i * radPerPxl + angOffset );

            Op_sky = dSun   * tan( epsilon );
            Ad_oos = Op_sky * cos( epsilon );
            dPxlPlus  = dThom + Ad_oos * tan( zetaPlus  - epsilon );
            dPxlMinus = dThom + Ad_oos * tan( zetaMinus - epsilon );

            posnPlus = vec4f{ 0.0, 0.0, dPxlPlus, 1.0 };
            posnPlus = xRot * yRot * posnPlus;

            posnMinus = vec4f{ 0.0, 0.0, dPxlMinus, 1.0 };
            posnMinus = xRot * yRot * posnMinus;
        }
    }
}




////////// MAIN ////////////////////////////////////////////////////////////////////////////////////
int main( int argc, char *argv[] ){
    map<string,vector<CMEPtr>> sunPix;
    vstr dataDirs = {
        "../data/cme0_dcmer_0000_bang_0000_pB",
        "../data/cme0_dcmer_0000_bang_0000_tB",
        "../data/cme0_dcmer_030E_bang_0000_pB",
        "../data/cme0_dcmer_030E_bang_0000_tB",
        "../data/cme0_dcmer_060W_bang_0000_pB",
        "../data/cme0_dcmer_060W_bang_0000_tB",
        "../data/cme0_dcmer_090E_bang_0000_pB",
        "../data/cme0_dcmer_090E_bang_0000_tB"
    };
    vstr   filePaths;
    vstr   parts;
    string key;
    for( const string& fDir : dataDirs ){
        filePaths = list_files_at_path_w_ext( fDir, "fits" );
        parts     = split_string_on_char( fDir, '/' );
        key /*-*/ = parts[ parts.size()-1 ];
        sunPix[ key ] = vector<CMEPtr>{};
        for( const string& fPath : filePaths ){
            cout << "Open " << fPath << endl;
            sunPix[ key ].push_back( CMEPtr{ new Corona_Data_FITS{ fPath } } );
        }
    }
}