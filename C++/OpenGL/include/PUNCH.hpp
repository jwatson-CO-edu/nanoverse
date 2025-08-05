#ifndef PUNCH_HPP // This pattern is to prevent symbols to be loaded multiple times
#define PUNCH_HPP // from multiple imports

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
#include <filesystem>
using std::filesystem::directory_iterator;
#include <cmath>
using std::asin, std::pow, std::sqrt;

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
typedef vector<vstr>   vvstr;
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



////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////// PUNCH.cpp //////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////

////////// HELPER FUNCTIONS ////////////////////////////////////////////////////////////////////////
void    load_arr( addr coords, long* arr );
double* alloc_dbbl_arr( size_t N );
vstr    split_string_on_char( string input, char ch );
vstr    list_files_at_path( string path, bool sortAlpha = true );
string  to_upper( string input );
bool    file_has_ext( string path, string ext );
vstr    list_files_at_path_w_ext( string path, string ext, bool sortAlpha = true );
mat4f   R_x( float theta );
mat4f   R_y( float theta );
mat4f   R_z( float theta );

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
    float /*---*/ valMin  =  1e9f;
    float /*---*/ valMax  = -1e9f;
    
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

    void   setup_null_pixel_values(); // ----------------------------- Set a null value for each datatype
    size_t get_elem_size(); // --------------------------------------- Get the size in bytes of each element
    void   report_status( string prefix = "" ); // ------------------- Print any error messages
    void   get_HDU_value( string key, int datatype_, void *value ); // Fetch a Header Data Unit value 
    float  float_HDU( string key ); // ------------------------------- Fetch a Header Data Unit float and return as float
    double float_HDU_as_double( string key ); // --------------------- Fetch an HDU and cast to double value 

    FITS_File( string path_ ); // Open FITS at path and init params
    ~FITS_File(); // ------------ Free all dyn mem

    void display_keys(); // ------------ Load the string cards
    long get_dim( int index ); // ------ Get the extent of the dimension
    void close(); // ------------------- Close the file
    bool malloc_arr( size_t Nelems ); // Make space for a bulk pixel fetch
    // Wrapper to fetch pixel data from the FITS, return a pointer to the array
    void* fetch_pixel_data( addr coords, long nelements ); 
    void* fetch_row( long row ); // Get data from an entire row
};
typedef shared_ptr<FITS_File> FitsPtr;



////////// CUSTOM DATA FRAME ///////////////////////////////////////////////////////////////////////

class Corona_Data_FITS { public:
    // Container for 3D CME reconstuction data and params
    FitsPtr dataFileFITS = nullptr; // Pointer to the FITS wrapper
    long  xDim; // ----------------- Number of columns
    long  yDim; // ----------------- Number of rows
    float xRefPxlVal; // ----------- Reference Pixel Value, X
    float yRefPxlVal; // ----------- Reference Pixel Value, Y
    float xRadPerPxl; // ----------- Radians per Pixel, X
    float yRadPerPxl; // ----------- Radians per Pixel, Y
    float dToSun; // --------------- Distance to Sun
    float radSun; // --------------- Radius of Sun
    float obsLat; // --------------- Observer Latitude
    float obsLng; // --------------- Observer Longitude
    float dSun; // ----------------- Distance to the sun [Solar Radii]
    float zRot; // ----------------- Image rotation [rad]
    
    Mat   img; // ------------------ Unpacked Image Data
    Mat   shw; // ------------------ Debug Image Data (Greyscale, Offset & Scaled)

    Corona_Data_FITS( string fitsPath ); // Load all params relevent to 3D CME reconstruction
};
typedef shared_ptr<Corona_Data_FITS> CMEPtr;



////////// CORONAL MASS EJECTION 3D MAPPING ////////////////////////////////////////////////////////

class SolnPair{ public:
    // Container for near and far solutions for each pixel
    Mat /*---*/ zetaPlus;
    Mat /*---*/ zetaMinus;
    list<vec4f> pntsPlus;
    list<vec4f> pntsMinus;

    SolnPair( int width, int height, int depth );
};
typedef shared_ptr<SolnPair> PairPtr;

// Calculate 3D coordinates from polarized data
SolnPair calc_coords( string totalPath, string polarPath, float cutoffFrac = 0.5f );
PairPtr  calc_coords_ptr( string totalPath, string polarPath, float cutoffFrac = 0.5f );


#endif