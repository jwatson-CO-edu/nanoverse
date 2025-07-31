#include "../../include/math_glm.hpp"
#include "../../include/toolbox.hpp"
#include "../../include/PUNCH.hpp"

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


vstr split_string_on_char( string input, char ch ){
    // Return a vector of strings found in `input` separated by whitespace
    vstr   rtnWords;
    string currWord;
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


vstr list_files_at_path( string path, bool sortAlpha ){
    // List all the files found at a path
    vstr   rtnNams;
    string path_i;
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


vstr list_files_at_path_w_ext( string path, string ext, bool sortAlpha ){
    vstr allPaths = list_files_at_path( path, sortAlpha );
    vstr rtnPaths;
    for( string fPath : allPaths ){
        if( file_has_ext( fPath, ext ) )
            rtnPaths.push_back( string( fPath ) );
    }
    return rtnPaths;
}





////////// FITS ////////////////////////////////////////////////////////////////////////////////////

void FITS_File::setup_null_pixel_values(){
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

size_t FITS_File::get_elem_size(){
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

void FITS_File::report_status( string prefix ){  
    /* print any error messages */
    if( verbose ){
        if( prefix.size() == 0 ){  cout << "Status: ";  }else{  cout << prefix << ": ";  }
        if( status ){  fits_report_error( stderr, status );  }else{ cout << "No error!";  }
        cout << endl;
    }
} 

void FITS_File::get_HDU_value( string key, int datatype_, void *value ){
    // Fetch a Header Data Unit value 
    char* comment = nullptr;
    fits_read_key( fptr, datatype_, key.c_str(),
                    value, comment, &status);
    if( verbose && (comment != nullptr) ){  cout << key << ", Comment: " << comment << endl;  } 
}

double FITS_File::float_HDU_as_double( string key ){
    float fltVal;
    get_HDU_value( key, TFLOAT, &fltVal );
    if( verbose ){  cout << "For \"" << key << "\", got " << fltVal << " --> " << (double) fltVal << endl;  }
    return (double) fltVal;
}

float FITS_File::float_HDU( string key ){
    // Fetch a Header Data Unit float and return as double
    float fltVal;
    get_HDU_value( key, TFLOAT, &fltVal );
    if( verbose ){  cout << "For \"" << key << "\", got " << fltVal << " --> " << (double) fltVal << endl;  }
    return fltVal;
}

FITS_File::FITS_File( string path_ ){
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

FITS_File::~FITS_File(){  
    // Free all dyn mem
    close(); // Frees the pointer
    if( dataArr ){  free( dataArr );  }
    if( axisDim ){  free( axisDim );  }
    for( char* card : cards ){  if( card ){  free( card );  }  }
}

void FITS_File::display_keys(){
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
long FITS_File::get_dim( int index ){  if( index < Naxes ) return (long) axisDim[ index ]; else return 0;  }

void FITS_File::close(){
    // Close the file
    fits_close_file( fptr, &status );
    report_status( "FILE CLOSE" );
}

bool FITS_File::malloc_arr( size_t Nelems ){
    // Make space for a bulk pixel fetch
    if( dataArr ){  free( dataArr );  }
    dataArr = malloc( get_elem_size() * Nelems );
    return (bool) dataArr;
}

void* FITS_File::fetch_pixel_data( addr coords, long nelements ){
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

void* FITS_File::fetch_row( long row ){
    // Get data from an entire row
    if( (row > -1) && (row < get_dim(0)) ){
        return fetch_pixel_data( {row, 0}, (long) get_dim(1) );
    }else{  return nullptr;  }
}


Corona_Data_FITS::Corona_Data_FITS( string fitsPath ){
    // Load all params relevent to 3D CME reconstruction
    dataFileFITS = FitsPtr{ new FITS_File{ fitsPath } };    
    xRefPxlVal   = dataFileFITS->float_HDU( "CRVAL1" );    
    yRefPxlVal   = dataFileFITS->float_HDU( "CRVAL2" );    
    xRadPerPxl   = dataFileFITS->float_HDU( "CDELT1" );    
    yRadPerPxl   = dataFileFITS->float_HDU( "CDELT2" );    
    dSun /*---*/ = dataFileFITS->float_HDU( "OBS_R0" );    
    img /*----*/ = Mat( cv::Size( (int) dataFileFITS->get_dim(0), (int) dataFileFITS->get_dim(1) ), CV_32F, cv::Scalar(0.0) );
    shw /*----*/ = Mat( cv::Size( (int) dataFileFITS->get_dim(0), (int) dataFileFITS->get_dim(1) ), CV_8U , cv::Scalar(0)   );
    xDim   = dataFileFITS->get_dim(0);
    yDim   = dataFileFITS->get_dim(1);
    float* row_i = nullptr;
    
    float span   =  0.0f;
    for( long i = 0; i < xDim; i++ ){
        row_i = (float*) dataFileFITS->fetch_row(i);
        for( long j = 0; j < yDim; j++ ){  
            img.at<float>(i,j) = row_i[j]; 
            dataFileFITS->valMin = min( row_i[j], dataFileFITS->valMin );
            dataFileFITS->valMax = max( row_i[j], dataFileFITS->valMax );
        }
    }
    /// Debug Info! ///
    cout << "Value Range: [" << dataFileFITS->valMin << ", " << dataFileFITS->valMax << "]" << endl;
    span = dataFileFITS->valMax - dataFileFITS->valMin;
    for( long i = 0; i < xDim; i++ ){
        for( long j = 0; j < yDim; j++ ){  
            shw.at<unsigned char>(i,j) = static_cast<unsigned char>((img.at<float>(i,j)-dataFileFITS->valMin)/span*255.0);
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



////////// CORONAL MASS EJECTION 3D MAPPING ////////////////////////////////////////////////////////

SolnPair::SolnPair( int width, int height, int depth ){
    int sizes[] = {width, height, depth}; // Example: 3 "planes", each 4 rows x 5 columns
    zetaPlus  = Mat( 3, sizes, CV_32F, cv::Scalar(0.0) );
    zetaMinus = Mat( 3, sizes, CV_32F, cv::Scalar(0.0) );
    cout << zetaMinus.size[0] << " x " << zetaMinus.size[1] << " || " << zetaMinus.size[0] << " x " << zetaMinus.size[1] << endl;
}

SolnPair calc_coords( string totalPath, string polarPath, float angleFromSolarNorth_rad, float cutoffFrac ){
    // Calculate 3D coordinates from polarized data
    Corona_Data_FITS tB_data{ totalPath };
    Corona_Data_FITS pB_data{ polarPath };
    // Reconstruction params
    long  xDim /*---*/ = tB_data.dataFileFITS->get_dim(0);
    long  yDim /*---*/ = tB_data.dataFileFITS->get_dim(1);
    long  hDim /*---*/ = yDim / 2;
    float pxFromCenter = 0.0f;
    float epsilon /**/ = 0.0f;
    float pB_over_tB   = 0.0;
    float bRatio /*-*/ = 0.0f;
    float angOffset    = tB_data.xRefPxlVal;
    float radPerPxl    = tB_data.xRadPerPxl;
    float zetaPlus     = 0.0f;
    float zetaMinus    = 0.0f;
    float Op_sky /*-*/ = 0.0f;
    float Ad_oos /*-*/ = 0.0f;
    float dThom /*--*/ = 0.0f;
    float dPxlPlus     = 0.0f;
    float dPxlMinus    = 0.0f;
    float tThresh /**/ = tB_data.dataFileFITS->valMin + (tB_data.dataFileFITS->valMax - tB_data.dataFileFITS->valMin) * cutoffFrac;
    // Locations
    vec4f ertLoc{ 0.0f, 0.0f, 0.0f /*--*/ , 1.0f }; // Place the Earth at the origin
    vec4f sunLoc{ 0.0f, 0.0f, tB_data.dSun, 1.0f }; // Place the Sun along the Z-axis
    vec4f posnPlus;
    vec4f posnMinus;
    // Rotation matrices
    mat4f xRot;
    mat4f yRot;
    mat4f zRot = R_z( angleFromSolarNorth_rad );
    // Return Struct
    SolnPair rtnPair{ (int) xDim, (int) yDim, 3 };

    // Perform 
    for( long i = 0; i < xDim; i++ ){
        // Get Y rotation of the column
        yRot = R_y( 1.0f * i * radPerPxl + angOffset );
        for( long j = 0; j < yDim; j++ ){  
            // Skip pixels w v small values
            if( tB_data.img.at<float>(i,j) < tThresh ){  continue;  } 
            // Get angle to object and distance to Thompson Surface
            pB_over_tB   = pB_data.img.at<float>(i,j) / tB_data.img.at<float>(i,j);
            pxFromCenter = sqrt( pow( abs( i-hDim ), 2.0f ) + pow( abs( j-hDim ), 2.0f ) );
            epsilon /**/ = pxFromCenter * radPerPxl;
            dThom /*--*/ = tB_data.dSun * cos( epsilon );
            // Get possible Out-of-Sky angles: zeta
            bRatio    = sqrt( (1.0f - pB_over_tB) / (1.0f + pB_over_tB) );
            zetaPlus  = epsilon + asin(  bRatio );
            zetaMinus = epsilon + asin( -bRatio );
            // cout << zetaPlus << " // " << zetaMinus << endl;
            // Get possible distances from sensor
            Op_sky    = tB_data.dSun * tan( epsilon );
            Ad_oos    = Op_sky * cos( epsilon );
            // cout << epsilon << " // " << tB_data.dSun << " // " << dThom << " // " << Op_sky << " // " << Ad_oos << endl;
            dPxlPlus  = dThom + Ad_oos * tan( zetaPlus  - epsilon );
            dPxlMinus = dThom + Ad_oos * tan( zetaMinus - epsilon );
            // cout << dPxlPlus << " // " << dPxlMinus << endl;
            // Get X rotation of the row
            xRot = R_x( 1.0f * j * radPerPxl + angOffset );
            // Get relative position of Near Solution
            posnPlus = vec4f{ 0.0, 0.0, dPxlPlus, 1.0 };
            posnPlus = zRot * xRot * yRot * posnPlus;
            // Get relative position of Far Solution
            posnMinus = vec4f{ 0.0, 0.0, dPxlMinus, 1.0 };
            posnMinus = zRot * xRot * yRot * posnMinus;
            cout << i << " // " << j << ", " << 1.0f * i * radPerPxl + angOffset << " // " << 1.0f * j * radPerPxl + angOffset << ", " << posnPlus << " // " << posnMinus << endl;
            // Load positions
            for( long k = 0; k < 3; ++k ){
                rtnPair.zetaPlus.at<float>(i,j,k)  = posnPlus[k];
                rtnPair.zetaMinus.at<float>(i,j,k) = posnMinus[k];
            }
        }
    }
    return rtnPair;
}