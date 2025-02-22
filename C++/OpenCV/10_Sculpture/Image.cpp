////////// INIT ////////////////////////////////////////////////////////////////////////////////////
#include "SfM.hpp"

////////// KAZE ////////////////////////////////////////////////////////////////////////////////////

KAZE::KAZE(){  akaze = AKAZE::create();  }

void KAZE::get_KAZE_keypoints( const Mat& img, vector<KeyPoint>& kptsOut, Mat& descOut ){
    akaze->detectAndCompute( img, cv::noArray(), kptsOut, descOut, false );
}

////////// STRING PROCESSING ///////////////////////////////////////////////////////////////////////

string to_upper( string input ){
    // Return a version of the string that is upper case
    string output;
    for( char& c : input ){ output += toupper( c ); }
    return output;
}

vector<string> split_string_on_char( string input, char ch ){
    // Return a vector of strings found in `input` separated by whitespace
    vector<string> rtnWords;
    string /*---*/ currWord;
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



////////// FILE OPERATIONS /////////////////////////////////////////////////////////////////////////

bool file_has_ext( string path, string ext ){
    // Return true if a `path` has `ext`
    vector<string> parts = split_string_on_char( path, '.' );
    // cout << "There are " << parts.size() << " segments!" << endl;
    return (to_upper( parts[ parts.size()-1 ] ) == to_upper( ext ));
}


vector<string> list_files_at_path( string path, bool sortAlpha ){
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


vector<string> list_files_at_path_w_ext( string path, string ext, bool sortAlpha ){
    // List all paths under `path` with the given `ext`
    vector<string> allPaths = list_files_at_path( path, sortAlpha );
    // cout << allPaths.size() << " candidate paths ..." << flush;
    vector<string> rtnPaths;
    for( string fPath : allPaths ){
        // cout << "Check: " << fPath << endl;
        if( file_has_ext( fPath, ext ) )
            rtnPaths.push_back( string( fPath ) );
    }
    return rtnPaths;
}


void fetch_images_at_path( string path, vector<string>& fNames, vector<Mat>& images, 
                           uint limit, string ext ){
    // Load all the images found at a path
    fNames.clear();
    images.clear();
    fNames = list_files_at_path_w_ext( path, ext, true );
    uint /*-----*/ Nimg = fNames.size();
    Mat /*------*/ img;  
    uint /*-----*/ i = 0;
    cout << "Found: " << Nimg << " files ... " << endl;
    for( string fName : fNames ){
        if( (limit > 0) && (i > (limit-1)) ) break;
        cout << "Found: " << fName << " ... " << flush;
        img = imread( fName, IMREAD_GRAYSCALE );
        images.push_back( img );
        cout << "Loaded: " << fName << ", Bytes: " << (img.total() * img.elemSize()) << endl;
        i++;
    }
    cout << endl << "Got " << images.size() << " images!" << endl;
}


bool file_exists( const string& fName ){ 
    // Return true if the file exists , otherwise return false
    struct stat buf; 
    if( stat( fName.c_str() , &buf ) != -1 ){ return true; } else { return false; }
}


vector<string> read_lines( string path ){ 
    // Return all the lines of text file as a string vector
    vector<string> rtnVec;
    if( file_exists( path ) ){
        ifstream fin( path ); // Open the list file for reading
        string   line; // String to store each line
        while ( std::getline( fin , line ) ){ // While we are able to fetch a line
            rtnVec.push_back( line ); // Add the file to the list to read
        }
        fin.close();
    } else { cout << "readlines: Could not open the file " << path << endl; }
    return rtnVec;
}


////////// UTILITY FUNCTIONS ///////////////////////////////////////////////////////////////////////

Mat deserialize_2d_Mat_d( string input, int Mrows, int Ncols, char sep ){
    // Deserialize an OpenCV `CV_64F` matrix stored row-major in a comma-separated list in a string
    Mat rtnMat = Mat::zeros( Mrows, Ncols, CV_64F );

    // cout << "Got input: " << input << endl;

    vector<string> tokens = split_string_on_char( input, sep );
    // cout << "Separated input: " << endl;
    // for( string& token : tokens ) cout << '\t' << token << endl;


    if( tokens.size() < ((size_t) Mrows*Ncols) )  return rtnMat; // Return the zero matrix if there are insufficient elements
    int k = 0;
    string item;
    for( int i = 0; i < Mrows; ++i ){
        for( int j = 0; j < Ncols; ++j ){
            // cout << endl << "\t\tAt (" << i << ',' << j << "), " << k << flush;
            // cout << ", " << tokens[k] << flush;
            // item = tokens[k];
            // item += '\0'
            // cout << ", " << stof( tokens[k] ) << flush;
            try{
                // rtnMat.at<double>(i,j) = stof( tokens[k] );
                rtnMat.at<double>(i,j) = stod( tokens[k] );
            }catch (const std::out_of_range& e) {
                cout << "Out of Range error." << endl;
                rtnMat.at<double>(i,j) = nanf("");
            }
            ++k;
        }
    }
    return rtnMat;
}


// Get everything after the last ':' in a string
string get_line_arg( string line ){  return get_last( split_string_on_char( line, ':' ) );  }


matXef OCV_matx_to_Eigen3_matx_f( const Mat& ocvMatx ){
    // Transfer an OpenCV matrix to Eigen3 matrix
    matXef rtnMtx = matXef::Zero( ocvMatx.rows, ocvMatx.cols ); // Eigen3: For matrices, the row index is always passed first. 
    for( size_t i = 0; i < ocvMatx.rows; ++i ){
        for( size_t j = 0; j < ocvMatx.cols; ++j ){
            rtnMtx( i, j ) = (float) ocvMatx.at<double>( i, j );
        }
    }
    return rtnMtx;
}



////////// OPERATORS ///////////////////////////////////////////////////////////////////////////////

///// Point Cloud Element Operators ///////////////////////////////////////


PntPos operator+( const PntPos& left, const PntPos& right ){
    // Add two PCL points
    return PntPos{
        (float)(left.x + right.x),
        (float)(left.y + right.y),
        (float)(left.z + right.z)
    };
}


PntPos operator-( const PntPos& left, const PntPos& right ){
    // Subtract two PCL points
    return PntPos{
        (float)(left.x - right.x),
        (float)(left.y - right.y),
        (float)(left.z - right.z)
    };
}


PntPos operator/( const PntPos& left, double right ){
    // Divide a PCL point by a scalar
    return PntPos{
        (float)(left.x / right),
        (float)(left.y / right),
        (float)(left.z / right)
    };
}


///// Color Point Cloud Element Operators /////////////////////////////////

PntClr operator+( const PntClr& left, const PntClr& right ){
    // Add two PCL points
    return PntClr{
        (float)(left.x + right.x),
        (float)(left.y + right.y),
        (float)(left.z + right.z),
        left.r,
        left.g,
        left.b,
        left.a
    };
}


PntClr operator-( const PntClr& left, const PntClr& right ){
    // Subtract two PCL points
    return PntClr{
        (float)(left.x - right.x),
        (float)(left.y - right.y),
        (float)(left.z - right.z),
        left.r,
        left.g,
        left.b,
        left.a
    };
}


PntClr operator/( const PntClr& left, double right ){
    // Subtract two PCL points
    return PntClr{
        (float)(left.x / right),
        (float)(left.y / right),
        (float)(left.z / right),
        left.r,
        left.g,
        left.b,
        left.a
    };
}


///// Color Operators /////////////////////////////////////////////////////

std::ostream& operator<<( std::ostream& os , const Color& clr ){ 
	os << "[" << string_format( "%u", clr[0] ) <<  " , " 
              << string_format( "%u", clr[1] ) <<  " , " 
              << string_format( "%u", clr[2] ) <<  " , " 
              << string_format( "%u", clr[3] ) << "]";
	return os;
}

std::ostream& operator<<( std::ostream& os , const XColor& clr ){ 
	os << "[" << string_format( "%i", clr[0] ) <<  " , " 
              << string_format( "%i", clr[1] ) <<  " , " 
              << string_format( "%i", clr[2] ) <<  " , " 
              << string_format( "%i", clr[3] ) << "]";
	return os;
}

XColor operator-( const XColor& left, const XColor& right ){
    // Subtract two eXteneded Colors
    return XColor{
        left[0]-right[0],
        left[1]-right[1],
        left[2]-right[2],
        max( left[3], right[3] )
    };
}


XColor operator+( const XColor& left, const XColor& right ){
    // Add two eXteneded Colors
    return XColor{
        left[0]+right[0],
        left[1]+right[1],
        left[2]+right[2],
        max( left[3], right[3] )
    };
}


XColor operator*( const XColor& left, double right ){
    // Scale an eXteneded Color
    return XColor{
        (int)(left[0]*right),
        (int)(left[1]*right),
        (int)(left[2]*right),
        (int)(left[3]*right)
    };
}


Color get_Color( const XColor& clr ){
    // Cast eXteneded Color to ubyte Color
    return Color{
        (ubyte) max( 0, min( 255, clr[0] ) ),
        (ubyte) max( 0, min( 255, clr[1] ) ),
        (ubyte) max( 0, min( 255, clr[2] ) ),
        (ubyte) max( 0, min( 255, clr[3] ) )
    };
}


XColor get_XColor( const Color& clr ){
    // Cast ubyte Color to eXteneded Color
    return XColor{
        clr[0],
        clr[1],
        clr[2],
        clr[3]
    };
}



////////// IMAGE ANALYSIS //////////////////////////////////////////////////////////////////////////

double measure_sharpness( const Mat& grayImage ){
    /* One common approach involves using the Laplacian operator to detect edges and then calculating 
       the variance of the Laplacian response. A lower variance indicates more blur. */
    Mat laplacianImage;
    cv::Laplacian( grayImage, laplacianImage, CV_64F );
    Scalar mean, stddev;
    cv::meanStdDev( laplacianImage, mean, stddev );
    return stddev[0] * stddev[0]; // Variance
}