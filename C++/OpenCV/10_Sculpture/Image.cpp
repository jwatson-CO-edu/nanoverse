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
    uint /*-----*/ Nimg   = fNames.size();
    Mat /*------*/ img;  
    uint /*-----*/ i = 0;
    cout << "Found: " << Nimg << " files ... " << flush;
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

Mat deserialize_2d_Mat_f( string input, int Mrows, int Ncols, char sep ){
    // Deserialize an OpenCV `CV_32F` matrix stored row-major in a comma-separated list in a string
    Mat rtnMat = Mat::zeros( Mrows, Ncols, CV_32F );

    // cout << "Got input: " << input << endl;

    vector<string> tokens = split_string_on_char( input, sep );
    // cout << "Separated input: " << endl;
    // for( string& token : tokens ) cout << '\t' << token << endl;


    if( tokens.size() < Mrows*Ncols )  return rtnMat; // Return the zero matrix if there are insufficient elements
    int k = 0;
    float val;
    string item;
    for( int i = 0; i < Mrows; ++i ){
        for( int j = 0; j < Ncols; ++j ){
            // cout << endl << "\t\tAt (" << i << ',' << j << "), " << k << flush;
            // cout << ", " << tokens[k] << flush;
            // item = tokens[k];
            // item += '\0'
            // cout << ", " << stof( tokens[k] ) << flush;
            try{
                rtnMat.at<float>(i,j) = stof( tokens[k] );
            }catch (const std::out_of_range& e) {
                cout << "Out of Range error." << endl;
                rtnMat.at<float>(i,j) = nanf("");
            }
            ++k;
        }
    }
    return rtnMat;
}

// Get everything after the last ':' in a string
string get_line_arg( string line ){  return get_last( split_string_on_char( line, ':' ) );  }