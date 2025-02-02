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