////////// INIT ////////////////////////////////////////////////////////////////////////////////////
#include "SfM.hpp"



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


////////// PATH AND FILE OPERATIONS ////////////////////////////////////////////////////////////////

bool file_has_ext( string path, string ext ){
    // Return true if a file has a 
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


////////// TRIGONOMETRY ////////////////////////////////////////////////////////////////////////////

// Cosine and Sine in degrees
// Author: Willem A. (Vlakkies) Schreüder  
double Cos( double x ){  return cos( (x) * 3.1415927 / 180 );  }
double Sin( double x ){  return sin( (x) * 3.1415927 / 180 );  }
double Tan( double x ){  return tan( (x) * 3.1415927 / 180 );  }
float  Cosf( float x ){  return cosf( (x) * 3.1415927f / 180.0f );  }
float  Sinf( float x ){  return sinf( (x) * 3.1415927f / 180.0f );  }
float  Tanf( float x ){  return tanf( (x) * 3.1415927f / 180.0f );  }
float  Atan2f( float y, float x ){  return (atan2f( y, x ) * 3.1415927f / 180.0f);  }


////////// 3D GEOMETRY /////////////////////////////////////////////////////////////////////////////

Mat skew_symm_mat( Vec3f& x ){
    // Evaluate skew-symmetric matrix, Accepts both 1x3 and 3x1 vectors
    Mat S_x = Mat::zeros( 3, 3, CV_32F );
    S_x.at<float>(0,0) =  0.0f;   S_x.at<float>(0,1) = -x[2];   S_x.at<float>(0,2) =  x[1];
    S_x.at<float>(1,0) =  x[2];   S_x.at<float>(1,1) =  0.0f;   S_x.at<float>(1,2) = -x[0];
    S_x.at<float>(2,0) = -x[1];   S_x.at<float>(2,1) =  x[0];   S_x.at<float>(2,2) =  0.0f;
    return S_x;
}
    