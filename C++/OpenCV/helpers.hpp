#ifndef HELPERS_HPP
#define HELPERS_HPP

////////// INIT ////////////////////////////////////////////////////////////////////////////////////
#include <vector>
using std::vector;
#include <string>
using std::string, std::to_string;
#include <sys/stat.h>
#include <fstream>
using std::ifstream, std::ofstream;
#include <iostream>
using std::cout, std::endl, std::flush;

#include "Eigen/Core"
typedef Eigen::Vector2i /*-------*/ Vec2i;
typedef Eigen::Vector2f /*-------*/ Vec2f;
typedef Eigen::Vector3i /*-------*/ Vec3i;
typedef Eigen::Vector3f /*-------*/ Vec3f;
typedef Eigen::Vector3d /*-------*/ Vec3;
typedef Eigen::Vector4i /*-------*/ Vec4i;
typedef Eigen::Matrix<float, 6, 1>  Vec6f;
typedef Eigen::Matrix<float, 9, 1>  Vec9f;
typedef Eigen::Matrix<double, 9, 1> Vec9;

typedef Eigen::Matrix2f /*-------*/ Mat2f;
typedef Eigen::Matrix3f /*-------*/ Mat3f;
typedef Eigen::Matrix4i /*-------*/ Mat4i;
typedef Eigen::Matrix4f /*-------*/ Mat4f;
typedef Eigen::Matrix<float, 3, 4>  Mat34f;


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

vector<string> list_files_at_path_w_ext( string path, string ext, bool sortAlpha = true ){
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


////////// STANDARD CONTAINERS /////////////////////////////////////////////////////////////////////


template<typename T>
bool p_vec_has_item( const vector<T>& vec, const T& item ){
    for( const T& elem : vec ) if( elem == item )  return true;
    return false;
}

template<typename T>
T get_last( vector<T>& vec ){  
    // Get the last element of a vector, if it exists, otherwise throw an index error
    size_t N = vec.size();
    if( N > 0 )
        return vec[ N-1 ];
    else
        throw std::out_of_range{ "get_last: Vector was EMPTY!" };
}

template<typename T>
T get_last( const vector<T>& vec ){  
    // Get the last element of a vector, if it exists, otherwise throw an index error
    size_t N = vec.size();
    if( N > 0 )
        return vec[ N-1 ];
    else
        throw std::out_of_range{ "get_last: Vector was EMPTY!" };
}


////////// OPENCV //////////////////////////////////////////////////////////////////////////////////
#include <opencv2/core/utility.hpp>
using cv::Mat;

template<typename T>
string serialize_Mat_2D( const Mat& mat ){
    string rtnStr;
    size_t M = mat.rows;
    size_t N = mat.cols;
    for( size_t i = 0; i < M; ++i ){
        for( size_t j = 0; j < N; ++j ){
            rtnStr += to_string( mat.at<T>(i,j) ) + ',';
        }    
    }
    return rtnStr;
}

#endif