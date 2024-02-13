#ifndef HELPERS_HPP
#define HELPERS_HPP

////////// INIT ////////////////////////////////////////////////////////////////////////////////////
#include <vector>
using std::vector;
#include <string>
using std::string, std::to_string;

////////// STRING PROCESSING ///////////////////////////////////////////////////////////////////////


vector<string> split_string_ws( string input ){
    // Return a vector of strings found in `input` separated by whitespace
    vector<string> rtnWords;
    string /*---*/ currWord;
    char /*-----*/ currChar;
    size_t /*---*/ strLen = input.size();

    input.push_back( ' ' ); // Separator hack
    
    for( size_t i = 0 ; i < strLen ; i++ ){
        currChar = input[i];
        if( isspace( currChar ) ){
            if( currWord.length() > 0 )  rtnWords.push_back( currWord );
            currWord = "";
        }else{
            currWord.push_back( currChar );
        }
    }
    return rtnWords; 
}

// FIXME, START HERE: NEED TO SPLIT ON AN ARBITRARY CHAR
// FIXME: NEED TO FILTER ON FILE EXTENSION

////////// STANDARD CONTAINERS /////////////////////////////////////////////////////////////////////


template<typename T>
bool p_vec_has_item( const vector<T>& vec, const T& item ){
    for( const T& elem : vec ) if( elem == item )  return true;
    return false;
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