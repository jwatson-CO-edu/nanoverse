#ifndef HELPERS_HPP
#define HELPERS_HPP

////////// STANDARD CONTAINERS /////////////////////////////////////////////////////////////////////
#include <vector>
using std::vector;
#include <string>
using std::string;


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
            rtnStr << mat.at<T>(i,j) << ",";
        }    
    }
    return rtnStr;
}

#endif