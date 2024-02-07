#ifndef HELPERS_HPP
#define HELPERS_HPP

#include <vector>
using std::vector;

template<typename T>
bool p_vec_has_item( const vector<T>& vec, const T& item ){
    for( const T& elem : vec ) if( elem == item )  return true;
    return false;
}

#endif