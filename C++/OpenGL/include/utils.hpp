#ifndef UTILS_HPP // This pattern is to prevent symbols to be loaded multiple times
#define UTILS_HPP // from multiple imports

////////// INIT ////////////////////////////////////////////////////////////////////////////////////
#include <vector>
using std::vector;
#include <list>
using std::list;
#include <array>
using std::array;
#include <string>
using std::string;
#include <filesystem>
using std::filesystem::directory_iterator;
#include <algorithm>
using std::sort;


///// TYPE DEFINES ////////////////////////////////////////////////////////
typedef vector<string> vstr;
typedef vector<vstr>   vvstr;
typedef array<uint,2>  vec2u;
typedef vector<vec2u>  vvec2u;


////////// CONTAINERS //////////////////////////////////////////////////////////////////////////////
template<typename T>
vector<T> list_2_vector( const list<T>& lst ){
    // Convert a Linked List to a Dynamic Array
    vector<T> rtnVec;
    size_t    i = 0;
    rtnVec.reserve( lst.size() );
    for( const T& elem : lst ){
        // rtnVec[i] = elem;
        rtnVec.push_back( elem );
        ++i;
    }
    return rtnVec;
}



////////// STRING OPERATIONS ///////////////////////////////////////////////////////////////////////
vstr split_string_on_char( string input, char ch ); // Return a vector of strings found in `input` separated by whitespace
string to_upper( string input ); // ------------------ Return a version of the string that is upper case
std::ostream& operator<<( std::ostream& os , vvstr arr );


////////// FILE OPERATIONS /////////////////////////////////////////////////////////////////////////
vstr  list_files_at_path( string path, bool sortAlpha = true ); // ----------------- List all the files found at a path, Optionally sort
bool  file_has_ext( string path, string ext ); // ---------------------------------- Return true if a file has the given `ext`
vstr  list_files_at_path_w_ext( string path, string ext, bool sortAlpha = true ); // List all the files found at a path that have the given `ext`, Optionally sort
// List all the files found at given paths that have the given `ext`, Optionally sort
vvstr list_files_at_paths_w_ext( const vstr& paths, string ext, bool sortAlpha = true ); 

#endif