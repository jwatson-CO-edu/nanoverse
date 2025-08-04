////////// INIT ////////////////////////////////////////////////////////////////////////////////////
#include "include/utils.hpp"



////////// STRING OPERATIONS ///////////////////////////////////////////////////////////////////////

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


string to_upper( string input ){
    // Return a version of the string that is upper case
    string output;
    for( char& c : input ){ output += toupper( c ); }
    return output;
}



////////// FILE OPERATIONS /////////////////////////////////////////////////////////////////////////

vstr list_files_at_path( string path, bool sortAlpha ){
    // List all the files found at a path, Optionally sort
    vstr   rtnNams;
    string path_i;
    for (const auto & entry : directory_iterator( path ) ){  
        path_i = entry.path().string();
        rtnNams.push_back( path_i );  
    }
    if( sortAlpha )  sort( rtnNams.begin(),rtnNams.end() );
    return rtnNams;
}


bool file_has_ext( string path, string ext ){
    // Return true if a file has the given `ext`
    vstr parts = split_string_on_char( path, '.' );
    // cout << "There are " << parts.size() << " segments!" << endl;
    return (to_upper( parts[ parts.size()-1 ] ) == to_upper( ext ));
}


vstr list_files_at_path_w_ext( string path, string ext, bool sortAlpha ){
    // List all the files found at a path that have the given `ext`, Optionally sort
    vstr allPaths = list_files_at_path( path, sortAlpha );
    vstr rtnPaths;
    for( string fPath : allPaths ){
        if( file_has_ext( fPath, ext ) )
            rtnPaths.push_back( string( fPath ) );
    }
    return rtnPaths;
}