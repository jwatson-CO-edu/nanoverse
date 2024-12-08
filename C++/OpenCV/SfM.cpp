#include "SfM.hpp"

////////// SCENE GRAPH /////////////////////////////////////////////////////////////////////////////

///// SG_Node /////////////////////////////////////////////////////////////

SG_Node::SG_Node( const string fName, const Mat& img ){
    imPth = fName;
    image = img.clone();
}

vector<NodePtr> images_to_nodes( string path, string ext ){
    // Populate a vector of nodes with paths and images
    vector<NodePtr> rtnNds;
    vector<string>  fNames;
    vector<Mat>     images;
    fetch_images_at_path( path, fNames, images, 0, ext );
    uint N = images.size();
    for( uint i = 0; i < N; ++i ){
        rtnNds.push_back( NodePtr{ new SG_Node( fNames[i], images[i] ) } );
    }
    return rtnNds;
}

