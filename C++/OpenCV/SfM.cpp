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
    SG_Node* /*--*/ node_i;
    KAZE /*------*/ kazeMaker{};
    fetch_images_at_path( path, fNames, images, 0, ext );
    uint N = images.size();
    cout << endl;
    for( uint i = 0; i < N; ++i ){
        node_i = new SG_Node( fNames[i], images[i] );
        kazeMaker.get_KAZE_keypoints( images[i], node_i->kypts, node_i->kpNfo );
        rtnNds.push_back( NodePtr{ node_i } );
        cout << node_i->kypts.size() << " kp, " << flush;
    }
    cout << endl;
    return rtnNds;
}



////////// TRIANGULATION ///////////////////////////////////////////////////////////////////////////

void triangulate_keypoints_DLT( Mat& uv1, Mat& uv2 ){
    // FIXME, START HERE: https://docs.opencv.org/3.4/d0/dbd/group__triangulation.html
    // https://homepages.inf.ed.ac.uk/rbf/CVonline/LOCAL_COPIES/EPSRC_SSAZ/node3.html
}