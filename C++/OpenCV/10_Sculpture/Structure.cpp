#include "SfM.hpp"


ImgNode::ImgNode( string path, const Mat& sourceImg ){
    imgPth = path;
    image  = sourceImg;
}


vector<NodePtr> images_to_nodes( string path, string ext ){
    // Populate a vector of nodes with paths and images
    NodePtr /*---*/ node_i;
    vector<NodePtr> rtnNds;
    vector<string>  fNames;
    vector<Mat>     images;
    KAZE /*------*/ kazeMaker{};
    Mat /*-------*/ kpNfo; // Keypoint Info (Not used?)
    fetch_images_at_path( path, fNames, images, 0, ext );
    uint N = images.size();
    cout << endl;
    for( size_t i = 0; i < N; ++i ){
        node_i = NodePtr{ new ImgNode{ fNames[i], images[i] } };
        kazeMaker.get_KAZE_keypoints( images[i], node_i->keyPts, kpNfo );
        // Assume pix were taken in a sequence
        if( i > 0 ){  
            node_i->prev = NodePtr{ rtnNds.back() };
            rtnNds.back()->next = NodePtr{ node_i };
        } 
        rtnNds.push_back( NodePtr{ node_i } );
        cout << node_i->keyPts.size() << " kp, " << flush;
    }
    cout << endl;
    return rtnNds;
}