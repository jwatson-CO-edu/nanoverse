#include "SfM.hpp"

////////// SCENE GRAPH /////////////////////////////////////////////////////////////////////////////

///// SG_Node /////////////////////////////////////////////////////////////

SG_Node::SG_Node( const string fName, const Mat& img ){
    // Constructor from an image
    imPth = fName;
    image = img.clone();
    visit = 0;
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

vector<size_t> find_correspondences_I2J( NodePtr nodeI, NodePtr nodeJ ){
    // Simple linear search for the correspondence of each keypoint
    float /*----*/ d_ij = 0.0f;
    float /*----*/ dMin = 1e9f;
    size_t /*---*/ iMin = 0;
    size_t /*---*/ j;
    vector<size_t> matches;
    for( KeyPoint& kp_i : nodeI->kypts ){
        dMin = 1e9f;
        iMin = 0;
        j    = 0;
        for( KeyPoint& kp_j : nodeJ->kypts ){
            d_ij = norm( kp_i.pt - kp_j.pt );
            if( d_ij < dMin ){  
                iMin = j;  
                dMin = d_ij;
            }
            ++j;
        }
        matches.push_back( iMin );
    }
    return matches;
}


void find_graph_correspondences( NodePtr root ){
    // BFS traversal of graph to find correspondences
    queue<NodePtr> frontier;
    NodePtr /*--*/ node_i;
    frontier.push( NodePtr{ root } );
    while( frontier.size() ){
        node_i = frontier.front();
        node_i->visit = 1;
        frontier.pop();
        for( NodePtr node_j : node_i->nhbrs ){
            if( !(node_j->visit) ){
                node_i->match.push_back(  find_correspondences_I2J( node_i, node_j )  );
                frontier.push( NodePtr{ node_j } );
            }
        }
    }
}


void triangulate_keypoints_DLT( NodePtr node1, NodePtr node2 ){
    // https://docs.opencv.org/3.4/d0/dbd/group__triangulation.html
    // https://homepages.inf.ed.ac.uk/rbf/CVonline/LOCAL_COPIES/EPSRC_SSAZ/node3.html
}