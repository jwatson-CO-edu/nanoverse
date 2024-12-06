////////// INIT ////////////////////////////////////////////////////////////////////////////////////

///// Includes /////

/// Standard ///
#include <iostream>
using std::cout, std::endl;
#include <vector>
using std::vector;
#include <string>
using std::string, std::to_string;
#include <memory>
using std::shared_ptr;




////////// SCENE GRAPH /////////////////////////////////////////////////////////////////////////////
class SG_Node;
typedef shared_ptr<SG_Node> NodePtr;


class SG_Node{ public:
    // Scene Graph Node for Structure from Motion. Assumes one image per pose

    /// Members ///
    string /*-----*/ path;
    Mat /*--------*/ imag;
    vector<KeyPoint> kpts;
    Mat /*--------*/ xfrm;
    vector<NodePtr>  nbrs;

};

vector<NodePtr> images_to_nodes( string path );