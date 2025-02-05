// cls && g++ 00_test_01.cpp Structure.cpp Image.cpp `pkg-config --cflags --libs opencv4` -std=c++17 -lopencv_xfeatures2d -I /usr/local/include/opencv4/ -o 00_two-image-kps.out

////////// INIT ////////////////////////////////////////////////////////////////////////////////////
#include "SfM.hpp"



////////// GLOBALS /////////////////////////////////////////////////////////////////////////////////
string _CAL_PATH = "input/SfM/00_sculpture/moto-g-power-2022.intrinsic";
string _IMG_PATH = "input/SfM/04_Test-01/";




////////// MAIN ////////////////////////////////////////////////////////////////////////////////////

int main(){

    // ulong Nfeat  = 25000;
    // ulong Ktop   = Nfeat/2;

    CamData camInfo{ _CAL_PATH, _IMG_PATH }; // Default params are for Moto G Power (2022)

    vector<NodePtr> nodes = images_to_nodes( _IMG_PATH, "jpg" );
    
    TwoViewCalculator est{};

    TwoViewResult res = est.estimate_pose( camInfo, 
                                        nodes[0]->keyPts, nodes[0]->kpDesc, 
                                        nodes[1]->keyPts, nodes[1]->kpDesc );

    cout << "There are " << res.good_matches.size() << " good matches!" << endl;
    cout << "Translation:\n" << res.t << endl;
    cout << "Rotation:\n" << res.R << endl;
    cout << res.matched_points1.size() << ", " << res.matched_points2.size() << ", " << res.success << endl;

    est.generate_point_cloud( res.matched_points1, res.matched_points2,
                              res.R, res.t, camInfo );

    

    // cout << "Returned a " <<  <<  "x" << " matrix"

    // est.visualize_matches( nodes[0]->image, nodes[0]->keyPts, nodes[1]->image, nodes[1]->keyPts, res );

    return 0;
}