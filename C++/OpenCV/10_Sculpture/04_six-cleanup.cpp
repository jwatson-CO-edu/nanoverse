////////// INIT ////////////////////////////////////////////////////////////////////////////////////
#include "SfM.hpp"



////////// GLOBALS /////////////////////////////////////////////////////////////////////////////////
string _CAL_PATH = "input/SfM/00_sculpture/moto-g-power-2022.intrinsic";
string _IMG_PATH = "input/SfM/09_Test-02/";
// string _IMG_PATH = "input/SfM/00_sculpture/";



////////// MAIN ////////////////////////////////////////////////////////////////////////////////////

int main( int argc, char* argv[] ){

    CamData camInfo{ _CAL_PATH, _IMG_PATH }; // Default params are for Moto G Power (2022)

    vector<NodePtr> nodes = images_to_nodes( _IMG_PATH, "jpg", camInfo, 5.0, 40.0, 50.0 );

    remove_major_planes_from_clouds( nodes[0], 2.5 ); // 5.0

    cout << "About to colorize nodes ..." << endl;
    node_seq_to_PntPos_pcd( nodes[0], true );
    colorize_node_seq_pcd( nodes[0], GREEN, RED, true );
    
    VizPtr viewer = viz_current_soln( nodes[0] );
    
    while( !viewer->wasStopped() ){
        viewer->spinOnce( 100 );
        std::this_thread::sleep_for( 100ms );
    }

    return 0;
}