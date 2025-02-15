// cls && g++ 01_pcd-display.cpp OGL_Display.cpp Structure.cpp Image.cpp `pkg-config --cflags --libs opencv4` -std=c++17 -O3 -Wall -lm -lglut -lGLU -lGL -lopencv_xfeatures2d -I /usr/local/include/opencv4/ -o 01_pcd-show.out

////////// INIT ////////////////////////////////////////////////////////////////////////////////////
#include "SfM.hpp"



////////// GLOBALS /////////////////////////////////////////////////////////////////////////////////
string _CAL_PATH = "input/SfM/00_sculpture/moto-g-power-2022.intrinsic";
string _IMG_PATH = "input/SfM/09_Test-02/";
// string _IMG_PATH = "input/SfM/00_sculpture/";



////////// MAIN ////////////////////////////////////////////////////////////////////////////////////

int main(){

    CamData camInfo{ _CAL_PATH, _IMG_PATH }; // Default params are for Moto G Power (2022)

    vector<NodePtr> nodes    = images_to_nodes( _IMG_PATH, "jpg", camInfo, 100.0 );
    PCXYZPtr /*--*/ totCloud = node_seq_to_PointXYZ_pcd( nodes[0] );
    
    pcl::visualization::PCLVisualizer::Ptr viewer = simpleVis( totCloud );
    
    while( !viewer->wasStopped() ){
        viewer->spinOnce( 100 );
        std::this_thread::sleep_for( 100ms );
    }

    return 0;
}