// cls && g++ 01_pcd-display.cpp OGL_Display.cpp Structure.cpp Image.cpp `pkg-config --cflags --libs opencv4` -std=c++17 -O3 -Wall -lm -lglut -lGLU -lGL -lopencv_xfeatures2d -I /usr/local/include/opencv4/ -o 01_pcd-show.out

////////// INIT ////////////////////////////////////////////////////////////////////////////////////
#include "SfM.hpp"



////////// GLOBALS /////////////////////////////////////////////////////////////////////////////////
string _CAL_PATH = "input/SfM/00_sculpture/moto-g-power-2022.intrinsic";
string _IMG_PATH = "input/SfM/04_Test-01/";



////////// MAIN ////////////////////////////////////////////////////////////////////////////////////
// vector<Point3d> pts;

// void draw(){
//     glColor4d( 1.0, 1.0, 1.0, 1.0 );
//     glBegin( GL_POINTS );
//     for( Point3d& point : pts ){
//         glVertex3d( point.x, point.y, point.z );
//     }
//     glEnd();
// }

int main(){

    CamData camInfo{ _CAL_PATH, _IMG_PATH }; // Default params are for Moto G Power (2022)

    vector<NodePtr> nodes = images_to_nodes( _IMG_PATH, "jpg", camInfo );
    
    TwoViewCalculator est{};

    TwoViewResult res = est.estimate_pose( camInfo, 
                                           nodes[0]->keyPts, nodes[0]->kpDesc, 
                                           nodes[1]->keyPts, nodes[1]->kpDesc );

    cout << "There are " << res.good_matches.size() << " good matches!" << endl;
    cout << "Translation:\n" << res.t << endl;
    cout << "Rotation:\n" << res.R << endl;
    cout << res.matched_points1.size() << ", " << res.matched_points2.size() << ", " << res.success << endl;

    est.generate_point_cloud( camInfo, res );
    // pts = res.PCD;
    PCPosPtr pcd = vec_Point3d_to_PntPos_pcd( res.PCD, true );

    pcl::visualization::PCLVisualizer::Ptr viewer = simpleVis( pcd );
    
    while( !viewer->wasStopped() ){
        viewer->spinOnce( 100 );
        std::this_thread::sleep_for( 100ms );
    }

    return 0;
}