// cls && g++ 02_pcd-test.cpp OGL_Display.cpp Structure.cpp Image.cpp `pkg-config --cflags --libs opencv4` -std=c++17 -O3 -Wall -lm -lglut -lGLU -lGL -lopencv_xfeatures2d -I /usr/local/include/opencv4/ -o 02_pcd-test.out

////////// INIT ////////////////////////////////////////////////////////////////////////////////////
#include "SfM.hpp"



////////// HELPER FUNCTIONS ////////////////////////////////////////////////////////////////////////

void init_rand(){  srand( time( NULL ) );  }

double rand_d(){
    // Return a pseudo-random number between 0.0 and 1.0
    return ((double) rand()) / RAND_MAX;
}

double randrange_d( double lo, double hi ){
    // Return a pseudo-random number between `lo` and `hi`
    // NOTE: This function assumes `hi > lo`
    double span = hi - lo;
    return lo + span * rand_d();
}

Point3d sample_3D_noise( Point3d mag ){
    return Point3d{
        randrange_d( -mag.x, mag.x ),
        randrange_d( -mag.y, mag.y ),
        randrange_d( -mag.z, mag.z ),
    };
}

vector<Point3d> sample_3D_noise_N( Point3d mag, size_t N ){
    vector<Point3d> rtnPts;
    for( size_t i = 0; i < N; ++i ){  rtnPts.push_back( sample_3D_noise( mag ) );  }
    return rtnPts;
}


////////// MAIN ////////////////////////////////////////////////////////////////////////////////////
vector<Point3d> pts;

void draw(){
    glColor4d( 1.0, 1.0, 1.0, 1.0 );
    glBegin( GL_POINTS );
    cout << "About to draw " << pts.size() << " points ..." << endl;
    for( Point3d& point : pts ){
        glVertex3d( point.x, point.y, point.z );
    }
    glEnd();
}

int main(){

    pts = sample_3D_noise_N( Point3d{0.5,0.5,0.5}, 3000 );

    OGL_window& win = OGL_window::make_window( 800, 800, "PCD" );
    win.dim = 5.0;
    win.set_callbacks( draw );
    win.set_eye_target( Point3d{3.0,3.0,32.0}, Point3d{0.0,0.0,0.0} );
    win.run();

    return 0;
}

