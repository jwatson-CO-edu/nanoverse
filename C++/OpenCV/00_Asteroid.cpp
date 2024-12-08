// g++ 00_Asteroid.cpp SfM.cpp image_proc.cpp helpers.cpp  `pkg-config --cflags --libs opencv4` -std=c++17 -lopencv_xfeatures2d -I /usr/local/include/opencv4/ -o build-asteroid.out

#include "SfM.hpp"

const string _IMAGE_DIR = "images/";
const string _IMAGE_EXT = "png";

int main( int argc, char* argv[] ){

    cout << "About to calculate KAZE keypoints ..." << endl;

    vector<NodePtr> nodes = images_to_nodes( _IMAGE_DIR, _IMAGE_EXT );

    cout << "Calculated KAZE keypoints for the following images ..." << endl;
    for( NodePtr node : nodes ){
        cout << node->imPth << endl;
    }

    return 0;
}