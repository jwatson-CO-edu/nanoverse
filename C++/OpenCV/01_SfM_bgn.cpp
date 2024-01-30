////////// DEV PLAN & NOTES ////////////////////////////////////////////////////////////////////////
/*
##### Sources #####
* https://imkaywu.github.io/tutorials/sfm/


##### DEV PLAN #####
[ ] Load images in a dir
[ ] 00 SURF Example
[ ] Compute ORB features for one image
[ ] Basic SfM Tutorial: https://imkaywu.github.io/tutorials/sfm/
    [ ] Compute ORB for all images
    [ ] Feature matching
        [ ] Match features between each pair of images in the input image set
            [ ] Brute force
            [ ] Approximate nearest neighbour library, such as FLANN, ANN, Nanoflann
        [ ] Bi-directional verification
    [ ] Relative pose estimation
        [ ] For each pair of images with sufficient number of matches, compute relative pose
        [ ] N-view triangulation
        [ ] Bundle adjustment
    [ ] Two-view SfM
        [ ] Render PC
    [ ] N-view SfM
        [ ] iterative step that merges multiple graphs into a global graph, for graph in graphs
            [ ] pick graph g in graphs having maximum number of common tracks with the global graph
            [ ] merge tracks of global graph with those of graph g
            [ ] triangulate tracks
            [ ] perform bundle adjustment
            [ ] remove track outliers based on baseline angle and reprojection error
            [ ] perform bundle adjustment
        [ ] Render PC 
            [ ] Point Cloud
            [ ] Camera poses
            [ ] Fly around
[ ] Advanced Textured SfM
    [ ] SfM PC
    [ ] Delaunay at each camera view
    [ ] Consolidate triangulations
    [ ] Project textures at each camera view
    [ ] Consolidate textures @ facet objects
    [ ] Write individual textures to a single image file
    [ ] OBJ Output
    [ ] Render OBJ (or other appropriate format)
[ ] Virtual Sculpture Garden (Dangerous!)
    [ ] Place sculptures tastefully
    [ ] Add FPV
    [ ] Post walkthrough to IG
*/


////////// INIT ////////////////////////////////////////////////////////////////////////////////////

///// Includes /////

/// Standard ///
#include <vector>
using std::vector;
#include <string>
using std::string;
#include <filesystem>
using std::filesystem;
#include <iostream>
using std::cout, std::endl, std::flush;

/// Special ///
#include <opencv2/opencv.hpp>

////////// UTILITY FUNCTIONS ///////////////////////////////////////////////////////////////////////

vector<string> list_files_at_path( string path, bool prepend = false ){
    // List all the files found at a path
    vector<string> rtnNams;
    string /*---*/ path_i;
    for (const auto & entry : directory_iterator( path ) ){  
        if( prepend )
            path_i = path + "/" + entry.path();
        else
            path_i = entry.path();
        rtnNams.push_back( path_i );  
    }
    return rtnNams;
}



////////// GLOBALS /////////////////////////////////////////////////////////////////////////////////
string _IMG_PATH = "";



////////// IMAGE PROCESSING ////////////////////////////////////////////////////////////////////////


vector<Mat> fetch_images_at_path( string path ){
    vector<string> fNames = list_files_at_path( path, true );
    uint /*-----*/ Nimg   = fNames.size();
    vector<Mat>  images;
    for( string fName : fNames ){
        images.push_back( imread( fName, IMREAD_COLOR ) );
        cout << fName << endl;
    }
    cout << endl << "Got " << images.size() << " images!" << endl;
    return images;
}



////////// MAIN ////////////////////////////////////////////////////////////////////////////////////
int main(){

    fetch_images_at_path( _IMG_PATH );

    return 0;
}