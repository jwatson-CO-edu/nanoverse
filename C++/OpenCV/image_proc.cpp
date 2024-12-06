////////// INIT ////////////////////////////////////////////////////////////////////////////////////

///// Includes /////

/// Standard ///
#include <vector>
using std::vector;
#include <string>
using std::string, std::to_string, std::stof;

////////// IMAGE PROCESSING ////////////////////////////////////////////////////////////////////////

void fetch_images_at_path( string path, vector<string>& fNames, vector<Mat>& images, 
                           uint limit = 0, string ext = "jpg" ){
    // Load all the images found at a path
    fNames.clear();
    images.clear();
    fNames = list_files_at_path_w_ext( path, ext, true );
    uint /*-----*/ Nimg   = fNames.size();
    Mat /*------*/ img;  
    uint /*-----*/ i = 0;
    cout << "Found: " << Nimg << " files ... " << flush;
    for( string fName : fNames ){
        if( (limit > 0) && (i > (limit-1)) ) break;
        cout << "Found: " << fName << " ... " << flush;
        img = imread( fName, IMREAD_GRAYSCALE );
        images.push_back( img );
        cout << "Loaded: " << fName << ", Bytes: " << (img.total() * img.elemSize()) << endl;
        i++;
    }
    cout << endl << "Got " << images.size() << " images!" << endl;
}