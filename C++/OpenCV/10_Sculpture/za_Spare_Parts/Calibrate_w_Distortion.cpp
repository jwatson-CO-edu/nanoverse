#include <opencv2/opencv.hpp>
#include <iostream>

using namespace cv;
using namespace std;

int main() {
    // Define chessboard dimensions
    int numCornersHor = 6;
    int numCornersVer = 9;
    int numSquares = numCornersHor * numCornersVer;
    Size patternSize = Size(numCornersHor, numCornersVer);

    // Vector to store 2D points
    vector<vector<Point2f>> imgPoints;

    // Vector to store 3D points
    vector<vector<Point3f>> objPoints;

    // Define world coordinates for chessboard corners
    vector<Point3f> objp;
    for (int i = 0; i < numCornersVer; i++) {
        for (int j = 0; j < numCornersHor; j++) {
            objp.push_back(Point3f(j, i, 0));
        }
    }

    // Read images and find chessboard corners
    vector<String> images;
    glob("path/to/calibration/images/*.jpg", images); // Path to calibration images
    Mat gray;
    bool patternFound;

    for (const auto& imagePath : images) {
        Mat img = imread(imagePath);
        cvtColor(img, gray, COLOR_BGR2GRAY);

        vector<Point2f> corner_pts;
        patternFound = findChessboardCorners(gray, patternSize, corner_pts, CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_FAST_CHECK | CALIB_CB_NORMALIZE_IMAGE);

        if (patternFound) {
            imgPoints.push_back(corner_pts);
            objPoints.push_back(objp);
        }
        drawChessboardCorners(img, patternSize, Mat(corner_pts), patternFound);
        imshow("Chessboard", img);
        waitKey(100);
    }
    destroyAllWindows();
    
    // Calibrate camera
    Mat cameraMatrix, distCoeffs, R, T;
    calibrateCamera(objPoints, imgPoints, gray.size(), cameraMatrix, distCoeffs, R, T);

    // Save camera parameters
    FileStorage fs("camera_params.xml", FileStorage::WRITE);
    fs << "cameraMatrix" << cameraMatrix;
    fs << "distCoeffs" << distCoeffs;
    fs.release();
    
    cout << "Camera calibration done" << endl;
    return 0;
}