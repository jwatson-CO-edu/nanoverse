#include <opencv2/opencv.hpp>
#include <iostream>
// using namespaces to nullify use of cv::function(); syntax and std::function();
using namespace std;
using namespace cv;
 
int main()
{
    // Reading image
    Mat img = imread("/home/james/nanoverse/C++/OpenCV/data/SfM/01_sculpture/IMG_20240127_152402276_BURST022.jpg");
    // Display original image
    imshow("original Image", img);
    waitKey(0);
 
    // Convert to graycsale
    Mat img_gray;
    cvtColor(img, img_gray, COLOR_BGR2GRAY);
    // Blur the image for better edge detection
    Mat img_blur;
    GaussianBlur(img_gray, img_blur, Size(3,3), 0);
     
    // Sobel edge detection
    Mat sobelx, sobely, sobelxy;
    // Sobel(img_blur, sobelx, CV_64F, 1, 0, 5);
    // Sobel(img_blur, sobely, CV_64F, 0, 1, 5);
    
    Sobel(img_blur, sobelxy, CV_64F, 2, 2, 5);
    
    // Display Sobel edge detection images
    // imshow("Sobel X", sobelx);
    // waitKey(0);
    // imshow("Sobel Y", sobely);
    // waitKey(0);
    imshow("Sobel XY using Sobel() function", sobelxy);
    waitKey(0);
 
    // Canny edge detection
    Mat edges;
    Canny(img_blur, edges, 100, 200, 3, false);
    // Display canny edge detected image
    imshow("Canny edge detection", edges);
    waitKey(0);
     
    destroyAllWindows();
    return 0;
}