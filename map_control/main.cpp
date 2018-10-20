#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/ximgproc.hpp"

#include <iostream>
#include <Map.h>

using namespace std;
using namespace cv;

int main(int argc, char** argv)
{
    const char* default_file = "../map_control/big_floor_plan.png";
    //const char* default_file = "../map_control/floor_plan.png";
    const char* filename = argc >=2 ? argv[1] : default_file;
    // Loads an image
    Mat smallWorld = imread( filename, IMREAD_GRAYSCALE );
    Map smallMap(smallWorld);

    vector<Point> detectedCorners = smallMap.cornerDetection();
    smallMap.trapezoidalLines(detectedCorners);

    smallMap.printMap();
    smallMap.drawNShowPoints("Detected Corners", detectedCorners);
    /* SHOWS UPPER AND LOWER SEPARATELY
    smallMap.drawNShowPoints("Upper Goals", smallMap.getUpperTrapezoidalGoals());
    smallMap.drawNShowPoints("Lower Goals", smallMap.getLowerTrapezoidalGoals());
    */
    /* SHOWS UPPER AND LOWER TOGETHER
    Mat tempMap = smallWorld;
    cvtColor(smallWorld, tempMap, COLOR_GRAY2BGR);
    vector<Point> upper = smallMap.getUpperTrapezoidalGoals();
    vector<Point> lower = smallMap.getLowerTrapezoidalGoals();
    for(int i = 0; i < upper.size(); i++) // Upper goal green
    {
        tempMap.at<Vec3b>(upper[i].y,upper[i].x)[1] = 255;
    }
    for(int i = 0; i < lower.size(); i++)
    {
        tempMap.at<Vec3b>(lower[i].y,lower[i].x)[2] = 255; // Lower goal red
    }
    resize(tempMap,tempMap,tempMap.size()*10,0,0,INTER_NEAREST);
    imshow("Goals", tempMap);
    */
    waitKey(0);
    return 0;
}
