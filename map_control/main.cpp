#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/ximgproc.hpp"

#include <iostream>
#include "Map.h"
#include "path_planning.h"

using namespace std;
using namespace cv;

void printMap(Mat &map) {
    Mat resizeMap;
    resize(map,resizeMap,map.size()*10,0,0,INTER_NEAREST);
    imshow("Map", resizeMap);
}

int main(int argc, char** argv)
{
    const char* default_file = "../map_control/floor_plan.png";
//    const char* default_file = "../map_control/big_floor_plan.png";
    const char* filename = argc >=2 ? argv[1] : default_file;
    Mat src = cv::imread(filename, IMREAD_COLOR);

<<<<<<< HEAD
    Map grid;
    Mat img = grid.bushfire_img(src);
    vector<Point> mid_points = grid.find_centers(img);

    draw_pixel_red(mid_points, src);
    printMap(src);
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
    //SHOWS UPPER AND LOWER TOGETHER
    Mat tempMap = smallWorld;
    cvtColor(smallWorld, tempMap, COLOR_GRAY2BGR);
    vector<Point> upper = smallMap.getUpperTrapezoidalGoals();
    vector<Point> lower = smallMap.getLowerTrapezoidalGoals();

    // DRAWS IN SAME MAP

    for(int i = 0; i < upper.size(); i++) // Upper goal green
    {
        cout << "Upper x: " << upper[i].x << " y: " << upper[i].y << endl;
        tempMap.at<Vec3b>(upper[i].y,upper[i].x)[1] = 255;
    }
    for(int i = 0; i < lower.size(); i++)
    {
        cout << "Lower x: " << lower[i].x << " y: " << lower[i].y << endl;
        tempMap.at<Vec3b>(lower[i].y,lower[i].x)[2] = 255; // Lower goal red
    }

    // COUT COORDINATES FROM PIXELS TO GAZEBO
    vector<Point_<double>> gazeboGoals = smallMap.convertToGazeboCoordinatesTrapezoidal(upper, lower);
    for(int i = 0; i < gazeboGoals.size(); i++)
    {
        cout << "Goal " << i << " x: " << gazeboGoals[i].x << " y: " << gazeboGoals[i].y << endl;
    }


    resize(tempMap,tempMap,tempMap.size()*10,0,0,INTER_NEAREST);
    imshow("Goals", tempMap);
=======
    Path_planning path;
    Point start(2,2), goal(16,2);
    cout << path.way_around_obstacle(start, goal, src) << endl;
>>>>>>> way_around_obstacle

    waitKey(0);
    return 0;
}
