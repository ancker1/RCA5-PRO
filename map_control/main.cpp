#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/ximgproc.hpp"

#include <iostream>
#include <fstream>
#include <vector>
#include <Map.h>
#include <Cell.h>
#include <Cellpoint.h>

using namespace std;
using namespace cv;

void printMap(Mat &map, string s) {
    Mat resizeMap;
    resize( map, resizeMap, map.size()*10, 0, 0, INTER_NEAREST);
    imshow(s, resizeMap);
}

void draw_pixel_red(vector<Point> &v, Mat &img) {
    for (size_t i = 0; i < v.size(); i++) {
        Vec3b color(0, 0, 255);
        img.at<Vec3b>(v[i].y, v[i].x) = color;
    }
}

int main(int argc, char** argv)
{
    //const char* default_file = "../map_control/floor_plan.png";
    const char* default_file = "../map_control/big_floor_plan.png";
    const char* filename = argc >=2 ? argv[1] : default_file;
    Mat src = cv::imread(filename, IMREAD_COLOR);

    Map grid;
    Mat img = grid.brushfire_img(src);
    vector<Point> mid_points = grid.find_centers(img);

    draw_pixel_red(mid_points, src);
    //printMap(src);
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
    for(size_t i = 0; i < upper.size(); i++) // Upper goal green
    {
        //cout << "Upper x: " << upper[i].x << " y: " << upper[i].y << endl;
        tempMap.at<Vec3b>(upper[i].y,upper[i].x)[1] = 255;
    }
    for(size_t i = 0; i < lower.size(); i++)
    {
        //cout << "Lower x: " << lower[i].x << " y: " << lower[i].y << endl;
        tempMap.at<Vec3b>(lower[i].y,lower[i].x)[2] = 255; // Lower goal red
    }

    // COUT COORDINATES FROM PIXELS TO GAZEBO
    vector<Point_<double>> gazeboGoals = smallMap.convertToGazeboCoordinatesTrapezoidal(upper, lower);
    for(size_t i = 0; i < gazeboGoals.size(); i++)
    {
        //cout << "Goal " << i << " x: " << gazeboGoals[i].x << " y: " << gazeboGoals[i].y << endl;
    }

    resize(tempMap,tempMap,tempMap.size()*10,0,0,INTER_NEAREST);
    imshow("Goals", tempMap);
    vector<Cell> t = smallMap.calculateCells(upper, lower);
    vector<Cellpoint> allCellpoint;
    for(size_t i = 0; i < t.size(); i++)
    {
        for(size_t j = 0; j < t[i].getAllCellPoints().size(); j++)
        {
            allCellpoint.push_back(t[i].getAllCellPoints()[j]);
        }
    }
    //smallMap.astar(allCellpoint, Point(2,63), Point(114,55));
    //smallMap.drawCellsPath("Cell Path", t);
    /*
    for(size_t i = 0; i < t.size(); i++)
    {
        for(size_t j = 0; j < t[i].neighborcells.size(); j++)
        {
            cout << "Celle 1 " << i << " Punkt x: " << t[i].cellPoint.x << " Punkt y: " << t[i].cellPoint.y << " Connected til " << j << " x: " << t[i].neighborcells[j].x << " y: " << t[i].neighborcells[j].y << endl;
        }
    }
    */
    /* Writes to file
    // OPSTACLE
    vector<Point> obstacle;
    Point temp;
    for(int i = 0; i < smallWorld.rows; i++)
    {
        for(int j = 0; j < smallWorld.cols; j++)
        {
            if(smallWorld.at<uchar>(i,j) == 255)
            {
                temp.x = j;
                temp.y = i;
                obstacle.push_back(temp);
            }
        }
    }

    vector<Point_<double>> opstaclegazebo = smallMap.convertToGazeboCoordinates(obstacle);
    ofstream myfile1;
    myfile1.open ("x-coordinates.txt");
    for(size_t i = 0; i < opstaclegazebo.size(); i++)
    {
        myfile1 << opstaclegazebo[i].x << "\n";
    }
    myfile1.close();

    myfile1.open ("y-coordinates.txt");
    for(size_t i = 0; i < opstaclegazebo.size(); i++)
    {
        myfile1 << opstaclegazebo[i].y << "\n";
    }
    myfile1.close();

    myfile1.open ("x-coordinates-goals.txt");
    for(size_t i = 0; i < gazeboGoals.size(); i++)
    {
        myfile1 << gazeboGoals[i].x << "\n";
    }
    myfile1.close();
    myfile1.open ("y-coordinates-goals.txt");
    for(size_t i = 0; i < gazeboGoals.size(); i++)
    {
        myfile1 << gazeboGoals[i].y << "\n";
    }
    myfile1.close();
    */

    waitKey(0);
    return 0;
}
