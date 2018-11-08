#ifndef MAP_H
#define MAP_H

#include <iostream>
#include <vector>
#include <math.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/ximgproc.hpp"
#include <algorithm>

using namespace std;
using namespace cv;
using namespace cv::ximgproc;

class Map
{
public:
    Map();
    Map(Mat picture);

    //GET FUNCTIONS
    int getMapRows();
    int getMapCols();
    vector<Point> getUpperTrapezoidalGoals();
    vector<Point> getLowerTrapezoidalGoals();

    vector<Point> cornerDetection();
    void trapezoidalLines(vector<Point> criticalPoints);

    vector<Point_<double>> convertToGazeboCoordinates(vector<Point> goals);
    vector<Point_<double>> convertToGazeboCoordinatesTrapezoidal(vector<Point> upperGoals, vector<Point> lowerGoals);
    //ILLUSTRATIVE FUNCTIONS
    void printMap();
    void drawNShowPoints(string pictureText, vector<Point> points);

    // BRUSHFIRE
    Mat brushfire_img(Mat &img);
    vector<Point> find_centers(Mat &img);

    ~Map();

private:
    //Atributes
    Mat map;
    vector<Point> upperTrapezoidalGoals;
    vector<Point> lowerTrapezoidalGoals;

    // Bushfire
    void binarize_img(Mat &img);
    void find_neighbors(vector<Point> &v, Mat &img, int x, int y);
    void make_brushfire_grid(Mat &img);
    void remove_points_in_corners(vector<Point> &v, Mat &img);
};

#endif // MAP_H
