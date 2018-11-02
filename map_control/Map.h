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

struct cell
{
    int cellNumber;
    int x;
    int y;
    vector<cell> neighborcells;
};

class Map
{
public:
    Map();
    Map(Mat picture);

    //GET FUNCTIONS
    int getMapRows();
    int getMapCols();
    Mat getSweepLineMap();
    vector<Point> getUpperTrapezoidalGoals();
    vector<Point> getLowerTrapezoidalGoals();

    vector<Point> cornerDetection();
    void trapezoidalLines(vector<Point> criticalPoints);
    vector<cell> calculateCells(vector<Point> upperTrap, vector<Point> lowerTrap);

    vector<Point_<double>> convertToGazeboCoordinates(vector<Point> goals);
    vector<Point_<double>> convertToGazeboCoordinatesTrapezoidal(vector<Point> upperGoals, vector<Point> lowerGoals);
    //ILLUSTRATIVE FUNCTIONS
    void printMap();
    void drawNShowPoints(string pictureText, vector<Point> points);

    // BUSHFIRE
    Mat bushfire_img(Mat &img);
    vector<Point> find_centers(Mat &img);

    ~Map();

private:
    //Atributes
    Mat map;
    Mat sweepLineMap;
    vector<Point> upperTrapezoidalGoals;
    vector<Point> lowerTrapezoidalGoals;
    vector<cell> cells;
    // Bushfire
    void binarize_img(Mat &img);
    void find_neighbors(vector<Point> &v, Mat &img, int x, int y);
    void make_bushfire_grid(Mat &img);
    void remove_points_in_corners(vector<Point> &v, Mat &img);
};

#endif // MAP_H
