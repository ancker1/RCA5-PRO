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
#include <Cell.h>
#include <Cellpoint.h>
using namespace std;
using namespace cv;
using namespace cv::ximgproc;

struct cell
{
    Point cellPoint;
    vector<Point> neighborcells;
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
    vector<Cell> calculateCells(vector<Point> upperTrap, vector<Point> lowerTrap);

    vector<Point_<double>> convertToGazeboCoordinates(vector<Point> goals);
    vector<Point_<double>> convertToGazeboCoordinatesTrapezoidal(vector<Point> upperGoals, vector<Point> lowerGoals);

    //ILLUSTRATIVE FUNCTIONS
    void printMap();
    void drawNShowPoints(string pictureText, vector<Point> points);
    void drawCellsPath(string pictureText, vector<Cell> cells);
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

    //Functions
    vector<Point> sortxAndRemoveDuplicate(vector<Point> list);
    vector<Point> findSamexPointWithoutObstacle(vector<Point> list);
    bool metObstacleDownOrUp(Point start, Point end);
    bool metObstacleLeft(Point start, Point end);
    bool metObstacleRight(Point start, Point end);
    tuple<string,Point> getClosestPointLeft(vector<Point> samex, vector<Point> nonSamex, Point cellPoint);
    tuple<string, Point> getClosestPointRight(vector<Point> samex, vector<Point> nonSamex, Point cellPoint);
    bool obstacleDetectedWithLine(Point start, Point end);
    vector<Point> get_points(LineIterator &it);
    bool isRightSameCellpoint(vector<Cellpoint> list, Point cellpoint, Point connectionPointRight);
    bool isLeftSameCellpoint(vector<Cellpoint> list, Point cellpoint, Point connectionPointLeft);
    bool findNremovePoint(vector<Point> &list, Point point);


    Point checkPointIfGoal(int currentPointx, int currentPointy, vector<Point> goals);
    void deleteNonClosesPoint(cell checkCell);
    // Bushfire
    void binarize_img(Mat &img);
    void find_neighbors(vector<Point> &v, Mat &img, int x, int y);
    void make_brushfire_grid(Mat &img);
    void remove_points_in_corners(vector<Point> &v, Mat &img);
};

#endif // MAP_H
