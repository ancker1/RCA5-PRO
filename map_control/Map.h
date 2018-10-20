#ifndef MAP_H
#define MAP_H
#include <iostream>
#include <vector>
#include <math.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>

using namespace std;
using namespace cv;

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

    //ILLUSTRATIVE FUNCTIONS
    void printMap();
    void drawNShowPoints(string pictureText, vector<Point> points);

    ~Map();



private:

    //Atributes
    Mat map;
    vector<Point> upperTrapezoidalGoals;
    vector<Point> lowerTrapezoidalGoals;
};

#endif // MAP_H
