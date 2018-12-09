#ifndef BOUSTROPHEDON_H
#define BOUSTROPHEDON_H

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

class Boustrophedon
{
public:
    /****************** Contructer/Decontructer *****************/
    Boustrophedon();
    Boustrophedon(Mat map);
    ~Boustrophedon();

    /*********************** Get Functions **********************/
    Mat getMap();
    vector<Point> getCorners();
    vector<Point> getUpperMidpoints();
    vector<Point> getLowerMidpoints();

    /********************** Print Functions *********************/
    Mat drawNShowPoints(vector<vector<Point>> points);
    Mat drawCellsPath(vector<Cell> cells);

    /******************* Calculation Functions ******************/
    vector<Cell> calculateCells();

protected:
    /************************* Atributes ************************/
    Mat map;
    vector<Point> corners;
    vector<Point> upperMidpoints;
    vector<Point> lowerMidpoints;

    /***************** Private Helping Functions ****************/
    void cornerDetection();
    void findMidPoint();

    vector<Point> sortxAndRemoveDuplicate(vector<Point> list);
    vector<Point> findSamexPointWithoutObstacle(vector<Point> list);
    bool metObstacleDownOrUp(Point start, Point end);

    bool metObstacleLeft(Point start, Point end);
    bool metObstacleRight(Point start, Point end);
    tuple<string,Point> getClosestPointLeft(vector<Point> samex, vector<Point> nonSamex, Point cellPoint);
    tuple<string, Point> getClosestPointRight(vector<Point> samex, vector<Point> nonSamex, Point cellPoint);

    bool obstacleDetectedWithLine(Point start, Point end);
    bool findNremovePoint(vector<Point> &list, Point point);

};

#endif // BOUSTROPHEDON_H
