#ifndef CELLPOINT_H
#define CELLPOINT_H

#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/ximgproc.hpp"
#include <vector>

using namespace std;
using namespace cv;
using namespace cv::ximgproc;

class Cellpoint
{
public:
    Cellpoint();
    Cellpoint(Point onCell);
    void setPointLeft(Point leftCell);
    void setPointRight(Point rightCell);
    vector<Point> getPointLeft();
    vector<Point> getPointRight();
    void removePointLeft(int space);
    void removePointRight(int space);
    Point getOnCell();

    double getHeuristicdist();
    double getCurrentdist();
    void setHeuristicdist(double dist);
    void setCurrentdist(double dist);
    ~Cellpoint();
protected:
    Point onCell;
    vector<Point> connectedToLeft;
    vector<Point> connectedToRight;

    double heuristicdist; // Used by astar
    double currentdist; // Used by astar
};

#endif // CELLPOINT_H
