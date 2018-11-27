#ifndef LINK_H
#define LINK_H

#include <iostream>

#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/ximgproc.hpp"

#include <math.h>

using namespace std;
using namespace cv;
using namespace cv::ximgproc;

class Link
{
public:
    Link();
    Link(Point connectedTo, char leftOrRight);

    Point getConnectedTo();
    double getDijkstraDist();
    char getLeftOrRight();

    void calculateDijkstra(Point cellPoint);


    ~Link();

protected:
    Point connectedTo;
    double dijkstraDist;
    char leftOrRight;

};

#endif // LINK_H
