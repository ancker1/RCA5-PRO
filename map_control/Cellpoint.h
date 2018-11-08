#ifndef CELLPOINT_H
#define CELLPOINT_H

#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/ximgproc.hpp"

using namespace std;
using namespace cv;
using namespace cv::ximgproc;

class Cellpoint
{
public:
    Cellpoint();
    Cellpoint(Point onCell);
    void setPointLeft(Point rightCell);
    void setPointRight(Point rightCell);
    Point getPointLeft();
    Point getPointRight();
    Point getOnCell();

protected:
    Point onCell;
    Point connectedTo[2];
};

#endif // CELLPOINT_H
