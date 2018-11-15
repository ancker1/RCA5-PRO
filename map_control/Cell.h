#ifndef CELL_H
#define CELL_H

#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/ximgproc.hpp"

#include "Cellpoint.h"
#include <vector>

using namespace std;
using namespace cv;
using namespace cv::ximgproc;

class Cell
{
public:
    Cell();
    Cell(Cellpoint);
    vector<Point> getCellPointLeft();
    vector<Point> getCellPointRight();
    Point getCellPointOnCell();
    vector<Cellpoint> getAllCellPoints(); // Only used by same x
    void addCellPoint(Cellpoint); // Only used by same x

    ~Cell();
protected:
    Cellpoint cellpoint;
    vector<Cellpoint> allCellPoints; // Only used by same x

};

#endif // CELL_H
