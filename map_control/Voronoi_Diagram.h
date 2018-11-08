#ifndef VORONI_DIAGRAM_H
#define VORONI_DIAGRAM_H

#include <iostream>
#include <vector>
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/ximgproc.hpp"

#include "Map.h"

using namespace std;
using namespace cv;

class Voronoi_Diagram
{
public:
    Voronoi_Diagram();
    Voronoi_Diagram(Mat &src);

    void printMap(Mat &img, string s);

    Mat get_brushfire_grid();
    Mat get_rooms_map();

private:
    Mat brushfire_grid;
    Mat rooms_map;

    void watershed_algorithm(Mat &src);
};

#endif // VORONI_DIAGRAM_H
