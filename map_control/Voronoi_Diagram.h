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

    vector<Point> get_points();
    Mat get_voroni_diagram(Mat &img);
    void printMap(Mat &img);

private:
    vector<Point> points;

    Mat1b get_local_max(Mat &brushfire_img);
    void find_voroni_points(Mat1b &img);
    void remove_points_in_corners(Mat &brushfire_img);
    void connect_voroni_points(Mat &brushfire_img);

    Mat create_binary_img(Mat &src);
    Mat distance_transform(Mat &binary_img);
};

#endif // VORONI_DIAGRAM_H
