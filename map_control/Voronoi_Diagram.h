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

    void print_map(Mat &img, string s);

    Mat get_brushfire_grid();
    Mat get_rooms_map();
    Mat get_binary_img();
    Mat get_mag_img();
    vector<Point> get_voronoi_points();

private:
    Mat source;
    Mat brushfire_grid;
    Mat mag_img;
    Mat voronoi_diagram_img;
    Mat rooms_map;
    Mat binary_img;
    vector<Point> voronoi_points;

    void watershed_algorithm(Mat &src);
    void make_voronoi_points();
};

#endif // VORONI_DIAGRAM_H
