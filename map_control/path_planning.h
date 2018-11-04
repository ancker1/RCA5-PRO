#ifndef PATH_PLANNING_H
#define PATH_PLANNING_H

#include <iostream>
#include <vector>
#include <math.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/ximgproc.hpp"

#include "Map.h"

using namespace std;
using namespace cv;
using namespace cv::ximgproc;

class Path_planning {
public:
    Path_planning();
    ~Path_planning();

    int way_around_obstacle(Point start, Point goal, Mat &src);

private:
    void go_left(Point &p, Mat &img);
    void go_right(Point &p, Mat &img);
    vector<Point> get_points(LineIterator &it);
    Point get_p_before_obstacle(vector<Point> &v, Mat &img);
    bool obstacle_detected(Point start, Point goal, Mat &img);
};

#endif // PATH_PLANNING_H
