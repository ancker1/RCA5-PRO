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

#include "Link.h"

using namespace std;
using namespace cv;
using namespace cv::ximgproc;

class Cellpoint
{
public:
    Cellpoint();
    Cellpoint(Point onCell);
    void addConnection(Link connection);
    Point getOnCell();
    vector<Link> getLinks();
    void removePointFromLinks(Point p); // Used only by astar
    double getHeuristicdist();
    double getCombinedHeuristic();
    void setCombinedHeuristic(double t);
    void calculateHeuristicdist(Point dist);
    ~Cellpoint();
protected:
    Point onCell;
    vector<Link> connectedTo;
    double heuristicdist; // Used by astar
    double combinedHeuristic; // Used by astar
};

#endif // CELLPOINT_H
