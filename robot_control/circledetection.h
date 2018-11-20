#ifndef CIRCLEDETECTION_H
#define CIRCLEDETECTION_H

#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
using namespace cv;

#include <vector>
using std::vector;

#include <iostream>
using std::cout;
using std::endl;

#include <cmath>

#define MAP_ENLARGEMENT 10

enum detection_algorithm {CD_HOUGH, CD_SPR};
enum circle_side {LEFT, RIGHT};

struct circleInfo {
	int		x0, y0;
	float	r, angle, d;
};

class CircleDetection {
public:

	CircleDetection();
	~CircleDetection();

	vector<circleInfo>	detectCircles(Mat& image, detection_algorithm algo = CD_HOUGH);
	void								findBoundaries(vector<Point>& hull, Point& top, Point& right, Point& bottom, Point& left);
	void								drawCircles(Mat& image, vector<circleInfo>& circles);
	void								calcCirclePositions(vector<circleInfo>& circles, int imagewidth);
	void								mapMarbles(Mat& map, double x_robot, double y_robot, double angle_robot, vector<circleInfo>& circles);

private:
};

#endif // CIRCLEDETECTION_H
