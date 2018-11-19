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
	Rect								findBoundaries(vector<Point>& hull, Point& top);
	void								drawCircles(Mat& image, vector<circleInfo>& circles);
	void								calcCirclePosition(vector<circleInfo>& circles, int imagewidth);
	void								mapMarbles(Mat& map, int x_robot, int y_robot, float angle, vector<circleInfo>& circles);

private:
};

#endif // CIRCLEDETECTION_H
