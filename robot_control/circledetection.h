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

struct circleInfo {
	float x0, y0, r, angle, d;
};

class CircleDetection {
public:

	CircleDetection();
	~CircleDetection();

	vector<circleInfo>	detectCircles(Mat& image, detection_algorithm algo = CD_HOUGH);
	void								drawCircles(Mat& image, vector<circleInfo>& circles);
	void								calcCirclePosition(vector<circleInfo>& circles, int imagewidth);

private:
};

#endif // CIRCLEDETECTION_H
