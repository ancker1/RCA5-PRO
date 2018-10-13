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

class CircleDetection {
public:

	CircleDetection();
	~CircleDetection();

	vector<Vec3f>detectCircles(Mat& image);
	void         drawCircles(Mat            image,
													 vector<Vec3f>& circles);

private:
};

#endif // CIRCLEDETECTION_H
