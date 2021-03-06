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

#define ERASE_ABOVE					30 // [m]
#define TARGET_IMAGES_NO		10
#define TARGET_IMAGES_SCALE	0.1 // Apparently resize doesn't like 1 / TARGET_IMAGES_NO
#define MAP_ENLARGEMENT			1
#define M_TO_PIX						MAP_ENLARGEMENT * 10. / 7.
#define MARBLE_RADIUS_M			0.5f
#define MARBLE_RADIUS_P			MARBLE_RADIUS_M * M_TO_PIX
#define IMAGE_FOV						1.047f
#define TEMPLATE_MARGIN			10
#define TEMPLATE_THRESHOLD	0.8f

enum detection_algorithm {CD_HOUGH, CD_NOT_SPR, CD_SQUARE_FIT, CD_TEMPLATE_MATCHING};
enum circle_side {LEFT, RIGHT};

struct circleInfo {
	unsigned int	n = 1;
	int						x0, y0;
	float					r, angle, d;
	double				map_x, map_y;
	bool					prevspotted, spotted = true;
	Scalar				color;

	int						isSpotted(vector<circleInfo>& spottedCircles);
	void					update(circleInfo& spottedCircle);
	void					merge(circleInfo& circle);
};

struct targetImage {
	Mat						img;
	unsigned int	r;
	float					d;
};

class CircleDetection {
public:

	CircleDetection();
	~CircleDetection();

	vector<circleInfo>	detectCircles(Mat& image, detection_algorithm algo = CD_HOUGH);
	void								findBoundaries(vector<Point>& hull, Point& top, Point& right, Point& bottom, Point& left);
	void								drawCircles(Mat& image, vector<circleInfo>& circles);
	void								calcCirclePositions(vector<circleInfo>& spottedCircles, Mat& image, Mat& map, double x_robot, double y_robot, double angle_robot);
	void								mergeMarbles(vector<circleInfo>& circles, vector<circleInfo> spottedCircles);
	void								mapMarbles(Mat& map, vector<circleInfo>& g_circles, vector<circleInfo>& h_circles, vector<circleInfo>& tm_circles, vector<circleInfo>& circles, vector<circleInfo>& spottedCircles);
	void								error(vector<circleInfo>& spottedCircles, vector<circleInfo>& g_circles, double &error);
	void								avgerror(vector<circleInfo>& spottedCircles, vector<circleInfo>& g_circles, double &error, double &n);

private:
};

#endif // CIRCLEDETECTION_H
