#include "circledetection.h"

CircleDetection::CircleDetection() {}

CircleDetection::~CircleDetection() {}

vector<Vec3f>CircleDetection::detectCircles(Mat& image)
{
	Mat image_hls;

	cvtColor(image, image_hls, COLOR_BGR2HLS);

	// Isolate blue
	inRange(image_hls, Scalar(100, 20, 200), Scalar(140, 255, 255), image_hls);

	// Median blur to reduce noise and avoid false circle detection
	medianBlur(image_hls, image_hls, 5);

	// Vector with 3D float vectors
	vector<Vec3f> circles;

	// Apply HoughCircles
	HoughCircles(image_hls,           // Input image
							 circles,             // Vector with circle centers (a, b) and
																		// radii (r)
							 HOUGH_GRADIENT,      // Detection method (only one available)
							 1,                   // Inverse ratio of resolution (??)
							 image_hls.rows / 16, // Minimum distance between detected
																		// centers
							 100,                 // Canny edge detector upper threshold
							 10,                  // Center detection threshold
							 1,                   // Minimum radius of circles
							 30                   // Maximum radius of circles
							 );

	return circles;
}

void CircleDetection::drawCircles(Mat image, vector<Vec3f>& circles) {
	// For every circle (a, b and r)
	for (int i = 0; i < circles.size(); i++)
	{
		// Draw center and edge
		circle(image, Point(circles[i][0], circles[i][1]), 1,
					 Scalar(0, 0, 255), 1, LINE_AA);
		circle(image, Point(circles[i][0], circles[i][1]), circles[i][2],
					 Scalar(0, 0, 255), 2, LINE_AA);
	}

	// Display image
	imshow("Detected circles", image);
	waitKey();
}
