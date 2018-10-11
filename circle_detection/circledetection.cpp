#include "circledetection.h"

CircleDetection::CircleDetection() {}

CircleDetection::~CircleDetection() {}

vector<Vec3f>CircleDetection::detectCircles(Mat& image)
{
	// Convert image to grayscale
	Mat image_gray;

	cvtColor(image, image_gray, COLOR_BGR2GRAY);

	// Median blur to reduce noise and avoid false circle detection
	medianBlur(image_gray, image_gray, 5);

	// Vector with 3 dimensional float vectors
	vector<Vec3f> circles;

	// Apply HoughCircles
	HoughCircles(image_gray,           // Input image
							 circles,              // Vector with circle centers (a, b) and
																		 // radii (r)
							 HOUGH_GRADIENT,       // Detection method (only one available)
							 1,                    // Inverse ratio of resolution (??)
							 image_gray.rows / 16, // Minimum distance between detected
																		 // centers
							 100,                  // Canny edge detector upper threshold
							 30,                   // Center detection threshold
							 1,                    // Minimum radius of circles
							 30                    // Maximum radius of circles
							 );

	return circles;
}

void CircleDetection::drawCircles(Mat& image, vector<Vec3f>& circles) {
	// For every circle (a, b and r)
	for (int i = 0; i < circles.size(); i++)
	{
		// Draw center and edge
		circle(image, Point(circles[i][0], circles[i][1]), 1,
					 Scalar(0, 100, 100), 3, LINE_AA);
		circle(image, Point(circles[i][0], circles[i][1]), circles[i][2],
					 Scalar(255, 0, 255), 3, LINE_AA);
	}
	imshow("Detected circles", image);
	waitKey();
}
