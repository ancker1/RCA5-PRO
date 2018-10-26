#include "circledetection.h"

CircleDetection::CircleDetection() {}

CircleDetection::~CircleDetection() {}

vector<Vec3f>CircleDetection::detectCircles(Mat& image)
{
	Mat image_filtered;

	// Isolate marble
	inRange(image, Scalar(80, 20, 20), Scalar(255, 200, 200), image_filtered);

	GaussianBlur(image_filtered, image_filtered, Size(9, 9), 2, 2);

	// Canny(image_filtered, image_filtered, 0, 0, 3);

	imshow("BLEH", image_filtered);

	// Vector with 3D float vectors
	vector<Vec3f> circles;

	// Apply HoughCircles
	HoughCircles(image_filtered,           // Input image
							 circles,                  // Vector with circle centers (a, b)
																				 // and radii (r)
							 HOUGH_GRADIENT,           // Detection method (only one
																				 // available)
							 1,                        // Inverse ratio of resolution (??)
							 image_filtered.rows / 30, // Minimum distance between detected
																				 // centers
							 10,                       // Canny edge detector upper threshold
							 25,                       // Center detection threshold
							 5,                        // Minimum radius of circles (0 =
																				 // UNKNOWN)
							 100                       // Maximum radius of circles (0 =
																				 // UNKNOWN)
							 );

	return circles;
}

void CircleDetection::drawCircles(Mat image, vector<Vec3f>& circles) {
	// For every circle (a, b and r)
	for (unsigned int i = 0; i < circles.size(); i++)
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
