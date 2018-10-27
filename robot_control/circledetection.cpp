#include "circledetection.h"

CircleDetection::CircleDetection() {}

CircleDetection::~CircleDetection() {}

vector<circleInfo>CircleDetection::detectCircles(Mat& image)
{
	Mat image_filtered;

	// Isolate marble
	inRange(image, Scalar(80, 20, 20), Scalar(255, 200, 200), image_filtered);

	GaussianBlur(image_filtered, image_filtered, Size(9, 9), 2, 2);

	// Canny(image_filtered, image_filtered, 0, 0, 3);
	// imshow("BLEH", image_filtered);

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

	vector<circleInfo> circlevector;
	float r     = 0.5;
	float theta = 1.047;

	for (unsigned int i = 0; i < circles.size(); i++) {
		circlevector.push_back(circleInfo());
		circlevector[i].x0 = circles[i][0];
		circlevector[i].y0 = circles[i][1];
		circlevector[i].r  = circles[i][2];

		circlevector[i].angle = theta * (image.cols - circlevector[i].x0) /
														image.cols - theta / 2;
		circlevector[i].d = r / tan((circlevector[i].r * 2 / image.cols) * theta);
	}

	return circlevector;
}

void CircleDetection::drawCircles(Mat image, vector<circleInfo>& circles) {
	// For every circle (a, b and r)
	for (unsigned int i = 0; i < circles.size(); i++)
	{
		// Draw center and edge
		circle(image, Point(circles[i].x0, circles[i].y0), 1,
					 Scalar(0, 0, 255), 1, LINE_AA);
		circle(image, Point(circles[i].x0, circles[i].y0), circles[i].r,
					 Scalar(0, 0, 255), 2, LINE_AA);
		putText(image,
						std::to_string(circles[i].angle),
						Point(circles[i].x0 - circles[i].r,
									circles[i].y0 - circles[i].r - 20),
						FONT_HERSHEY_SIMPLEX,
						0.4,
						Scalar(0, 0, 255),
						1,
						LINE_AA);
		putText(image,
						std::to_string(circles[i].d),
						Point(circles[i].x0 - circles[i].r,
									circles[i].y0 - circles[i].r - 10),
						FONT_HERSHEY_SIMPLEX,
						0.4,
						Scalar(0, 0, 255),
						1,
						LINE_AA);
	}
}
