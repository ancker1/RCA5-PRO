#include "circledetection.h"

CircleDetection::CircleDetection() {}

CircleDetection::~CircleDetection() {}

vector<circleInfo>CircleDetection::detectCircles(Mat& image, detection_algorithm algo)
{
	Mat image_filtered;

	// Isolate marble
	inRange(image, Scalar(0, 0, 40), Scalar(80, 90, 160), image_filtered);
	//inRange(image, Scalar(40, 0, 0), Scalar(160, 90, 80), image_filtered);

	vector<circleInfo> circlevector;

	switch (algo) {
	case CD_HOUGH:
		{
			GaussianBlur(image_filtered, image_filtered, Size(9, 9), 2, 2);

			// Vector with 3D float vectors
			vector<Vec3f> circles;

			// Apply HoughCircles
			HoughCircles(image_filtered,						// Input image
									 circles,										// Vector with circle centers (a, b) and radii (r)
									 HOUGH_GRADIENT,						// Detection method (only one available)
									 1,													// Inverse ratio of resolution (??)
									 image_filtered.cols / 15,	// Minimum distance between detected centers
									 5,													// Canny edge detector upper threshold
									 25,												// Center detection threshold
									 0,													// Minimum radius of circles (0 = UNKNOWN)
									 0													// Maximum radius of circles (0 = UNKNOWN)
									 );

			for (unsigned int i = 0; i < circles.size(); i++) {
				circlevector.push_back(circleInfo());
				circlevector[i].x0		= circles[i][0];
				circlevector[i].y0		= circles[i][1];
				circlevector[i].r			= circles[i][2];
			}
		}
		break;

	case CD_SPR:
		{
			vector<vector<Point>>	contours;
			vector<Vec4i>					hierarchy;
			findContours(image_filtered, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_NONE);

			vector<double>				areas(contours.size());
			vector<vector<Point>>	hulls(contours.size());
			vector<double>				hull_areas(contours.size());
			vector<int>						holes(contours.size());
			vector<double>				ratio(contours.size());
			vector<double>				circularities(contours.size());

			for (int i = 0; 0 <= i && contours.size() > 0; i = hierarchy[i][0]) {
				// AREA
				areas[i]			= contourArea(contours[i]);
				/* Subtract holes
				for (int j = hierarchy[i][2]; 0 <= j; j = hierarchy[j][0]) {
					areas[i]		-= contourArea(contours[j]) - contours[j].size()/2 + 1;
				} */

				// CONVEX HULL
				convexHull(contours[i], hulls[i]);
				hull_areas[i]	= contourArea(hulls[i]) + hulls[i].size()/2 + 1;

				// HOLES
				/*for (int j = hierarchy[i][2]; 0 <= j; j = hierarchy[j][0]) {
					holes[i]++;
				} */

				// Height/width ratio
				RotatedRect min_bounding_rect = minAreaRect(hulls[i]);
				ratio[i] = min_bounding_rect.size.height / min_bounding_rect.size.width;

				// Circularity
				double perimeter_length = 0;
				#define PI 3.14159265358979323846

				// With contours
				/*for (unsigned int j = 0; j < contours[i].size(); j++) {
					perimeter_length	+= sqrt(pow(contours[i][j].x	- contours[i][(j + 1) % contours[i].size()].x,	2)	+ pow(contours[i][j].y - contours[i][(j + 1)	% contours[i].size()].y,	2));
				}
				circularities[i] = 4*PI*areas[i]/pow(perimeter_length, 2);
				printf("%5.2f ", circularities[i]); */

				for (unsigned int j = 0; j < hulls[i].size(); j++) {
					perimeter_length += sqrt(pow(hulls[i][j].x - hulls[i][(j + 1) % hulls[i].size()].x, 2) + pow(hulls[i][j].y - hulls[i][(j + 1) % hulls[i].size()].y, 2));
				}
				circularities[i] = 4*PI*hull_areas[i]/pow(perimeter_length, 2);

				// Is circle?
				if (0.6 <= circularities[i]) {
					if (0.9 <= circularities[i] && circularities[i] <= 1.1 && hull_areas[i] <= areas[i]) {

						// ONE CIRCLE
						circlevector.push_back(circleInfo());
						circlevector[circlevector.size() - 1].x0	= min_bounding_rect.center.x;
						circlevector[circlevector.size() - 1].y0	= min_bounding_rect.center.y;
						circlevector[circlevector.size() - 1].r		= min_bounding_rect.size.height / 2;

					} else if (1 <= ratio[i] && ratio[i] <= 2) {

						// AT LEAST HALF CIRCLE
						circlevector.push_back(circleInfo());
						circlevector[circlevector.size() - 1].x0	= min_bounding_rect.center.x - min_bounding_rect.size.width / 2 + min_bounding_rect.size.height / 2;
						circlevector[circlevector.size() - 1].y0	= min_bounding_rect.center.y;
						circlevector[circlevector.size() - 1].r		= min_bounding_rect.size.height / 2;

					} else if (ratio[i] <= 1 && areas[i] <= hull_areas[i]) {
						// MORE THAN ONE CIRCLE
					} else if (float(image_filtered.rows) * 0.8 <= min_bounding_rect.size.height) {
					} else {
						// MAYBE CIRCLE?
					}
				} else {
					// NO CIRCLE
				}
			}

			// For analytical purposes
			/*printf(" i        Area      Area2 Holes  Rect  Circ\n");
			for (unsigned int i = 0; i < contours.size(); i++) {
				printf("%2d: %10.1f %10.1f %5d %5.2f %5.2f\n", i, areas[i], hull_areas[i], holes[i], ratio[i], circularities[i]);
			}*/
		}
		break;

	default:;
	}

	calcCirclePosition(circlevector, image.cols);

	return circlevector;
}

void CircleDetection::drawCircles(Mat& image, vector<circleInfo>& circles) {
	putText(image, format("Marbles: %d", circles.size()), Point(10, 10), FONT_HERSHEY_SIMPLEX, 0.4, Scalar(0, 0, 255), 1, LINE_AA);

	for (unsigned int i = 0; i < circles.size(); i++)
	{
		// Draw center and edge
		circle(image, Point(circles[i].x0, circles[i].y0), 1,							Scalar(0, 0, 255), 1, LINE_AA);
		circle(image, Point(circles[i].x0, circles[i].y0), circles[i].r,	Scalar(0, 0, 255), 1, LINE_AA);

		// Display angle and distance to marbles
		putText(image, format("%8.3f", circles[i].angle), Point(circles[i].x0 - circles[i].r, circles[i].y0 - circles[i].r - 20),
						FONT_HERSHEY_SIMPLEX, 0.4, Scalar(0, 0, 255), 1, LINE_AA);
		putText(image, format("%8.3f", circles[i].d), Point(circles[i].x0 - circles[i].r, circles[i].y0 - circles[i].r - 10),
						FONT_HERSHEY_SIMPLEX, 0.4, Scalar(0, 0, 255), 1, LINE_AA);
	}
}

void CircleDetection::calcCirclePosition(vector<circleInfo>& circles, int imagewidth) {
	float r     = 0.5;
	float theta = 1.047;

	for (unsigned int i = 0; i < circles.size(); i++) {
		circles[i].d			= r / tan(theta * circles[i].r / imagewidth);
		circles[i].angle	= theta * (0.5 - circles[i].x0 / imagewidth);
	}
}
