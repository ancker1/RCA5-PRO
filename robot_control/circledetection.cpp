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

			//double				area;
			vector<Point>	hull;
			double				hull_area;
			//int						holes;
			double				ratio;
			double				circularity;

			for (int i = 0; 0 < contours.size() && 0 <= i; i = hierarchy[i][0]) {
				// AREA
				//area			= contourArea(contours[i]);
				/* Subtract holes
				for (int j = hierarchy[i][2]; 0 <= j; j = hierarchy[j][0]) {
					areas[i]		-= contourArea(contours[j]) - contours[j].size()/2 + 1;
				} */

				// CONVEX HULL
				convexHull(contours[i], hull);
				hull_area	= contourArea(hull);

				/*for (unsigned int j = 0; j < hull.size(); j++) {
					circle(image, hull[j], 1, Scalar(255 * j / hull.size(), 255 * j / hull.size(), 255 * j / hull.size()));
				}*/

				// HOLES
				/*for (int j = hierarchy[i][2]; 0 <= j; j = hierarchy[j][0]) {
					holes++;
				} */

				// Height/width ratio
				RotatedRect min_bounding_rect = minAreaRect(hull);
				ratio = min_bounding_rect.size.height / min_bounding_rect.size.width;

				// Circularity
				double perimeter_length = 0;
				#ifndef PI
				#define PI 3.14159265358979323846
				#endif

				// With contours
				/*for (unsigned int j = 0; j < contours[i].size(); j++) {
					perimeter_length	+= sqrt(pow(contours[i][j].x	- contours[i][(j + 1) % contours[i].size()].x,	2)	+ pow(contours[i][j].y - contours[i][(j + 1)	% contours[i].size()].y,	2));
				}
				circularity = 4*PI*area/pow(perimeter_length, 2);
				printf("%5.2f ", circularity); */

				for (unsigned int j = 0; j < hull.size(); j++) {
					double dis = sqrt(pow(hull[j].x - hull[(j + 1) % hull.size()].x, 2) + pow(hull[j].y - hull[(j + 1) % hull.size()].y, 2));
					perimeter_length += dis;
				}
				circularity = 4*PI*hull_area/pow(perimeter_length, 2);

				// Side showing
				int left_area = 0;
				int right_area = 0;
				circle_side side;

				for (int row = 0; row < min_bounding_rect.size.height; row++) {
					for (int col = 0; col < min_bounding_rect.size.width; col++) {
						if (image_filtered.at<uchar>(min_bounding_rect.boundingRect().y + row, min_bounding_rect.boundingRect().x + col) == 255) {
							col < min_bounding_rect.size.width / 2 ? left_area++ : right_area++;
						}
					}
				}
				left_area < right_area ? side = LEFT : side = RIGHT;

				// Is circle?
				if (0.9 <= circularity && circularity <= 1.1) {

					// ONE CIRCLE
					circlevector.push_back(circleInfo());
					circlevector[circlevector.size() - 1].r		= sqrt(hull_area / PI);
					circlevector[circlevector.size() - 1].x0	= min_bounding_rect.center.x;
					circlevector[circlevector.size() - 1].y0	= min_bounding_rect.center.y;

				} else if (0.9 * image_filtered.rows <= min_bounding_rect.size.height) {

					// CIRCLE CLOSE
					circlevector.push_back(circleInfo());
					circlevector[circlevector.size() - 1].x0	= min_bounding_rect.center.x;
					circlevector[circlevector.size() - 1].y0	= min_bounding_rect.center.y;

					// COULD BE BETTER
					circlevector[circlevector.size() - 1].r		= max(min_bounding_rect.size.height, min_bounding_rect.size.width) / 2;

				} else if (1 <= ratio && ratio <= 2 && 0.5 <= circularity) {

					// AT LEAST HALF CIRCLE
					circlevector.push_back(circleInfo());
					circlevector[circlevector.size() - 1].r		= min_bounding_rect.size.height / 2;
					circlevector[circlevector.size() - 1].y0	= min_bounding_rect.center.y;

					if (side == LEFT) {
						circlevector[circlevector.size() - 1].x0	= hull[0].x - min_bounding_rect.size.width + circlevector[circlevector.size() - 1].r;
					} else {
						circlevector[circlevector.size() - 1].x0	= hull[0].x - circlevector[circlevector.size() - 1].r;
					}
					
				} else if (2 <= ratio) {
					
					// LESS THAN HALF CIRCLE SHOWING
					circlevector.push_back(circleInfo());
					circlevector[circlevector.size() - 1].r		= min_bounding_rect.size.width / 2 + pow(min_bounding_rect.size.height, 2) / (8 * min_bounding_rect.size.width);
					circlevector[circlevector.size() - 1].y0	= min_bounding_rect.center.y;
					if (side == LEFT) {
						circlevector[circlevector.size() - 1].x0	= hull[0].x - min_bounding_rect.size.width + circlevector[circlevector.size() - 1].r;
					} else {
						circlevector[circlevector.size() - 1].x0	= hull[0].x - circlevector[circlevector.size() - 1].r;
					}

				} else if (ratio < 0.7 && 0.5 <= circularity) {

					// MORE THAN ONE CIRCLE
					circlevector.push_back(circleInfo());
					circlevector[circlevector.size() - 1].r		= min_bounding_rect.size.height / 2;
					circlevector[circlevector.size() - 1].y0	= min_bounding_rect.center.y;
					if (side == LEFT) {
						circlevector[circlevector.size() - 1].x0	= hull[0].x - min_bounding_rect.size.width + circlevector[circlevector.size() - 1].r;
					} else {
						circlevector[circlevector.size() - 1].x0	= hull[0].x - circlevector[circlevector.size() - 1].r;
					}

					circlevector.push_back(circleInfo());
					// COULD BE BETTER
					circlevector[circlevector.size() - 1].r		= (min_bounding_rect.size.width - min_bounding_rect.size.height) / 2;
					circlevector[circlevector.size() - 1].y0	= min_bounding_rect.center.y;
					if (side == LEFT) {
						circlevector[circlevector.size() - 1].x0	= hull[0].x - circlevector[circlevector.size() - 1].r;
					} else {
						circlevector[circlevector.size() - 1].x0	= hull[0].x - min_bounding_rect.size.width + circlevector[circlevector.size() - 1].r;
					}

				} else {

					// Not a Circle

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
	putText(image, format("Marbles: %d", circles.size()), Point(20, 20), FONT_HERSHEY_SIMPLEX, 0.4, Scalar(255, 255, 255), 1, LINE_AA);

	for (unsigned int i = 0; i < circles.size(); i++)
	{
		// Draw center and edge
		circle(image, Point(circles[i].x0, circles[i].y0), 1,							Scalar(255, 255, 255), 1, LINE_AA);
		circle(image, Point(circles[i].x0, circles[i].y0), circles[i].r,	Scalar(255, 255, 255), 1, LINE_AA);

		// Display angle and distance to marbles
		putText(image, format("%5.2f", circles[i].angle), Point(circles[i].x0 - 20, circles[i].y0 - circles[i].r - 20),
						FONT_HERSHEY_SIMPLEX, 0.4, Scalar(0, 0, 255), 1, LINE_AA);
		putText(image, format("%5.2f", circles[i].d), Point(circles[i].x0 - 20, circles[i].y0 - circles[i].r - 10),
						FONT_HERSHEY_SIMPLEX, 0.4, Scalar(0, 0, 255), 1, LINE_AA);
	}
}

void CircleDetection::calcCirclePosition(vector<circleInfo>& circles, int imagewidth) {
	float r     = 0.5;
	float theta = 1.047;

	for (unsigned int i = 0; i < circles.size(); i++) {
		circles[i].d			= r / tan(theta * circles[i].r / imagewidth);
		circles[i].angle	= theta * (0.5 - float(circles[i].x0) / imagewidth);
	}
}

void CircleDetection::mapMarbles(Mat& map, int x_robot, int y_robot, float angle, vector<circleInfo>& circles) {
	for (unsigned int i = 0; i < circles.size(); i++) {
		map.at<Vec3b>(x_robot + circles[i].d * cos(angle + circles[i].angle), y_robot + circles[i].d * sin(angle + circles[i].angle))[1] += 10;
	}
}
