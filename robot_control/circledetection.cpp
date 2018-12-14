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
				// Area: Contours
				/*area			= contourArea(contours[i]);
				// Subtract holes
				for (int j = hierarchy[i][2]; 0 <= j; j = hierarchy[j][0]) {
					areas[i]		-= contourArea(contours[j]) - contours[j].size()/2 + 1;
				} */

				// Area: Convex Hull
				convexHull(contours[i], hull);
				//hull_area	= contourArea(hull);

				/*for (unsigned int j = 0; j < hull.size(); j++) {
					circle(image, hull[j], 1, Scalar(255 * j / hull.size(), 255 * j / hull.size(), 255 * j / hull.size()));
				}*/

				// # of holes
				/*for (int j = hierarchy[i][2]; 0 <= j; j = hierarchy[j][0]) {
					holes++;
				} */

				// Ratio (Height/Width)
				Point top, right, bottom, left;
				findBoundaries(hull, top, right, bottom, left);
				Rect bounding_rect(left.x, top.y, right.x - left.x, bottom.y - top.y);
				if (bounding_rect.width < 2) continue;
				ratio = double(bounding_rect.height) / double(bounding_rect.width);

				// Circularity
				double perimeter_length = 0;
				#ifndef PI
				#define PI 3.14159265358979323846
				#endif

				// Circularity: Contours
				/*for (unsigned int j = 0; j < contours[i].size(); j++) {
					perimeter_length	+= sqrt(pow(contours[i][j].x	- contours[i][(j + 1) % contours[i].size()].x,	2)	+ pow(contours[i][j].y - contours[i][(j + 1)	% contours[i].size()].y,	2));
				}
				circularity = 4*PI*area/pow(perimeter_length, 2);
				printf("%5.2f ", circularity); */

				// Circularity: Hull
				for (unsigned int j = 0; j < hull.size(); j++) {
					double dis = sqrt(pow(hull[j].x - hull[(j + 1) % hull.size()].x, 2) + pow(hull[j].y - hull[(j + 1) % hull.size()].y, 2));
					perimeter_length += dis;
				}
				circularity = 4*PI*hull_area/pow(perimeter_length, 2);

				// Which side is showing
				double left_area = 0;
				double right_area = 0;
				circle_side side;

				// Sides area: Hull, not as precise
				/*bool bottom_halfway_visited	=	false;
				bool top_halfway_visited		=	false;
				vector<Point> hull_left;
				vector<Point> hull_right;

				for (unsigned int j = 0; j < hull.size(); j++) {
					if (!bottom_halfway_visited && hull[j].x < bounding_rect.x + bounding_rect.width / 2)													bottom_halfway_visited	= true;
					if (bottom_halfway_visited && !top_halfway_visited && bounding_rect.x + bounding_rect.width / 2 < hull[j].x)	top_halfway_visited			= true;

					if (hull[j].x == bounding_rect.x / 2 && bounding_rect.width % 2 == 1) continue;

					if			(!bottom_halfway_visited)	hull_right.push_back(hull[j]);
					else if	(!top_halfway_visited)		hull_left.push_back(hull[j]);
					else															hull_right.push_back(hull[j]);
				}

				right_area	= contourArea(hull_right);
				left_area	= contourArea(hull_left);*/

				// Sides area: Pixel-counting, computationally expensive
				for (int row = 0; row < bounding_rect.height; row++) {
					for (int col = 0; col < bounding_rect.width; col++) {
						if (image_filtered.at<uchar>(bounding_rect.y + row, bounding_rect.x + col) != 0) {
							if (col == bounding_rect.width && bounding_rect.width % 2 == 1) continue;
							col < bounding_rect.width / 2 ? left_area++ : right_area++;
						}
					}
				}
				left_area < right_area ? side = LEFT : side = RIGHT;

				// Analytical purposes
				/*putText(image, format("Left  = %5.1f",	left_area),		Point(50, 50), FONT_HERSHEY_SIMPLEX, 0.3, Scalar(255, 255, 255));
				putText(image, format("Right = %5.1f",	right_area),	Point(50, 60), FONT_HERSHEY_SIMPLEX, 0.3, Scalar(255, 255, 255));
				putText(image, format("Hull  = %5.1f",	hull_area),		Point(50, 70), FONT_HERSHEY_SIMPLEX, 0.3, Scalar(255, 255, 255));
				putText(image, format("Circ  = %5.2f",	circularity),	Point(50, 80), FONT_HERSHEY_SIMPLEX, 0.3, Scalar(255, 255, 255));
				putText(image, format("Ratio = %5.3f",	ratio),				Point(50, 90), FONT_HERSHEY_SIMPLEX, 0.3, Scalar(255, 255, 255));*/

				// Is circle?
				if ((0.95 <= circularity && circularity <= 1.01) || (0.9 <= ratio && ratio <= 1.1)) {

					// ONE CIRCLE
					circlevector.push_back(circleInfo());
					circlevector[circlevector.size() - 1].r		= sqrt((left_area + right_area) / PI);
					circlevector[circlevector.size() - 1].x0	= bounding_rect.x + bounding_rect.width / 2;
					circlevector[circlevector.size() - 1].y0	= bounding_rect.y + bounding_rect.height / 2;

				} else if (0.7 * image_filtered.rows <= bounding_rect.height) {

					// CIRCLE CLOSE
					circlevector.push_back(circleInfo());
					circlevector[circlevector.size() - 1].x0	= bounding_rect.x + bounding_rect.width / 2;
					circlevector[circlevector.size() - 1].y0	= bounding_rect.y + bounding_rect.height / 2;

					// COULD BE BETTER
					circlevector[circlevector.size() - 1].r		= max(bounding_rect.height, bounding_rect.width) / 2;

				} else if (ratio <= 1 && left_area + right_area < bounding_rect.width * bounding_rect.height * 4 / PI) {

					// MORE THAN ONE CIRCLE
					/*
					circlevector.push_back(circleInfo());
					circlevector[circlevector.size() - 1].r		= bounding_rect.height / 2;
					circlevector[circlevector.size() - 1].y0	= bounding_rect.y + bounding_rect.height / 2;
					circlevector[circlevector.size() - 1].x0	= top.x;
					*/

					// Hough Circle
					vector<Vec3f> circles;
					Mat slice;
					image_filtered(bounding_rect).copyTo(slice);
					GaussianBlur(slice, slice, Size(9, 9), 2, 2);
					HoughCircles(slice, circles, HOUGH_GRADIENT, 1, ceil(slice.rows / 2), 5, (slice.cols - slice.rows < 10 ? 10 : slice.cols - slice.rows), 0, slice.rows);
					if (1 < circles.size()) {
						for (unsigned int j = 0; j < circles.size(); j++) {
							circlevector.push_back(circleInfo());
							circlevector[circlevector.size() - 1].x0	= bounding_rect.x + (circles[j][0] < 0 ? 0 : (slice.cols < circles[j][0] ? slice.cols : circles[j][0]));
							circlevector[circlevector.size() - 1].y0	= bounding_rect.y + (circles[j][1] < 0 ? 0 : (slice.rows < circles[j][1] ? slice.rows : circles[j][1]));
							circlevector[circlevector.size() - 1].r		= bounding_rect.height < circles[j][2] ? bounding_rect.height : circles[j][2];
						}
					} else {
						circlevector.push_back(circleInfo());
						circlevector[circlevector.size() - 1].x0	= top.x;
						circlevector[circlevector.size() - 1].y0	= bounding_rect.y + bounding_rect.height / 2;
						circlevector[circlevector.size() - 1].r		= bounding_rect.height / 2;
					}

					/* For detecting the small circle as well when 2 are present (unprecise)
					if (side == LEFT) {
						circlevector[circlevector.size() - 1].x0	= hull[0].x - circlevector[circlevector.size() - 1].r;
					} else {
						circlevector[circlevector.size() - 1].x0	= bounding_rect.x + circlevector[circlevector.size() - 1].r;
					}

					circlevector.push_back(circleInfo());
					// COULD BE BETTER
					circlevector[circlevector.size() - 1].r		= (bounding_rect.width - bounding_rect.height) / 2;
					circlevector[circlevector.size() - 1].y0	= bounding_rect.y + bounding_rect.height / 2;
					if (side == LEFT) {
						circlevector[circlevector.size() - 1].x0	= bounding_rect.x + circlevector[circlevector.size() - 1].r;
					} else {
						circlevector[circlevector.size() - 1].x0	= hull[0].x - circlevector[circlevector.size() - 1].r;
					}*/

					//putText(image, format("Multiple (ratio: %5.2f)", ratio), Point(circlevector[circlevector.size() - 1].x0, circlevector[circlevector.size() - 1].y0), FONT_HERSHEY_SIMPLEX, 0.3, Scalar(255, 255, 255));
					//rectangle(image, bounding_rect, Scalar(0, 0, 255));

				} else if (ratio <= 2) {

					// AT LEAST HALF CIRCLE
					circlevector.push_back(circleInfo());
					circlevector[circlevector.size() - 1].r		= bounding_rect.height / 2;
					circlevector[circlevector.size() - 1].y0	= bounding_rect.y + bounding_rect.height / 2;

					if (side == LEFT) {
						circlevector[circlevector.size() - 1].x0	= bounding_rect.x + circlevector[circlevector.size() - 1].r;
					} else {
						circlevector[circlevector.size() - 1].x0	= bounding_rect.x + bounding_rect.width - circlevector[circlevector.size() - 1].r;
					}

					//putText(image, format("More"), Point(circlevector[circlevector.size() - 1].x0, circlevector[circlevector.size() - 1].y0), FONT_HERSHEY_SIMPLEX, 0.3, Scalar(255, 255, 255));
					
				} else if (2 <= ratio) {
					
					// LESS THAN HALF CIRCLE SHOWING
					circlevector.push_back(circleInfo());
					circlevector[circlevector.size() - 1].r		= bounding_rect.width / 2 + pow(bounding_rect.height, 2) / (8 * bounding_rect.width);
					circlevector[circlevector.size() - 1].y0	= bounding_rect.y + bounding_rect.height / 2;

					if (side == LEFT) {
						circlevector[circlevector.size() - 1].x0	= bounding_rect.x + circlevector[circlevector.size() - 1].r;
					} else {
						circlevector[circlevector.size() - 1].x0	= bounding_rect.x + bounding_rect.width - circlevector[circlevector.size() - 1].r;
					}

					//putText(image, format("Less"), Point(circlevector[circlevector.size() - 1].x0, circlevector[circlevector.size() - 1].y0), FONT_HERSHEY_SIMPLEX, 0.3, Scalar(255, 255, 255));

				} else {

					// Not a Circle

				}
			}
		}
		break;

	default:;
	}

	calcCirclePositions(circlevector, image.cols);

	return circlevector;
}

void CircleDetection::findBoundaries(vector<Point>& hull, Point& top, Point& right, Point& bottom, Point& left) {
	top.y			= INT_MAX;
	right.x		= 0;
	bottom.y	= 0;
	left.x		= INT_MAX;

	for (unsigned int i = 0; i < hull.size(); i++) {
		if (hull[i].x < left.x)		left		= hull[i];
		if (right.x < hull[i].x)	right		= hull[i];
		if (hull[i].y < top.y)		top			= hull[i];
		if (bottom.y < hull[i].y) bottom	= hull[i];
	}
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

void CircleDetection::calcCirclePositions(vector<circleInfo>& circles, int imagewidth) {
	const float r = 0.5, theta = 1.047;

	for (unsigned int i = 0; i < circles.size(); i++) {
		circles[i].d			= r / tan(theta * circles[i].r / imagewidth);
		circles[i].angle	= theta * (0.5 - float(circles[i].x0) / imagewidth);
	}
}

void CircleDetection::mapMarbles(Mat& map, double x_robot, double y_robot, double angle_robot, vector<circleInfo>& circles, int (&detections)[4]) {
	const double mtopix = MAP_ENLARGEMENT * 10 / 7;
	const double radius = 0.5 * mtopix;
	const int h_shift = MAP_ENLARGEMENT * 80 / 2;
	const int w_shift = MAP_ENLARGEMENT * 120 / 2;
	Vec3b* current_pixel;
	unsigned char certainty = 255;

	for (unsigned int i = 0; i < circles.size(); i++) {
		certainty = ceil(2 / sqrt(circles[i].d));
		current_pixel = &map.at<Vec3b>(Point(w_shift + mtopix * (circles[i].d * cos(angle_robot + circles[i].angle) + x_robot), h_shift - mtopix * (circles[i].d * sin(angle_robot + circles[i].angle) + y_robot)));
		if (current_pixel->val[1] == 255 || current_pixel->val[0] + certainty <= 255) {
			*current_pixel = circles[i].d <= 10 ? Vec3b(current_pixel->val[0] + certainty, 0, 0) : Vec3b(0, 0, current_pixel->val[0] + certainty);
		}
		detections[int(circles[i].d < 100 ? circles[i].d / 10 : 10)]++;
		/*if (circles[i].d <= 10) {
			circle(map, Point(w_shift + mtopix * (circles[i].d * cos(angle_robot + circles[i].angle) + x_robot), h_shift - mtopix * (circles[i].d * sin(angle_robot + circles[i].angle) + y_robot)), radius, Scalar(certainty, 0, 0));
		}*/
	}

	//map.at<Vec3b>(Point(w_shift + mtopix * x_robot, h_shift - mtopix * y_robot)) = Vec3b(0, 0, 255);
}
