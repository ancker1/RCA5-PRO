#include "circledetection.h"

int main(int argc, char **argv)
{
	Mat image = imread(
		"/home/seht97/Desktop/RCA5-PRO/circle_detection/sample2.png",
		IMREAD_COLOR);

	if (!image.data) {
		cout << "Error reading image" << endl;
		return 1;
	}

	CircleDetection    cd;
	vector<circleInfo> circles = cd.detectCircles(image);
	cd.drawCircles(image, circles);

	// Display image
	imshow("Detected circles", image);
	waitKey();

	return 0;
}
