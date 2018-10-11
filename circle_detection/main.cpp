#include "circledetection.h"
#include <string>
using std::string;

int main(int argc, char *argv[])
{
	Mat image = imread("../smarties.png", IMREAD_COLOR);
	CircleDetection cd;

	if (!image.data) {
		cout << "Error reading image";
		return 1;
	}

	vector<Vec3f> circles = cd.detectCircles(image);
	cd.drawCircles(image, circles);

	return 0;
}
