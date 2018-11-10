#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/ximgproc.hpp"

#include <iostream>
#include "Map.h"
#include "path_planning.h"
#include "Voronoi_Diagram.h"

using namespace std;
using namespace cv;

void print_map(Mat &map, string s) {
    Mat resizeMap;
    resize( map, resizeMap, map.size()*10, 0, 0, INTER_NEAREST);
    imshow(s, resizeMap);
}

int main() {
    const string big_map_filename = "../map_control/big_floor_plan.png";
    const string small_map_filename = "../map_control/floor_plan.png";

    Mat big_map = cv::imread( big_map_filename, IMREAD_COLOR );
    Mat small_map = cv::imread( small_map_filename, IMREAD_COLOR );

    Map map;
    Mat img = map.brushfire_img(small_map);

    Voronoi_Diagram v_diagram(small_map);
    Mat brushfire_img = v_diagram.get_brushfire_grid();

    Mat dx, dy;
    Sobel(brushfire_img, dx, CV_32F, 1, 0);
    Sobel(brushfire_img, dy, CV_32F, 0, 1);

    Mat angle(big_map.size(), big_map.type());
    Mat mag(big_map.size(), big_map.type());
    cartToPolar(dx, dy, mag, angle);

    Mat dst;
    convertScaleAbs(mag, dst);
    dst = ( dst==img );
    print_map(dst, "Map");

    dst = ( dst+mag );
    print_map(dst, "Map2");
    cout << dst << endl;

    waitKey(0);
    return 0;
}
