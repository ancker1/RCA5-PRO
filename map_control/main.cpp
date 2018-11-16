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

Vec3b red(0,0,255), black(0,0,0), white(255,255,255), blue(255,0,0);

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

    Mat src = big_map.clone();
    Voronoi_Diagram v_d(src);

    Mat gray;
    cvtColor( src, gray, CV_BGR2GRAY );
    Mat binary;
    threshold(gray, binary, 127, 255, CV_THRESH_BINARY);
    Mat skel( binary.size(), CV_8UC1, Scalar(0));
    Mat temp;
    Mat eroded;
    Mat element = getStructuringElement( MORPH_CROSS, Size(3,3) );
    bool done;
    do {
        erode( binary, eroded, element );
        dilate( eroded, temp, element );
        subtract( binary, temp, temp );
        bitwise_or( skel, temp, skel );
        eroded.copyTo( binary );
        done =( countNonZero(binary) == 0 );
    } while ( !done );

    print_map(skel, "Skel");

    waitKey(0);
    return 0;
}
