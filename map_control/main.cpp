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
    Mat img = src.clone();

    Voronoi_Diagram v_d( src );
    vector<Point> v = v_d.get_voronoi_points();
    for (unsigned i = 0; i < v.size(); i++)
        img.at<Vec3b>( v[i] ) = red;
    cvtColor(img, img, CV_BGR2GRAY);

    for (int y = 0; y < img.rows; y++) {
        for (int x = 0; x < img.cols; x++) {

        }
    }

    print_map( img, "Img" );

//    // Removing walls
//    for (int y = 0; y < img.rows; y++) {
//        for (int x = 0; x < img.cols; x++) {
//            if ( img.at<uchar>(y,x)==0 ) {
//                img.at<uchar>(y,x)=255;
//            }
//        }
//    }

//    // erosion and dilation
//    Mat kernel = getStructuringElement( MORPH_RECT, Size(2,2) );
//    Mat dst;
//    erode( img, dst, kernel, Point(-1,-1), 3 );
//    dilate( dst, img, kernel, Point(-1,-1), 3 );

    waitKey(0);
    return 0;
}
