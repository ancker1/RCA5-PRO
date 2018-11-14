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

Vec3b red(0,0,255), black(0,0,0), white(255,255,255);

void print_map(Mat &map, string s) {
    Mat resizeMap;
    resize( map, resizeMap, map.size()*10, 0, 0, INTER_NEAREST);
    imshow(s, resizeMap);
}

bool obstacle_detected( Mat &img, Point start, Point goal ) {
    LineIterator it(img, start, goal, 8);
    vector<Point> v(it.count);
    for (int i = 0; i < it.count; i++, it++)
        v[i] = it.pos();

    for (unsigned i = 0; i < v.size(); i++) {
        if ( (int)img.at<uchar>( v[i] )==0 )
            return true;
    }

    return false;
}

void center_to_center( Mat &img, Point start) {
    for (int y = start.y; y < img.rows; y++) {
        for (int x = 0; x < img.cols; x++) {

        }
    }
}

int main() {
    const string big_map_filename = "../map_control/big_floor_plan.png";
    const string small_map_filename = "../map_control/floor_plan.png";

    Mat big_map = cv::imread( big_map_filename, IMREAD_COLOR );
    Mat small_map = cv::imread( small_map_filename, IMREAD_COLOR );

    Mat img = big_map.clone();
    Map map;
    Mat brushfire = map.brushfire_img(img);
    vector<Point> v = map.find_centers(brushfire);
    for (unsigned i = 0; i < v.size(); i++) {
        img.at<Vec3b>( v[i] ) = red;
    }
    for (unsigned i = 0; i < v.size(); i++) {
        center_to_center(img, v[i]);
    }

    print_map(img, "Map");


//    Mat binary, gray;
//    cvtColor(img, gray, CV_RGB2GRAY);
//    threshold(gray, binary, 128.0, 255.0, THRESH_BINARY);
//    binary = binary > 128;
//    print_map(binary, "Binary");
//    vector<Point> walls;
//    for (int y = 0; y < binary.rows; y++) {
//        for (int x = 0; x < binary.cols; x++) {
//            if ( (int)binary.at<uchar>(y,x)==0 ) {
//                walls.push_back( Point(x,y) );
//            }
//        }
//    }
//    Map map;
//    Mat brushfire = map.brushfire_img(img);
//    Mat1b kernel1( Size(5,5), 1u );
//    Mat b_dilate;
//    dilate(brushfire, b_dilate, kernel1);
//    Mat1b local_max = ( brushfire>=b_dilate);
//    bitwise_not(local_max, local_max);
//    for (unsigned i = 0; i < walls.size(); i++) {
//        local_max.at<uchar>( walls[i] ) = 255;
//    }
//    for (int i = 0; i < local_max.cols; i++) {
//        local_max.at<uchar>(0, i) = 255;
//        local_max.at<uchar>(local_max.rows-1, i) = 255;
//    }
//    for (int i = 0; i < local_max.rows; i++) {
//        local_max.at<uchar>(i, 0) = 255;
//        local_max.at<uchar>(i, local_max.cols-1) = 255;
//    }
//    bitwise_not(local_max, local_max);
//    print_map(local_max, "Map");

    waitKey(0);
    return 0;
}
