#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/ximgproc.hpp"

#include <iostream>
#include <vector>

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

void thinning_iteration( cv::Mat &img, int iter ) {
    cv::Mat marker = cv::Mat::zeros( img.size(), CV_8UC1 );
    for (int y = 1; y < img.rows; y++)
        for (int x = 1; x < img.cols; x++) {
            int p9 = (int)img.at<uchar>( y-1, x-1 );
            int p2 = (int)img.at<uchar>( y-1, x );
            int p3 = (int)img.at<uchar>( y-1, x+1 );
            int p4 = (int)img.at<uchar>( y, x+1 );
            int p5 = (int)img.at<uchar>( y+1, x+1 );
            int p6 = (int)img.at<uchar>( y+1, x );
            int p7 = (int)img.at<uchar>( y+1, x-1 );
            int p8 = (int)img.at<uchar>( y, x-1 );

            int A  = (p2 == 0 && p3 == 1) + (p3 == 0 && p4 == 1) +
                     (p4 == 0 && p5 == 1) + (p5 == 0 && p6 == 1) +
                     (p6 == 0 && p7 == 1) + (p7 == 0 && p8 == 1) +
                     (p8 == 0 && p9 == 1) + (p9 == 0 && p2 == 1);

            int B  = p2 + p3 + p4 + p5 + p6 + p7 + p8 + p9;

            int m1 = iter == 0 ? (p2 * p4 * p6) : (p2 * p4 * p8);
            int m2 = iter == 0 ? (p4 * p6 * p8) : (p2 * p6 * p8);

            if (A == 1 && (B >= 2 && B <= 6) && m1 == 0 && m2 == 0)
                marker.at<uchar>(y,x) = 1;
        }
    img &= ~marker;
}

void thinning_img( cv::Mat &img ) {
    img /= 255;
    cv::Mat prev = cv::Mat::zeros( img.size(), CV_8UC1 );
    cv::Mat diff;
    do {
        thinning_iteration( img, 0 );
        thinning_iteration( img, 1 );
        cv::absdiff( img, prev, diff );
        img.copyTo( prev );
    }
    while ( cv::countNonZero( diff ) > 0 );

    img *= 255;
}

int main() {
    const string big_map_filename = "../map_control/big_floor_plan.png";
    const string small_map_filename = "../map_control/floor_plan.png";

    Mat big_map = cv::imread( big_map_filename, IMREAD_COLOR );
    Mat small_map = cv::imread( small_map_filename, IMREAD_COLOR );

    Mat src = big_map.clone();
    Mat gray;
    cv::cvtColor( src, gray, CV_BGR2GRAY );
    cv::threshold( gray, gray, 10, 255, CV_THRESH_BINARY );
    thinning_img( gray );
    for (int y = 0; y < gray.rows; y++) {
        gray.at<uchar>(y, 0) = 0;
        gray.at<uchar>(y, gray.cols-1) = 0;
    }
    for (int x = 0; x < gray.cols; x++) {
        gray.at<uchar>(0, x) = 0;
        gray.at<uchar>(gray.rows-1, x) = 0;
    }
    print_map( gray, "Map" );

    waitKey(0);
    return 0;
}
