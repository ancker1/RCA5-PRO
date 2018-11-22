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
#include "Zhang_suen.h"

using namespace std;
using namespace cv;

Vec3b red(0,0,255), black(0,0,0), white(255,255,255), blue(255,0,0);

void print_map(Mat &map, string s) {
    Mat resizeMap;
    resize( map, resizeMap, map.size()*10, 0, 0, INTER_NEAREST);
    imshow(s, resizeMap);
}

Mat skeletinize( cv::Mat &src ) {
    Mat gray, binary;
    cvtColor( src, gray, CV_BGR2GRAY );
    threshold(gray, binary, 127, 255, CV_THRESH_BINARY);

    Mat skel(binary.size(), CV_8UC1, Scalar(0)), temp, eroded, element;
    element = getStructuringElement(MORPH_CROSS, Size(3,3));
    bool done;

    do {
        erode( binary, eroded, element );
        dilate( eroded, temp, element );
        subtract( binary, temp, temp );
        bitwise_or( skel, temp, skel );
        eroded.copyTo( binary );
        done =( countNonZero(binary) == 0 );
    } while ( !done );

    for (int y = 0; y < skel.rows; y++) {
        for (int x = 0; x < skel.cols; x++) {
            skel.at<uchar>( y, 0 ) = 0;
            skel.at<uchar>( y, skel.cols-1 ) = 0;
            skel.at<uchar>( 0, x ) = 0;
            skel.at<uchar>( skel.rows-1, x ) = 0;

            if ( (int)skel.at<uchar>(y,x) != 0 ) {
                skel.at<uchar>(y,x) = 255;
            }
        }
    }

    return skel;
}

//void thinning_iteration( cv::Mat &img, int iter ) {
//    CV_Assert( img.channels() == 1 );
//    CV_Assert( img.depth() != sizeof(uchar) );
//    CV_Assert( img.rows > 3 && img.cols > 3 );

//    cv::Mat marker = cv::Mat::zeros( img.size(), CV_8UC1 );

//    int n_rows = img.rows;
//    int n_cols = img.cols;

//    if ( img.isContinuous() ) {
//        n_cols *= n_rows;
//        n_rows = 1;
//    }

//    int x, y;
//    uchar *p_above, *p_curr, *p_below;
//    uchar *nw, *no, *ne;    // North (p_above)
//    uchar *we, *me, *ea;
//    uchar *sw, *so, *se;    // South (p_below)

//    uchar *p_dst;

//    // Initialize row pointers
//    p_above = NULL;
//    p_curr = img.ptr<uchar>(0);
//    p_below = img.ptr<uchar>(1);

//    for (y = 1; y < img.rows-1; y++) {
//        // shift the rows up by one
//        p_above = p_curr;
//        p_curr = p_below;
//        p_below = img.ptr<uchar>(y+1);

//        p_dst = marker.ptr<uchar>(y);

//        // Initialize col pointers
//        no = &( p_above[0] );
//        ne = &( p_above[1] );
//        me = &( p_curr[0] );
//        ea = &( p_curr[1] );
//        so = &( p_below[0] );
//        se = &( p_below[0] );

//        for (x = 1; x < img.cols; x++) {
//            // shift col pointers left by one (scan left to right)
//            nw = no;
//            no = ne;
//            ne = &( p_above[x+1] );
//            we = me;
//            me = ea;
//            ea = &( p_curr[x+1] );
//            sw = so;
//            so = se;
//            se = &( p_below[x+1] );

//            int A = ( *ne == 0 && *ne == 1 ) + ( *ne == 0 && *ea == 1 ) +
//                    ( *ea == 0 && *se == 1 ) + ( *se == 0 && *so == 1 ) +
//                    ( *so == 0 && *sw == 1 ) + ( *sw == 0 && *we == 1 ) +
//                    ( *we == 0 && *nw == 1 ) + ( *nw == 0 && *no == 1 );
//            int B = *no + *ne + *ea + *se + *so + *sw + *we + *nw;
//            int m1 = ( iter == 0 ) ? ( *no * *ea * *so ) : ( *no * *ea * *we );
//            int m2 = ( iter == 0 ) ? ( *ea * *so * *we ) : ( *no * *so * *we );

//            if ( A == 1 && ( B >= 2 && B <= 6 ) && m1 == 0 && m2 == 0 )
//                p_dst[x] = 1;
//        }
//    }

//    img &= ~marker;
//}

//void thinning( const cv::Mat &src, cv::Mat &dst ) {
//    dst = src.clone();
//    dst /= 255;

//    cv::Mat prev = cv::Mat::zeros( dst.size(), CV_8UC1 );
//    cv::Mat diff;

//    do {
//        thinning_iteration( dst, 0 );
//        thinning_iteration( dst, 1 );
//        cv::absdiff( dst, prev, diff );
//        dst.copyTo( prev );
//    }
//    while ( cv::countNonZero( diff ) > 0 );

//    dst *= 255;
//}

int main() {
    const string big_map_filename = "../map_control/big_floor_plan.png";
    const string small_map_filename = "../map_control/floor_plan.png";

    Mat big_map = cv::imread( big_map_filename, IMREAD_COLOR );
    Mat small_map = cv::imread( small_map_filename, IMREAD_COLOR );

    Mat src = big_map.clone();
    Mat gray;
    cvtColor(src, gray, CV_BGR2GRAY);
    threshold(gray, gray, 127, 255, CV_THRESH_BINARY);
    Mat dst;
    thinning( gray, dst, THINNING_ZHANGSUEN );
    vector<Point> voronoi_points;
    for (int y = 0; y < dst.rows; y++) {
        for (int x = 0; x < dst.cols; x++) {
            dst.at<uchar>(y, 0) = 0;
            dst.at<uchar>(y, dst.cols-1) = 0;
            dst.at<uchar>(0, x) = 0;
            dst.at<uchar>(dst.rows-1, x) = 0;

            if ( (int)dst.at<uchar>(y,x) == 255 )
                voronoi_points.push_back( Point(x,y) );
        }
    }

    for ( auto& p : voronoi_points )
        src.at<Vec3b>( p ) = red;

    print_map( src, "Voronoi map" );

    waitKey(0);
    return 0;
}
