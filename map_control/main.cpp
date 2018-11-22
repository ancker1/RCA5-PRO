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

template<class _T>
static inline cv::Rect boundingBox( const cv::Mat_<_T> &img )
{
    assert( img.isContinuous() );
    int x_min = 0, y_min = 0, x_max = 0, y_max = 0;
    bool was_init = false;
    const _T* img_it = img.ptr(0);

    for (int y = 0; y < img.rows; y++) {
        for (int x = 0; x < img.cols; x++) {

            if ( *img_it++ ) {

                if ( !was_init ) {
                    x_min = x_max = x;
                    y_min = y_max = y;
                    was_init = true;
                    continue;
                }

                if ( x < x_min )
                    x_min = x;
                else if ( x > x_max )
                    x_max = x;

                if ( y < y_min )
                    y_min = y;
                else if ( y > y_max )
                    y_max = y;
            }
        }
    }

    if ( !was_init )
        return cv::Rect( -1, -1, -1, -1 );

    return cv::Rect( x_min, y_min, 1+x_max-x_min, 1+y_max-y_min );
}

static inline cv::Rect copy_bounding_box_plusone( const cv::Mat1b &img,
                                                  cv::Mat1b &out,
                                                  bool crop_img_before = true) {
    if ( !crop_img_before ) {
        img.copyTo( out );
        return cv::Rect( 0, 0, img.cols, img.rows );
    }

    cv::Rect bbox = boundingBox( img );

    if ( bbox.x <= 0 || bbox.x + bbox.width >= img.cols ||
         bbox.y <= 0 || bbox.y + bbox.height >= img.rows ) {
        out.create( img.size() );
        out.setTo( 0 );
        cv::Mat1b out_roi = out( cv::Rect( 1, 1, img.cols-2, img.rows-2 ) );
        cv::resize( img, out_roi, out_roi.size(), 0, 0, cv::INTER_NEAREST );
        return cv::Rect( 0, 0, img.cols, img.rows );
    }

    bbox.x--; bbox.y--; bbox.width += 2; bbox.height += 2;
    img( bbox ).copyTo( out );
    return bbox;
}

void thin_zhang_suen_original_iter( cv::Mat &img, int iter ) {
    cv::Mat marker = cv::Mat::zeros( img.size(), CV_8UC1 );
    for (int y = 1; y < img.rows; y++) {
        for (int x = 1; x < img.cols; x++) {
            uchar p2 = img.at<uchar>( y-1, x );     // up
            uchar p3 = img.at<uchar>( y-1, x+1 );   // up right
            uchar p4 = img.at<uchar>( y, x+1 );     // right
            uchar p5 = img.at<uchar>( y+1, x+1 );   // down right
            uchar p6 = img.at<uchar>( y+1, x );     // down
            uchar p7 = img.at<uchar>( y+1, x-1 );   // down left
            uchar p8 = img.at<uchar>( y, x-1 );     // left
            uchar p9 = img.at<uchar>( y-1, x-1 );   // Up left

            int A = ( p2==0 && p3==1 ) +
                    ( p3==0 && p4==1 ) +
                    ( p4==0 && p5==1 ) +
                    ( p5==0 && p6==1 ) +
                    ( p6==0 && p7==1 ) +
                    ( p7==0 && p8==1 ) +
                    ( p8==0 && p9==1 ) +
                    ( p9==0 && p2==1 );
            int B = p2 + p3 + p4 + p5 + p6 + p7 + p8 + p9;
            int m1 = ( iter == 0 ? ( p2 * p4 * p6 ) : ( p2 * p4 * p8 ) );
            int m2 = ( iter == 0 ? ( p4 * p6 * p8 ) : ( p2 * p6 * p8 ) );

            if ( A==1 && ( B>=2 && B<=6 ) && m1==0 && m2==0 ) {
                marker.at<uchar>(y,x) = 1;
            }
        }
    }

    img = ~marker;
}

bool thin_zhang_suen_original( const cv::Mat &img,
                               bool crop_img_before = true,
                               int max_iters = 25) {

    cv::Mat1b temp, skel, prev, diff;
    cv::Rect _bbox;

    cv::threshold( img, temp, 10, 1, CV_THRESH_BINARY );
    _bbox = copy_bounding_box_plusone( temp, skel, crop_img_before );

    int n_iters = 0;
    do {
        thin_zhang_suen_original_iter(skel, 0);
        thin_zhang_suen_original_iter(skel, 1);
        cv::absdiff( skel, prev, diff );
        skel.copyTo( prev );

        if ( (n_iters++) >= max_iters )
            break;
    }
    while ( cv::countNonZero( diff ) < 0 );

    skel *= 255;
    print_map(skel, "Skel");

    return true;
}

bool thin_fast_custom_voronoi_fn( const cv::Mat1b &img ) {
    cv::Rect _bbox;
    cv::Mat1b skel;

    _bbox = copy_bounding_box_plusone( img, skel, true );


}

int main() {
    const string big_map_filename = "../map_control/big_floor_plan.png";
    const string small_map_filename = "../map_control/floor_plan.png";

    Mat big_map = cv::imread( big_map_filename, IMREAD_COLOR );
    Mat small_map = cv::imread( small_map_filename, IMREAD_COLOR );

    Mat src = big_map.clone();
    thin_zhang_suen_original( src );

//    // Skeletinize
//    Mat gray, binary;
//    cvtColor( src, gray, CV_BGR2GRAY );
//    threshold(gray, binary, 127, 255, CV_THRESH_BINARY);
//    Mat skel(binary.size(), CV_8UC1, Scalar(0)), temp, eroded, element;
//    element = getStructuringElement(MORPH_CROSS, Size(3,3));
//    bool done;
//    do {
//        erode( binary, eroded, element );
//        dilate( eroded, temp, element );
//        subtract( binary, temp, temp );
//        bitwise_or( skel, temp, skel );
//        eroded.copyTo( binary );
//        done =( countNonZero(binary) == 0 );
//    } while ( !done );

//    for (int y = 0; y < skel.rows; y++) {
//        for (int x = 0; x < skel.cols; x++) {
//            skel.at<uchar>( y, 0 ) = 0;
//            skel.at<uchar>( y, skel.cols-1 ) = 0;
//            skel.at<uchar>( 0, x ) = 0;
//            skel.at<uchar>( skel.rows-1, x ) = 0;

//            if ( (int)skel.at<uchar>(y,x) != 0 ) {
//                skel.at<uchar>(y,x) = 127;
//            }
//        }
//    }

    waitKey(0);
    return 0;
}
