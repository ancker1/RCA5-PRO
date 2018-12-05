#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/ximgproc.hpp"

#include <iostream>
#include <fstream>
#include <vector>
#include <Map.h>
#include <Cell.h>
#include <Cellpoint.h>
#include <vector>

#include "Map.h"
#include "path_planning.h"
#include "Voronoi_Diagram.h"
#include "A_Star.h"

using namespace std;
using namespace cv;

Vec3b red(0,0,255), black(0,0,0), white(255,255,255), blue(255,0,0);
RNG rng(12345);

void print_map(Mat &map, string s)
{
    Mat resizeMap;
    resize( map, resizeMap, map.size()*10, 0, 0, INTER_NEAREST);
    imshow(s, resizeMap);
}

void draw_pixel_red(vector<Point> &v, Mat &img)
{
    for (size_t i = 0; i < v.size(); i++) {
        Vec3b color(0, 0, 255);
        img.at<Vec3b>(v[i].y, v[i].x) = color;
    }
}

void makeRectangle( cv::Mat &img,
                    const cv::Point &p,
                    std::vector<cv::Point> &centers )
{

    int y = p.y, x = p.x;
    for ( ; y < img.rows; y++)
        if ( (int)img.at<uchar>( y, p.x ) == 0 )
        {
            y--;
            break;
        }

    for ( ; x < img.cols; x++)
        if ( (int)img.at<uchar>( p.y, x ) == 0 )
        {
            x--;
            break;
        }

    Point center( (p.x+x)/2, (p.y+y)/2 );
    centers.push_back( center );

    for (int i = p.y-7 ; i < y+7; i++ )
        for (int j = p.x-7; j < x+7; j++)
            img.at<uchar>( i, j ) = 0;
}

int main( ) {
    Mat big_map = cv::imread( "../map_control/big_floor_plan.png", IMREAD_COLOR);
    Mat big_map2 = cv::imread( "../map_control/big_floor_test.png", IMREAD_COLOR );
    Mat big_map3 = cv::imread( "../map_control/big_floor_test2.png", IMREAD_COLOR );
    Mat small_map = cv::imread( "../map_control/floor_plan.png", IMREAD_COLOR );

//    Mat src = big_map.clone(), img;
//    cvtColor( src, img, CV_BGR2GRAY );
//    threshold( img, img, 100, 255, THRESH_BINARY );
//    Mat element = getStructuringElement( MORPH_RECT, Size(10,10) );
//    erode( img, img, element );
//    print_map( img, "B" );

//    vector<Point> centers;
//    for (int y = 4; y < img.rows-4; y++)
//        for (int x = 4; x < img.cols-4; x++)
//            if ( (int)img.at<uchar>(y,x) == 255 )
//                makeRectangle( img, Point(x,y), centers );

//    Mat dst = big_map.clone();
//    for ( auto& point : centers )
//        dst.at<Vec3b>( point ) = red;

//    print_map( dst, "C" );

//    Mat src = big_map.clone();

//    Mat imgBinary;
//    cvtColor( src, imgBinary, CV_BGR2GRAY );
//    threshold( imgBinary, imgBinary, 100, 255, CV_THRESH_BINARY );

//    // Get walls
//    vector<Point> walls;
//    for (int y = 0; y < imgBinary.rows; y++)
//        for (int x = 0; x < imgBinary.cols; x++)
//            if ( (int)imgBinary.at<uchar>(y,x) == 0 )
//                walls.push_back( Point(x,y) );

//    Map *m = new Map();

//    Mat imgBrushfire = m->brushfire_img( src );

//    convertScaleAbs( imgBrushfire, imgBrushfire, 20 );

//    // Generate gradX and gradY
//    Mat grad, gradX, gradY, absGradX, absGradY;

//    // Gradient X
//    Sobel( imgBrushfire, gradX, CV_16S, 1, 0 );
//    convertScaleAbs( gradX, absGradX );

//    // Gradient Y
//    Sobel( imgBrushfire, gradY, CV_16S, 0, 1 );
//    convertScaleAbs( gradY, absGradY );

//    // Total Gradient (approximate)
//    addWeighted( absGradX, 1, absGradY, 1, 0, grad );

//    for ( auto& point : walls )
//        grad.at<uchar>( point ) = 255;

//    for (int y = 0; y < grad.rows; y++)
//        for (int x = 0; x < grad.cols; x++)
//        {
//            if ( (int)grad.at<uchar>(y,x) < 100 )
//                grad.at<uchar>(y,x) = 0;

//            grad.at<uchar>( y, 0 ) = 255;
//            grad.at<uchar>( y, grad.cols-1 ) = 255;

//            grad.at<uchar>( 0, x ) = 255;
//            grad.at<uchar>( grad.rows-1, x ) = 255;
//        }

//    print_map( grad, "Grad" );

    Mat src, img, imgVisibility;
    Map *m = new Map();
    Path_planning *pp = new Path_planning();
    vector<Point> centers;

    centers.clear();
    src = big_map3.clone();
    centers = m->brushfireFindCenters( src );
    img = big_map3.clone();
    imgVisibility = pp->make_visibility_map(img, centers);
    print_map( imgVisibility, "Map" );

    waitKey(0);
    return 0;
}
