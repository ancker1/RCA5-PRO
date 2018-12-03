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



Vec3b red(0,0,255), black(0,0,0), white(255,255,255), blue(255,0,0);

int main( ) {
    Mat big_map = cv::imread( "../map_control/big_floor_plan.png", IMREAD_COLOR);
    Mat big_map2 = cv::imread( "../map_control/big_floor_test.png", IMREAD_COLOR );
    Mat big_map3 = cv::imread( "../map_control/big_floor_test2.png", IMREAD_COLOR );
    Mat small_map = cv::imread( "../map_control/floor_plan.png", IMREAD_COLOR );

    Mat src = big_map.clone();

    // Get brushfire grid
    Map *m = new Map();
    Mat imgBrushfire = m->brushfire_img( src );

    // Detected rooms
    Mat imgDilate, element = getStructuringElement( MORPH_CROSS, Size(4,4), Point(0,0) );
    dilate( imgBrushfire, imgDilate, element );
    convertScaleAbs( imgDilate, imgDilate, 10 );
    for (int y = 0; y < imgDilate.rows; y++)
        for (int x = 0; x < imgDilate.cols; x++)
        {
            if ( (int)imgDilate.at<uchar>(y,x) >= 60 )
                imgDilate.at<uchar>(y,x) = 255;
            else
                imgDilate.at<uchar>(y,x) = 0;
        }

    // Get contours
    Mat cannyOutput;
    vector<vector<Point>> contours;
    vector<Vec4i> hierarchy;
    Canny( imgDilate, cannyOutput, 100, 200, 3 );
    findContours( cannyOutput, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0,0) );

    // Get the moments
    vector<Moments> mu( contours.size() );
    for (size_t i = 0; i < contours.size(); i++)
        mu[i] = moments( contours[i], false );

    // Get mass center
    vector<Point> mc( contours.size() );
    for (size_t i = 0; i < contours.size(); i++)
        mc[i] = Point( ( mu[i].m10/mu[i].m00 ), ( mu[i].m01/mu[i].m00 ) );

    Mat drawing = big_map.clone();
    for (size_t i = 0; i < contours.size(); i++)
    {
        drawContours( drawing, contours, (int)i, Scalar(0,0,250) );
        drawing.at<Vec3b>( mc[i] ) = blue;

        print_map( drawing, "Contours" );
        cv::waitKey(0);
    }

    waitKey(0);
    return 0;
}
