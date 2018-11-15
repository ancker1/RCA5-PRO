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
vector<Point> lines;

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
        if ( img.at<Vec3b>( v[i] ) == black )
            return true;
    }

    return false;
}

void forward_in_img( Mat &img, Point start) {
    img.at<Vec3b>( start ) = white;
    for ( int y = start.y ; y < img.rows ; y++ )
        for ( int x = 0 ; x < img.cols ; x++ ) {
            if ( img.at<Vec3b>(y,x) == red )
                if ( !obstacle_detected( img, start, Point(x,y) ) ) {
                    LineIterator it( img, start, Point(x,y), 8 );
                    for ( int i = 0 ; i < it.count ; i++, it++ )
                        lines.push_back( it.pos() );
                    return;
                }
        }
}

void backward_in_img( Mat &img, Point start ) {
    for ( int y = img.rows ; y > start.y ; y-- )
        for ( int x = img.cols ; x > start.x ; x-- ) {
            if ( img.at<Vec3b>(y,x) == red )
                if ( !obstacle_detected( img, start, Point(x,y) ) ) {
                    LineIterator it( img, start, Point(x,y), 8 );
                    vector<Point> v(it.count);
                    for ( int i = 0 ; i < it.count ; i++, it++ )
                        v[i] = it.pos();
                    for ( unsigned i = 0; i < v.size(); i++ )
                        img.at<Vec3b>( v[i] ) = red;
                    return;
                }
        }
}

bool remove_some_points( Mat &img, Point p, int thresh ) {
    bool result = false;;
    for ( int y = p.y-1; y < ( p.y+thresh ) && y < img.rows; y++ )
        for ( int x = p.x-1; x < ( p.x+thresh ) && x < img.cols; x++ ) {
            if ( img.at<Vec3b>(y,x) == red ) {
                img.at<Vec3b>(y,x) = white;
                result = true;
            }
        }

    for ( int y = p.y+1; y > ( p.y-thresh ) && y > 0; y-- )
        for ( int x = p.x+1; x > ( p.x-thresh ) && x > 0; x-- ) {
            if ( img.at<Vec3b>(y,x) == red ) {
                img.at<Vec3b>(y,x) = white;
                result = true;
            }
        }
    return result;
}

int main() {
    const string big_map_filename = "../map_control/big_floor_plan.png";
    const string small_map_filename = "../map_control/floor_plan.png";

    Mat big_map = cv::imread( big_map_filename, IMREAD_COLOR );
    Mat small_map = cv::imread( small_map_filename, IMREAD_COLOR );

    Mat img = big_map.clone();
    Mat img2 = img.clone();

    Voronoi_Diagram v_d(img);
    Mat b_img = v_d.get_brushfire_grid();
    Mat kernel( Size(5,5), CV_8U );
    Mat b_dilate;
    dilate( b_img, b_dilate, kernel );
    Mat1b local_max = ( b_img >= b_dilate );
    for (int y = 0; y < local_max.rows; y++) {
        for (int x = 0; x < local_max.cols; x++) {
            if ((int)local_max.at<uchar>(y,x) == 255) {
                for (int j = 1; (int)local_max.at<uchar>(y+j,x) == 255; j++) {
                    local_max.at<uchar>(y,x) = 0;
                    j++;
                }
                for (int j = 1; (int)local_max.at<uchar>(y,x+j) == 255; j++) {
                    local_max.at<uchar>(y,x) = 0;
                    j++;
                }
            }
        }
    }

    vector<Point> v;
    for (int y = 0; y < local_max.rows; y++) {
        for (int x = 0; x < local_max.cols; x++) {
            if ((int)local_max.at<uchar>(y,x) == 255) {
                v.push_back( Point(x,y) );
            }
        }
    }

    for (unsigned i = 0; i < v.size(); ) {
        if ((int)b_img.at<uchar>(v[i].y-1,v[i].x-1)==0) {
            v.erase(v.begin()+i);
        }
        else if ((int)b_img.at<uchar>(v[i].y,v[i].x-1)==0) {
            v.erase(v.begin()+i);
        }
        else if ((int)b_img.at<uchar>(v[i].y+1,v[i].x-1)==0) {
            v.erase(v.begin()+i);
        }
        else if ((int)b_img.at<uchar>(v[i].y-1,v[i].x)==0) {
            v.erase(v.begin()+i);
        }
        else if ((int)b_img.at<uchar>(v[i].y+1,v[i].x)==0) {
            v.erase(v.begin()+i);
        }
        else if ((int)b_img.at<uchar>(v[i].y-1,v[i].x+1)==0) {
            v.erase(v.begin()+i);
        }
        else if ((int)b_img.at<uchar>(v[i].y,v[i].x+1)==0) {
            v.erase(v.begin()+i);
        }
        else if ((int)b_img.at<uchar>(v[i].y+1,v[i].x+1)==0) {
            v.erase(v.begin()+i);
        }
        else {
            i++;
        }
    }

    for (int i = 0; i < local_max.cols; i++) {
        local_max.at<uchar>(0, i) = 0;
        local_max.at<uchar>(local_max.rows-1, i) = 0;
    }
    for (int i = 0; i < local_max.rows; i++) {
        local_max.at<uchar>(i, 0) = 0;
        local_max.at<uchar>(i, local_max.cols-1) = 0;
    }

    v.clear();
    for (int y = 0; y < img.rows; y++) {
        for (int x = 0; x < img.cols; x++) {
            if ( (int)local_max.at<uchar>(y,x) == 255 )
                v.push_back( Point(x,y) );
        }
    }

    v.erase( v.begin()+13 );

    for (unsigned i = 0; i < v.size(); i++) {
        img.at<Vec3b>( v[i] ) = red;
    }

    print_map(img, "Peaks");

    Map map;
    Mat brushfire = map.brushfire_img(img2);
    vector<Point> v2 = map.find_centers(brushfire);
    v2.erase( v2.begin()+15 );
    for (unsigned i = 0; i < v2.size(); i++) {
        img2.at<Vec3b>( v2[i] ) = red;
        img.at<Vec3b>( v2[i] ) = red;
    }

    for ( int y = 0; y < img.rows; y++)
        for (int x = 0; x < img.cols; x++) {
            Vec3b left = img.at<Vec3b>(y,x-1);
            Vec3b right = img.at<Vec3b>(y,x+1);
            Vec3b up = img.at<Vec3b>(y-1,x);
            Vec3b down = img.at<Vec3b>(y+1,x);
            Vec3b left_up = img.at<Vec3b>(y-1,x-1);
            Vec3b left_down = img.at<Vec3b>(y+1,x-1);
            Vec3b right_up = img.at<Vec3b>(y-1,x+1);
            Vec3b right_down = img.at<Vec3b>(y+1,x+1);
            if ( left==red || right==red || up==red || down==red ||
                 left_up==red || right_up==red || left_down==red || right_down==red) {
                img.at<Vec3b>(y,x) = white;
            }
        }

    print_map(img, "Map");

//    for (unsigned i = 0; i < v.size(); i++) {
//        forward_in_img(img, v[i]);
//    }

//    for (unsigned i = 0; i < lines.size(); i++) {
//        img.at<Vec3b>( lines[i] ) = red;
//    }
//    print_map(img, "Map");


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
