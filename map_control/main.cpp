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
#include <thread>
#include "Map.h"
#include "path_planning.h"
#include "Voronoi_Diagram.h"
#include "A_Star.h"
#include "DetectRooms.h"

#include <random>
using namespace std;
using namespace cv;

Vec3b red(0,0,255), black(0,0,0), white(255,255,255), blue(255,0,0);
RNG rng(12345);

void printMap(Mat &map, string s)
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

int main( ) {
    Vec3b red(0,0,255), black(0,0,0), white(255,255,255), blue(255,0,0);
    Mat big_map = cv::imread( "../map_control/big_floor_plan.png", IMREAD_COLOR);
    Mat big_map2 = cv::imread( "../map_control/big_floor_test.png", IMREAD_COLOR );
    Mat big_map3 = cv::imread( "../map_control/big_floor_test2.png", IMREAD_COLOR );
    Mat small_map = cv::imread( "../map_control/floor_plan.png", IMREAD_COLOR );

    A_Star *a = new A_Star(big_map);
    Mat src = big_map.clone(), dst;

    Voronoi_Diagram *v_d = new Voronoi_Diagram();
    v_d->get_voronoi_img( src, dst );
    std::vector<Point> points;
    for (int y = 0; y < dst.rows; y++)
       for (int x = 0; x < dst.cols; x++)
           if ( (int)dst.at<uchar>(y,x) == 255 )
               points.push_back( Point(x,y) );
    for ( auto& p : points )
       src.at<Vec3b>( p ) = red;
    print_map(src, "Voronoi Diagram"  );
    // Boustrophedon
    Mat src1 = big_map.clone();
    cvtColor(src1, src1, CV_BGR2GRAY);
    Map Boustrophedon(src1);
    vector<Point> detectedCorners = Boustrophedon.cornerDetection();
    Boustrophedon.trapezoidalLines(detectedCorners);
    vector<Point> upper = Boustrophedon.getUpperTrapezoidalGoals();
    vector<Point> lower = Boustrophedon.getLowerTrapezoidalGoals();

    Mat src2 = big_map.clone(); // Draw points on
    for ( auto& p : upper )
       src2.at<Vec3b>( p ) = red;
    for ( auto& p : lower )
       src2.at<Vec3b>( p ) = red;
    print_map(src2, "Critical Points Boustrophedon Diagram"  );
    vector<Cell> t = Boustrophedon.calculateCells(upper, lower);
    Mat img_Boustrophedon = Boustrophedon.drawCellsPath("Boustrophedon", t);


    waitKey(0);
    return 0;

}
