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

void print_map(Mat &map, string s) {
    Mat resizeMap;
    resize( map, resizeMap, map.size()*10, 0, 0, INTER_NEAREST);
    imshow(s, resizeMap);
}

void draw_pixel_red(vector<Point> &v, Mat &img) {
    for (size_t i = 0; i < v.size(); i++) {
        Vec3b color(0, 0, 255);
        img.at<Vec3b>(v[i].y, v[i].x) = color;
    }
}
 /* MIKKEL STYKKE TIL RAPPORT SKRIVNING
int main(int argc, char** argv)
{

    //const char* default_file = "../map_control/floor_plan.png";
    const char* default_file = "../map_control/big_floor_plan.png";
    const char* filename = argc >=2 ? argv[1] : default_file;
    Mat src = cv::imread(filename, IMREAD_COLOR);

    Map grid;
    Mat img = grid.brushfire_img(src);
    vector<Point> mid_points = grid.find_centers(img);

    draw_pixel_red(mid_points, src);
    //printMap(src);
    // Loads an image
    Mat smallWorld = imread( filename, IMREAD_GRAYSCALE );
    Map smallMap(smallWorld);

    vector<Point> detectedCorners = smallMap.cornerDetection();
    smallMap.trapezoidalLines(detectedCorners);

    smallMap.printMap();
    smallMap.drawNShowPoints("Detected Corners", detectedCorners);
    */
    /* SHOWS UPPER AND LOWER SEPARATELY
    smallMap.drawNShowPoints("Upper Goals", smallMap.getUpperTrapezoidalGoals());
    smallMap.drawNShowPoints("Lower Goals", smallMap.getLowerTrapezoidalGoals());
    */
    //SHOWS UPPER AND LOWER TOGETHER
    /*
    Mat tempMap = smallWorld;
    cvtColor(smallWorld, tempMap, COLOR_GRAY2BGR);
    vector<Point> upper = smallMap.getUpperTrapezoidalGoals();
    vector<Point> lower = smallMap.getLowerTrapezoidalGoals();
    vector<Point> upperNlower;
    // DRAWS IN SAME MAP
    for(size_t i = 0; i < upper.size(); i++) // Upper goal green
    {
        //cout << "Upper x: " << upper[i].x << " y: " << upper[i].y << endl;
        tempMap.at<Vec3b>(upper[i].y,upper[i].x)[1] = 255;
        upperNlower.push_back(upper[i]);
    }
    for(size_t i = 0; i < lower.size(); i++)
    {
        //cout << "Lower x: " << lower[i].x << " y: " << lower[i].y << endl;
        tempMap.at<Vec3b>(lower[i].y,lower[i].x)[2] = 255; // Lower goal red
        upperNlower.push_back(lower[i]);
    }

    // COUT COORDINATES FROM PIXELS TO GAZEBO
    vector<Point_<double>> gazeboGoals = smallMap.convertToGazeboCoordinatesTrapezoidal(upper, lower);
    for(size_t i = 0; i < gazeboGoals.size(); i++)
    {
        //cout << "Goal " << i << " x: " << gazeboGoals[i].x << " y: " << gazeboGoals[i].y << endl;
    }

    resize(tempMap,tempMap,tempMap.size()*10,0,0,INTER_NEAREST);
    imshow("Goals", tempMap);
    vector<Cell> t = smallMap.calculateCells(upper, lower);
    Mat img_Boustrophedon = smallMap.drawCellsPath("Boustrophedon", t);
    */

    //smallMap.drawCellsPath("Cell Path", t);
    /*
    for(size_t i = 0; i < t.size(); i++)
    {
        for(size_t j = 0; j < t[i].neighborcells.size(); j++)
        {
            cout << "Celle 1 " << i << " Punkt x: " << t[i].cellPoint.x << " Punkt y: " << t[i].cellPoint.y << " Connected til " << j << " x: " << t[i].neighborcells[j].x << " y: " << t[i].neighborcells[j].y << endl;
        }
    }
    */
    /* Writes to file
    // OPSTACLE
    vector<Point> obstacle;
    Point temp;
    for(int i = 0; i < smallWorld.rows; i++)
    {
        for(int j = 0; j < smallWorld.cols; j++)
        {
            if(smallWorld.at<uchar>(i,j) == 255)
            {
                temp.x = j;
                temp.y = i;
                obstacle.push_back(temp);
            }
        }
    }
    */
Vec3b red(0,0,255), black(0,0,0), white(255,255,255), blue(255,0,0);

// CONSTANTS
Mat big_map = cv::imread( "../map_control/big_floor_plan.png", IMREAD_COLOR);
Mat small_map = cv::imread( "../map_control/floor_plan.png", IMREAD_COLOR );

int main( ) {
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
    print_map( src, "Voronoi Diagram" );

    A_Star *a = new A_Star();
    std::vector<cv::Point> v = a->get_path( src, points[0], points[ points.size()-1 ] );
    Mat img = big_map.clone();
    for ( auto& p : v )
        img.at<Vec3b>( p ) = red;
    print_map( img, "Map" );
    // NOGET GALT I MIN FUNKTION
    vector<double> voronoiLength = a->findAstarPathLengthsForRoadmap(src);
    for ( size_t i = 0; i < voronoiLength.size(); i++)
    {
        cout << "Number start-end point: " << i+1 << " Total Length " << voronoiLength[i] << endl;
    }
    waitKey(0);
    return 0;
}
