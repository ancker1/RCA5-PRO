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
#include <random>
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

int main( ) {
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
    imshow( "Voronoi Diagram",src  );
    /* TAKES TOO LONG TIME
    // Calulation of whole map path for Voronoi
    vector<Point> roadmapPoints = a->calculateRoadmapPoints(src);
    vector<Point> testPoints = a->calculateTestPoints(src, roadmapPoints);
    thread thread1 (&A_Star::calculateDistThread, A_Star(), src, testPoints, roadmapPoints, 1, 8);
    thread thread2 (&A_Star::calculateDistThread, A_Star(), src, testPoints, roadmapPoints, 2, 8);
    thread thread3 (&A_Star::calculateDistThread, A_Star(), src, testPoints, roadmapPoints, 3, 8);
    thread thread4 (&A_Star::calculateDistThread, A_Star(), src, testPoints, roadmapPoints, 4, 8);
    thread thread5 (&A_Star::calculateDistThread, A_Star(), src, testPoints, roadmapPoints, 5, 8);
    thread thread6 (&A_Star::calculateDistThread, A_Star(), src, testPoints, roadmapPoints, 6, 8);
    thread thread7 (&A_Star::calculateDistThread, A_Star(), src, testPoints, roadmapPoints, 7, 8);
    thread thread8 (&A_Star::calculateDistThread, A_Star(), src, testPoints, roadmapPoints, 8, 8);

    thread1.join();
    thread2.join();
    thread3.join();
    thread4.join();
    thread5.join();
    thread6.join();
    thread7.join();
    thread8.join();
    */
    int sampleSize = 10000;
    vector<Point> startPoints;
    vector<Point> endPoints;
    //srand(time(0));
    default_random_engine generator;
    uniform_int_distribution<int> distribution_row(0,src.rows-1);
    uniform_int_distribution<int> distribution_cols(0,src.cols-1);
    Point randTemp;
    for(int i = 0; i < sampleSize; i++)
    {
        randTemp.x = distribution_row(generator);
        randTemp.y = distribution_cols(generator);
        startPoints.push_back(randTemp);
    }
    for(int i = 0; i < sampleSize; i++)
    {
        randTemp.x = distribution_row(generator);
        randTemp.y = distribution_cols(generator);
        endPoints.push_back(randTemp);
    }
    /*
    for(size_t i = 0; i < endPoints.size(); i++)
        cout << "start " << startPoints[i] << " end " << endPoints[i] << endl;
        */

    vector<Point> roadmapPoints_voronoi = a->calculateRoadmapPoints(src);
    vector<Point> startPoints_voronoi = a->checkInvalidTestPoints(src, roadmapPoints_voronoi, startPoints);
    vector<Point> endPoints_voronoi = a->checkInvalidTestPoints(src, roadmapPoints_voronoi, endPoints);

    // Boustrophedon
    Mat src1 = big_map.clone();
    cvtColor(src1, src1, CV_BGR2GRAY);
    Map Boustrophedon(src1);
    vector<Point> detectedCorners = Boustrophedon.cornerDetection();
    Boustrophedon.trapezoidalLines(detectedCorners);
    vector<Point> upper = Boustrophedon.getUpperTrapezoidalGoals();
    vector<Point> lower = Boustrophedon.getLowerTrapezoidalGoals();
    vector<Cell> t = Boustrophedon.calculateCells(upper, lower);
    Mat img_Boustrophedon = Boustrophedon.drawCellsPath("Boustrophedon", t);

    vector<Point> roadmapPoints_boustrophedon = a->calculateRoadmapPoints(img_Boustrophedon);
    vector<Point> startPoints_Boustrophedoni = a->checkInvalidTestPoints(img_Boustrophedon, roadmapPoints_boustrophedon, startPoints);

    vector<Point> endPoints_Boustrophedon = a->checkInvalidTestPoints(img_Boustrophedon, roadmapPoints_boustrophedon, endPoints);
    startPoints = a->findNRemoveDiff(startPoints_voronoi, startPoints_Boustrophedoni);
    endPoints = a->findNRemoveDiff(endPoints_voronoi, endPoints_Boustrophedon);
    cout << "test startpoint size: " << startPoints.size() << "test endpoints size: " << endPoints.size() << endl;

    vector<double> voronoiLength = a->findAstarPathLengthsForRoadmapRandom(src, roadmapPoints_voronoi, startPoints, endPoints); // random start- and end- points
    //vector<double> voronoiLength = a->findAstarPathLengthsForRoadmap(src); // Towards eachother
    vector<double> BoustrophedonLength = a->findAstarPathLengthsForRoadmapRandom(img_Boustrophedon, roadmapPoints_boustrophedon, startPoints, endPoints); // random start- and end- points
    //vector<double> BoustrophedonLength = a->findAstarPathLengthsForRoadmap(img_Boustrophedon); // Towards eachother

    vector<double> sorted_voronoi_length;
    vector<double> sorted_boustrophedon_length;
    vector<Point> sorted_start_points;
    vector<Point> sorted_end_points;
    double smallest;
    size_t index;
    while(!voronoiLength.empty())
    {
        smallest = voronoiLength[0];
        for(size_t i = 0; i < voronoiLength.size(); i++)
        {
            if(voronoiLength[i] <= smallest)
            {
                index = i;
                smallest = voronoiLength[i];
            }
        }

        sorted_voronoi_length.push_back(voronoiLength[index]);
        sorted_boustrophedon_length.push_back(BoustrophedonLength[index]);
        sorted_start_points.push_back(startPoints[index]);
        sorted_end_points.push_back(endPoints[index]);
        voronoiLength.erase(voronoiLength.begin()+index);
        BoustrophedonLength.erase(BoustrophedonLength.begin()+index);
        startPoints.erase(startPoints.begin()+index);
        endPoints.erase(endPoints.begin()+index);
    }
    // Writing Results:
    string file = "voronoi_length_test_rand_smallMap.txt";
    ofstream myFile;
    myFile.open(file);
    for(size_t i = 0; i < sorted_voronoi_length.size(); i++)
        myFile << sorted_voronoi_length[i] << endl;
    myFile.close();

    file = "Boustrophedon_length_test_rand_smallMap.txt";
    myFile.open(file);
    for(size_t i = 0; i < sorted_boustrophedon_length.size(); i++)
        myFile << sorted_boustrophedon_length[i] << endl;
    myFile.close();

    //Plot best and worst case map for Big_Map Boustro best at 3982 Voro best at 4852 //
    /*
    int indexToCheck = 3982;
    Point voroGraphPointStart = a->findWayToRoadMap(src, roadmapPoints_voronoi, sorted_start_points[indexToCheck]);
    Point voroGraphPointEnd = a->findWayToRoadMap(src, roadmapPoints_voronoi, sorted_end_points[indexToCheck]);
    Point bostoGraphPointStart = a->findWayToRoadMap(img_Boustrophedon, roadmapPoints_boustrophedon, sorted_start_points[indexToCheck]);
    Point bostoGraphPointEnd = a->findWayToRoadMap(img_Boustrophedon, roadmapPoints_boustrophedon, sorted_end_points[indexToCheck]);
    vector<Point> voro = a->get_path(src, voroGraphPointStart, voroGraphPointEnd);
    vector<Point> bostro = a->get_path(img_Boustrophedon, bostoGraphPointStart, bostoGraphPointEnd);
    cout << "size af voro path: " << voro.size() << endl;
    cout << "size af bostro path: " << bostro.size() << endl;
    Mat vorot = big_map.clone();
    Mat bostrot = big_map.clone();
    for ( auto& p : voro )
       vorot.at<Vec3b>( p ) = red;
    line(vorot, sorted_start_points[indexToCheck], voroGraphPointStart, Scalar(255,0,0), 1, 8, 0); // Draw line from start point to start point on graph
    line(vorot, sorted_end_points[indexToCheck], voroGraphPointEnd, Scalar(0,255,0), 1, 8, 0); // Draw line from end point to end point on graph
    //print_map( vorot, "Voronoi test" );
    resize(vorot,vorot,vorot.size()*10,0,0,INTER_NEAREST);
    imwrite("Voronoi_path_sample_10000_index_3982.png", vorot);
    for ( auto& p : bostro )
       bostrot.at<Vec3b>( p ) = red;
    line(bostrot, sorted_start_points[indexToCheck], bostoGraphPointStart, Scalar(255,0,0), 1, 8, 0); // Draw line from start point to start point on graph
    line(bostrot, sorted_end_points[indexToCheck], bostoGraphPointEnd, Scalar(0,255,0), 1, 8, 0); // Draw line from end point to end point on graph
    //print_map( bostrot, "Bostrot test" );
    resize(bostrot,bostrot,bostrot.size()*10,0,0,INTER_NEAREST);
    imwrite("Boustrophedon_path_sample_10000_index_3982.png", bostrot);
    //waitKey(0);
    return 0;
    */
}
