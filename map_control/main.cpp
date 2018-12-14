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
#include "Boustrophedon.h"

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
    Mat big_map1 = cv::imread( "../map_control/big_floor_plan.png", IMREAD_COLOR);
    Mat big_map2 = cv::imread( "../map_control/big_floor_test.png", IMREAD_COLOR );
    Mat big_map3 = cv::imread( "../map_control/big_floor_test2.png", IMREAD_COLOR );
    Mat small_map = cv::imread( "../map_control/floor_plan.png", IMREAD_COLOR );

    A_Star *a = new A_Star(big_map1);
    Mat src = big_map1.clone(), dst;

    Voronoi_Diagram *v_d = new Voronoi_Diagram();
    v_d->get_voronoi_img( src, dst );
    std::vector<Point> points;
    for (int y = 0; y < dst.rows; y++)
       for (int x = 0; x < dst.cols; x++)
           if ( (int)dst.at<uchar>(y,x) == 255 )
               points.push_back( Point(x,y) );
    for ( auto& p : points )
       src.at<Vec3b>( p ) = red;
    //print_map(src, "Voronoi Diagram"  );


    // Boustrophedon
    Mat img_Boustrophedon;
    Mat src1 = big_map1.clone();
    cvtColor(src1, src1, CV_BGR2GRAY);
    Map Boustrophedon(src1);
    vector<Point> detectedCorners = Boustrophedon.cornerDetection();
    Boustrophedon.trapezoidalLines(detectedCorners);
    vector<Point> upper = Boustrophedon.getUpperTrapezoidalGoals();
    vector<Point> lower = Boustrophedon.getLowerTrapezoidalGoals();
    vector<Cell> t = Boustrophedon.calculateCells(upper, lower);
    img_Boustrophedon = Boustrophedon.drawCellsPath("Boustrophedon", t);

    int sampleSize = 10000;
    vector<Point> startPoints;
    vector<Point> endPoints;
    //srand(time(0));
    default_random_engine generator;
    uniform_int_distribution<int> distribution_rows(0,src.rows-1);
    uniform_int_distribution<int> distribution_cols(0,src.cols-1);
    Point randTemp;
    for(int i = 0; i < sampleSize; i++)
    {
        randTemp.x = distribution_rows(generator);
        randTemp.y = distribution_cols(generator);
        startPoints.push_back(randTemp);
    }
    for(int i = 0; i < sampleSize; i++)
    {
        randTemp.x = distribution_rows(generator);
        randTemp.y = distribution_cols(generator);
        endPoints.push_back(randTemp);
    }

    vector<Point> roadmapPoints_voronoi = a->calculateRoadmapPoints(src); // Points on Roadmap
    vector<Point> startPoints_voronoi = a->checkInvalidTestPoints(src, roadmapPoints_voronoi, startPoints);
    vector<Point> endPoints_voronoi = a->checkInvalidTestPoints(src, roadmapPoints_voronoi, endPoints);

    vector<Point> roadmapPoints_boustrophedon = a->calculateRoadmapPoints(img_Boustrophedon);
    vector<Point> startPoints_Boustrophedoni = a->checkInvalidTestPoints(img_Boustrophedon, roadmapPoints_boustrophedon, startPoints_voronoi);

    vector<Point> endPoints_Boustrophedon = a->checkInvalidTestPoints(img_Boustrophedon, roadmapPoints_boustrophedon, endPoints_voronoi);
    startPoints = startPoints_Boustrophedoni;
    endPoints = endPoints_Boustrophedon;
    // Make startpoints and endpoints same size
    if(startPoints.size() != endPoints.size())
    {
        if(startPoints.size() < endPoints.size())
            endPoints.erase(endPoints.begin(), endPoints.begin()+(endPoints.size()-startPoints.size()));
        else
            startPoints.erase(startPoints.begin(), startPoints.begin()+(startPoints.size()-endPoints.size()));

    }
    cout << "test startpoint size: " << startPoints.size() << "test endpoints size: " << endPoints.size() << endl;
    vector<double> voronoiLength = a->findAstarPathLengthsForRoadmapRandom(src, roadmapPoints_voronoi, startPoints, endPoints); // random start- and end- points
    //vector<double> voronoiLength = a->findAstarPathLengthsForRoadmap(src); // Towards eachother
    vector<double> BoustrophedonLength = a->findAstarPathLengthsForRoadmapRandom(img_Boustrophedon, roadmapPoints_boustrophedon, startPoints, endPoints); // random start- and end- points
    //vector<double> BoustrophedonLength = a->findAstarPathLengthsForRoadmap(img_Boustrophedon); // Towards eachother

    // Sorts the results for plotting
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

    resize(img_Boustrophedon,img_Boustrophedon,img_Boustrophedon.size()*10,0,0,INTER_NEAREST);
    imwrite("Boustrophedon_roadmap_bigmap1.png", img_Boustrophedon);

    // Writing Results:
    string file = "voronoi_length_test_rand_bigMap.txt";
    ofstream myFile;
    myFile.open(file);
    for(size_t i = 0; i < sorted_voronoi_length.size(); i++)
        myFile << sorted_voronoi_length[i] << "\n";
    myFile.close();
    file = "Boustrophedon_length_test_rand_bigMap.txt";
    myFile.open(file);
    for(size_t i = 0; i < sorted_boustrophedon_length.size(); i++)
        myFile << sorted_boustrophedon_length[i] << "\n";
    myFile.close();

    //Plot best and worst case map for Big_Map Boustro best at 3991 Voro best at 4598
    int indexToCheck = 4598;
    Mat BoustroIndexPath = a->showPath(big_map1, img_Boustrophedon, roadmapPoints_boustrophedon, sorted_start_points[indexToCheck], sorted_end_points[indexToCheck]);
    Mat VoroIndexPath = a->showPath(big_map1, src, roadmapPoints_voronoi, sorted_start_points[indexToCheck], sorted_end_points[indexToCheck]);
    resize(BoustroIndexPath,BoustroIndexPath,BoustroIndexPath.size()*10,0,0,INTER_NEAREST);
    resize(VoroIndexPath,VoroIndexPath,VoroIndexPath.size()*10,0,0,INTER_NEAREST);
    imshow("Boustro_worst_index_4598.png", BoustroIndexPath);
    imshow("Voro_best_index_4598.png", VoroIndexPath);

    waitKey(0);

    return 0;
}
