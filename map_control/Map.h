#ifndef MAP_H
#define MAP_H
#include<iostream>
#include<vector>
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>

using namespace std;
using namespace cv;

class Map
{
public:
    Map();
    Map(Mat picture);

    int getMapRows();
    int getMapCols();
    void printMap();
    void findMiddleOfRooms(int threshold);
    ~Map();



private:
    Mat map;
    struct wall{
        Point start;
        Point end;
    };
};

#endif // MAP_H
