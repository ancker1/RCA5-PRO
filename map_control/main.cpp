#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>

#include <iostream>
#include <Map.h>

using namespace std;
using namespace cv;

int main()
{
    Mat smallWorld = imread("/home/mikkel/Documents/git_workspace/rb5-pro/models/smallworld/meshes/floor_plan.png", IMREAD_COLOR); // Imports the picture
    Size size(300,300);//the dst image size,e.g.100x100
    Mat test;//dst image
    resize(smallWorld,test,size);//resize image
    imshow("t", test);
    Map smallMap(smallWorld);
    waitKey(0);
    cout << smallMap.getMapCols() << " " << smallMap.getMapRows() << endl;
    smallMap.findMiddleOfRooms(5);
    return 0;
}
