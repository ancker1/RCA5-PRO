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

void print_map(Mat &map, string s) {
    Mat resizeMap;
    resize( map, resizeMap, map.size()*10, 0, 0, INTER_NEAREST);
    imshow(s, resizeMap);
}

int main() {
    const string big_map_filename = "../map_control/big_floor_plan.png";
    const string small_map_filename = "../map_control/floor_plan.png";

    Mat big_map = cv::imread( big_map_filename, IMREAD_COLOR );
    Mat small_map = cv::imread( small_map_filename, IMREAD_COLOR );

    Mat img = big_map.clone();

    Voronoi_Diagram v_diagram(img);
    Mat brushfire = v_diagram.get_brushfire_grid();

    Mat b_img;
    convertScaleAbs(brushfire, b_img, 255);
    cout << "Brushfire" << endl;
    cout << b_img << endl;
    print_map(b_img, "Brushfire");

    vector<Point> walls;
    for (int y = 0; y < b_img.rows; y++) {
        for (int x = 0; x < b_img.cols; x++) {
            if ( (int)b_img.at<uchar>(y,x)==0 ) {
                walls.push_back( Point(x,y) );
            }
        }
    }

    // SOBEL
    Mat dx1, dy1, dxy1;
    Sobel(brushfire, dx1, 5, 1, 0);
    Sobel(brushfire, dy1, 5, 0, 1);
    Sobel(brushfire, dxy1, 5, 1, 1);
    Mat angle1, mag1;
    cartToPolar(dx1, dy1, mag1, angle1);
    convertScaleAbs(mag1, mag1, 127);
    cout << "Mag" << endl;
    cout << mag1 << endl;
    print_map(mag1, "Mag");

    for (int y = 0; y < mag1.rows; y++) {
        for (int x = 0; x < mag1.cols; x++) {
            if ( (int)mag1.at<uchar>(y,x)==39 )
                mag1.at<uchar>(y,x) = 0;

            if ( (int)mag1.at<uchar>(y,x)>70 )
                mag1.at<uchar>(y,x) = 255;
        }
    }

    cout << "Thresh" << endl;
    cout << mag1 << endl;
    print_map(mag1, "Thresh");

    for (unsigned i = 0; i < walls.size(); i++) {
        mag1.at<uchar>( walls[i] ) = 255;
    }

    cout << "Walls white" << endl;
    cout << mag1 << endl;
    print_map(mag1, "Walls white");

    waitKey(0);
    return 0;
}
