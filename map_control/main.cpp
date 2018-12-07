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

static double angle(Point pt1, Point pt2, Point pt0)
{
    double result;
    double dx1 = pt1.x - pt0.x;
    double dy1 = pt1.y - pt0.y;
    double dx2 = pt2.x - pt0.x;
    double dy2 = pt2.y - pt0.y;
    result = (dx1 * dx2 + dy1 * dy2) / sqrt((dx1 * dx1 + dy1 * dy1) * (dx2 * dx2 + dy2 * dy2 ) + 1e-10);
    return result;
}

static void findSquares( const cv::Mat &img, vector<vector<Point>> &squares)
{
    squares.clear();

    int thresh = 50, N = 11;

    Mat pyr, timg, gray0(img.size(), CV_8U), gray;

    // downscale and upscale the image to filter out the noise
    pyrDown(img, pyr, Size(img.cols/2, img.rows/2));
    pyrUp(pyr, timg, img.size());

    // Find squares in the image
    vector<vector<Point>> contours;
    for (int c = 0; c < 3; c++)
    {
        int ch[] = {c, 0};
        mixChannels(&timg, 1, &gray0, 1, ch, 1);

        // Try several threshold levels
        for (int l = 0; l < N; l++)
        {
            if (l == 0)
            {
                Canny(gray0, gray, 0, thresh, 5);
                dilate(gray, gray, Mat(), Point(-1,-1));

                // TEST
                print_map(gray, "test");
                cv::waitKey(0);
            }
            else
            {
                gray = gray0 >= (l + 1) * 255/N;    // Apply threshold if l != 0

                // TEST
                print_map(gray, "test");
                cv::waitKey(0);
            }

            // Find contours and store them all as a list
            findContours(gray, contours, CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE);

            vector<Point> approx;

            // Test each contour
            for (size_t i = 0; i < contours.size(); i++)
            {
                // Approximate contour with accuracy proportional to the contour perimeter
                approxPolyDP(contours[i], approx, arcLength(contours[i], true) * 0.02, true);

                if ( (approx.size() == 4) &&
                     (fabs(contourArea(approx)) > 1000) &&
                     (isContourConvex(approx)) )
                {
                    double maxCosine = 0;

                    for (int j = 2; j < 5; j++)
                    {
                        // Find the maximum cosine of the angle between joint edges
                        double cosine = fabs(angle(approx[j % 4], approx[j - 2], approx[j - 1]));
                        maxCosine = MAX(maxCosine, cosine);
                    }

                    if (maxCosine < 0.3)
                        squares.push_back(approx);
                }
            }
        }
    }
}

static void drawSquares( Mat &img, const vector<vector<Point>> &squares )
{
    for (size_t i = 0; i < squares.size(); i++)
    {
        const Point * p = &squares[i][0];
        int n = (int)squares[i].size();
        polylines(img, &p, &n, 1, true, Scalar(0, 255, 0), 3, LINE_AA);
    }

    print_map(img, "Squares");
}

int main( ) {
    Mat big_map = cv::imread( "../map_control/big_floor_plan.png", IMREAD_COLOR);
    Mat big_map2 = cv::imread( "../map_control/big_floor_test.png", IMREAD_COLOR );
    Mat big_map3 = cv::imread( "../map_control/big_floor_test2.png", IMREAD_COLOR );
    Mat small_map = cv::imread( "../map_control/floor_plan.png", IMREAD_COLOR );

    Mat src = small_map.clone();

    Path_planning *pp = new Path_planning();

    int result = pp->way_around_obstacle(Point(2, 2), Point(14, 2), src);

    cout << result << endl;

    waitKey(0);
    return 0;
}
