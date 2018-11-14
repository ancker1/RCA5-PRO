#include "Voronoi_Diagram.h"

Voronoi_Diagram::Voronoi_Diagram() {}

Voronoi_Diagram::Voronoi_Diagram(Mat &src) {
    source = src.clone();
    watershed_algorithm(src);
    make_voronoi_points();
}

void Voronoi_Diagram::print_map(Mat &map, string s) {
    Mat resizeMap;
    resize( map, resizeMap, map.size()*10, 0, 0, INTER_NEAREST);
    imshow(s, resizeMap);
}

Mat Voronoi_Diagram::get_brushfire_grid() {
    return brushfire_grid;
}

Mat Voronoi_Diagram::get_rooms_map() {
    return rooms_map;
}

Mat Voronoi_Diagram::get_binary_img() {
    return binary_img;
}

vector<Point> Voronoi_Diagram::get_voronoi_points() {
    return voronoi_points;
}

void Voronoi_Diagram::watershed_algorithm(Mat &src) {
    // Create binary image from src img
    Mat bw_img;
    cvtColor(src, bw_img, CV_BGR2GRAY);
    threshold(bw_img, bw_img, 40, 255, CV_THRESH_BINARY | CV_THRESH_OTSU);
    binary_img = bw_img.clone();

    // Perform the distance transform algorithm
    Mat dist;
    distanceTransform(bw_img, dist, CV_DIST_L2, 3);

    // Normalize the distance image for range={0.0, 1.0}
    normalize(dist, dist, 0, 1., NORM_MINMAX);
    brushfire_grid = dist.clone();

    // Threshold to obtain peaks
    threshold(dist, dist, .4, 1., CV_THRESH_BINARY);

    // Dilate a bit the dist image
    Mat kernel1 = Mat::ones(1, 2, CV_8UC1);
    dilate(dist, dist, kernel1);

    // Create the CV_8U version of the distance image
    Mat dist_8u;
    dist.convertTo(dist_8u, CV_8U);

    // Find total markers
    vector<vector<Point>> contours;
    findContours(dist_8u, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

    // Create the marker image for the watershed algorithm
    Mat markers = Mat::zeros(dist_8u.size(), CV_32SC1);

    // Draw the foreground markers
    for (size_t i = 0; i < contours.size(); i++) {
        drawContours(markers, contours, static_cast<int>(i), Scalar::all(static_cast<int>(i)+1), -1);
    }

    // Perform hte watershed algorithm
    watershed(src, markers);

    Mat mark = Mat::zeros(markers.size(), CV_8UC1);
    markers.convertTo(mark, CV_8UC1);
    bitwise_not(mark, mark);

    // Generate random colors
    vector<Vec3b> colors;
    for (size_t i = 0; i < contours.size(); i++) {
        int b = theRNG().uniform(0, 255);
        int g = theRNG().uniform(0, 255);
        int r = theRNG().uniform(0, 255);
        colors.push_back(Vec3b( (uchar)b, (uchar)g, (uchar)r ));
    }

    // Create the result img
    Mat result = Mat::zeros(markers.size(), CV_8UC3);

    // Fill labeled objects with random colors
    for (int i = 0; i < markers.rows; i++) {
        for (int j = 0; j < markers.cols; j++) {
            int index = markers.at<int>(i,j);
            if ( index>0 && index<=static_cast<int>(contours.size()) ) {
                result.at<Vec3b>(i,j) = colors[ index-1 ];
            }
            else {
                result.at<Vec3b>(i,j) = Vec3b(0,0,0);
            }
        }
    }

    rooms_map = result.clone();
}

void Voronoi_Diagram::make_voronoi_points() {
    Vec3b red(0,0,255), black(0,0,0), white(255,255,255);
    Mat img = brushfire_grid.clone();

    // Get walls for later use
    Mat b_img;
    convertScaleAbs(img, b_img, 255);
    vector<Point> walls;
    for (int y = 0; y < b_img.rows; y++) {
        for (int x = 0; x < b_img.cols; x++) {
            if ( (int)b_img.at<uchar>(y,x)==0 ) {
                walls.push_back( Point(x,y) );
            }
        }
    }

    // differentiate brushfire_img
    Mat dx, dy;
    Sobel(img, dx, 5, 1, 0);
    Sobel(img, dy, 5, 0, 1);
    Mat angle, mag;
    cartToPolar(dx, dy, mag, angle);
    convertScaleAbs(mag, mag, 127);

    // Threshold
    for (int y = 0; y < mag.rows; y++) {
        for (int x = 0; x < mag.cols; x++) {
            if ( (int)mag.at<uchar>(y,x)==39 )
                mag.at<uchar>(y,x) = 0;

            if ( (int)mag.at<uchar>(y,x)>70 )
                mag.at<uchar>(y,x) = 255;
        }
    }

    // Remove walls for img
    for (unsigned i = 0; i < walls.size(); i++) {
        mag.at<uchar>( walls[i] ) = 255;
    }

    // Get all points which is not white(255)
    vector<Point> v;
    for (int y = 0; y < mag.rows; y++) {
        for (int x = 0; x < mag.cols; x++) {
            if ( (int)mag.at<uchar>(y,x)!=255 )
                v.push_back( Point(x,y) );
        }
    }

    for (unsigned i = 0; i < v.size(); i++) {
        source.at<Vec3b>( v[i] ) = red;
    }

    for (int y = 0; y < source.rows; y++) {
        for (int x = 0; x < source.cols; x++) {
            if ( source.at<Vec3b>(y,x)==red ) {
                Vec3b left = source.at<Vec3b>(y,x-1);
                Vec3b right = source.at<Vec3b>(y,x+1);
                Vec3b up = source.at<Vec3b>(y-1,x);
                Vec3b down = source.at<Vec3b>(y+1,x);
                Vec3b down_right = source.at<Vec3b>(y+1,x+1);
                Vec3b down_left = source.at<Vec3b>(y+1,x-1);
                Vec3b up_right = source.at<Vec3b>(y-1,x+1);
                Vec3b up_left = source.at<Vec3b>(y-1,x-1);
                if ( left==black || right==black || up==black ||
                     down==black || down_right==black ||
                     down_left==black ||up_right==black ||
                     up_left==black) {
                    source.at<Vec3b>(y,x)=white;
                }
            }
        }
    }

    voronoi_points.clear();
    for (int y = 0; y < source.rows; y++) {
        for (int x = 0; x < source.cols; x++) {
            if ( source.at<Vec3b>(y,x)==red ) {
                voronoi_points.push_back( Point(x,y) );
            }
        }
    }
}
