#include "Voronoi_Diagram.h"

Voronoi_Diagram::Voronoi_Diagram() {}

Voronoi_Diagram::Voronoi_Diagram(Mat &src) {
    watershed_algorithm(src);
}

void Voronoi_Diagram::printMap(Mat &map, string s) {
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

void Voronoi_Diagram::watershed_algorithm(Mat &src) {
    // Create binary image from src img
    Mat bw_img;
    cvtColor(src, bw_img, CV_BGR2GRAY);
    threshold(bw_img, bw_img, 40, 255, CV_THRESH_BINARY | CV_THRESH_OTSU);

    // Perform the distance transform algorithm
    Mat dist;
    distanceTransform(bw_img, dist, CV_DIST_L2, 3);

    // Normalize the distance image for range={0.0, 1.0}
    normalize(dist, dist, 0, 1., NORM_MINMAX);
    brushfire_grid = dist.clone();

    // Threshold to obtain peaks
    threshold(dist, dist, .4, 1., CV_THRESH_BINARY);

    // Dilate a bit the dist image
    Mat kernel1 = Mat::ones(1, 1, CV_8UC1);
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
