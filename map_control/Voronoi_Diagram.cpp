#include "Voronoi_Diagram.h"

void Voronoi_Diagram::printMap(Mat &map) {
    Mat resizeMap;
    resize( map, resizeMap, map.size()*10, 0, 0, INTER_NEAREST);
    imshow("Map", resizeMap);
}

Voronoi_Diagram::Voronoi_Diagram() {}

Voronoi_Diagram::Voronoi_Diagram(Mat &src) {
//    Map map;
//    Mat origin_img = src.clone();
//    Mat brushfire_img = map.brushfire_img(origin_img);
//    cout << brushfire_img << endl;
//    Mat1b local_max = get_local_max(brushfire_img);
//    find_voroni_points(local_max);
//    remove_points_in_corners(brushfire_img);
//    connect_voroni_points(brushfire_img);

    Mat bw = create_binary_img(src);
    printMap(bw);
    Mat dst = distance_transform(bw);
    printMap(dst);
}

vector<Point> Voronoi_Diagram::get_points() {
    return points;
}

Mat Voronoi_Diagram::get_voroni_diagram(Mat &src) {
    Mat img = src.clone();
    Vec3b color(0, 0, 255);
    for (unsigned i = 0; i < points.size(); i++) {
        img.at<Vec3b>(points[i]) = color;
    }
    return img;
}

Mat1b Voronoi_Diagram::get_local_max(Mat &brushfire_img) {
    Mat1b kernel( Size(5,5), 1u );
    Mat img_dilate;
    dilate(brushfire_img, img_dilate, kernel);
    Mat1b local_max = ( brushfire_img >= img_dilate );
    return local_max;
}

void Voronoi_Diagram::find_voroni_points(Mat1b &img) {
    for (int y = 0; y < img.rows; y++) {
        for (int x = 0; x < img.cols; x++) {
            if ( (int)img.at<uchar>(y, x)==255 ) {
                Point p(x,y);
                points.push_back(p);
            }
        }
    }
}

void Voronoi_Diagram::remove_points_in_corners(Mat &brushfire_img) {
    for (unsigned i = 0; i < points.size(); ) {
        Point left( points[i].x-1, points[i].y );
        Point left_up( points[i].x-1, points[i].y-1 );
        Point left_down( points[i].x-1, points[i].y+1 );
        Point right( points[i].x+1, points[i].y );
        Point right_up( points[i].x+1, points[i].y-1 );
        Point right_down( points[i].x+1, points[i].y+1 );
        Point up( points[i].x, points[i].y-1 );
        Point down( points[i].x, points[i].y+1 );

        int left_val = (int)brushfire_img.at<uchar>(left);
        int right_val = (int)brushfire_img.at<uchar>(right);
        int up_val = (int)brushfire_img.at<uchar>(up);
        int down_val = (int)brushfire_img.at<uchar>(down);
        int right_down_val = (int)brushfire_img.at<uchar>(right_down);
        int left_down_val = (int)brushfire_img.at<uchar>(left_down);
        int right_up_val = (int)brushfire_img.at<uchar>(right_up);
        int left_up_val = (int)brushfire_img.at<uchar>(left_up);

        if ( left_val==0 ) {
            points.erase( points.begin()+i );
        }
        else if ( right_val==0 ) {
            points.erase( points.begin()+i );
        }
        else if ( up_val==0 ) {
            points.erase( points.begin()+i );
        }
        else if ( down_val==0 ) {
            points.erase( points.begin()+i );
        }
        else if ( right_down_val==0 ) {
            points.erase( points.begin()+i );
        }
        else if ( left_down_val==0 ) {
            points.erase( points.begin()+i );
        }
        else if ( right_up_val==0 ) {
            points.erase( points.begin()+i );
        }
        else if ( left_up_val==0 ) {
            points.erase( points.begin()+i );
        }
        else {
            i++;
        }
    }
}

void Voronoi_Diagram::connect_voroni_points(Mat &brushfire_img) {
    return;
}

Mat Voronoi_Diagram::create_binary_img(Mat &src) {
    Mat result;
    cvtColor(src, result, CV_BGR2GRAY);
    threshold(result, result, 40, 255, CV_THRESH_BINARY | CV_THRESH_OTSU);
    return result;
}

Mat Voronoi_Diagram::distance_transform(Mat &binary_img) {
    Mat result;
    distanceTransform(binary_img, result, CV_DIST_L2, 3);
    normalize(result, result, 0, 1., NORM_MINMAX);
    return result;
}
