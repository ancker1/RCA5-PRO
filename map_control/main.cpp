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

Vec3b red(0,0,255), black(0,0,0), white(255,255,255), blue(255,0,0);

// CONSTANTS
Mat big_map = cv::imread( "../map_control/big_floor_plan.png", IMREAD_COLOR);
Mat small_map = cv::imread( "../map_control/floor_plan.png", IMREAD_COLOR );

const int ALLOW_VERTEX_PASSTHROUGH = 0;
const int NODE_FLAG_CLOSED = -1;
const int NODE_FLAG_UNDEFINED = 0;
const int NODE_FLAG_OPEN = 1;

const int NODE_TYPE_ZERO = 0;
const int NODE_TYPE_OBSTACLE = 1;
const int NODE_TYPE_START = 2;
const int NODE_TYPE_END = 3;

const int G_DIRECT = 10;
const int G_SKEW = 14;

class MapNode {
    public:
        int x = -1, y = -1, h = 0, g = 0;
        int type = NODE_TYPE_ZERO, flag = NODE_FLAG_UNDEFINED;
        MapNode *parent = 0;
        MapNode() {}
        MapNode(int x,
                int y,
                int type = NODE_TYPE_ZERO,
                int flag = NODE_FLAG_UNDEFINED,
                MapNode *parent = 0) {
            this->x = x;
            this->y = y;
            this->type = type;
            this->flag = flag;
            this->parent = parent;
        }

        int f() {
            return g + h;
        }
};

class MapSize {
    public:
        unsigned long width = 0;
        unsigned long height = 0;
        unsigned long size = 0;

        MapSize() {}

        MapSize( unsigned long width, unsigned long height ) {
            this->width = width;
            this->height = height;
            this->size = width * height;
        }
};

// VARIABLES
vector<MapNode *> open_list;
MapNode *start_node, *goal_node;
MapSize map_size;
vector<MapNode> map_data;
Mat map_a_star = big_map.clone();

int manhatten_distance( MapNode* node1, MapNode* node2 ) {
    return abs( node2->x - node1->x ) + abs( node2->y - node1->y );
}

int diagonal_distance( MapNode* node1, MapNode* node2 ) {
    return max( abs(node2->x - node1->x ), abs( node2->y - node1->y ) );
}

int compute_h( MapNode *node1, MapNode *node2 ) {
    if ( ALLOW_VERTEX_PASSTHROUGH )
        return diagonal_distance( node1, node2 ) * G_SKEW;
    else
        return manhatten_distance( node1, node2 ) * G_DIRECT;
}

int compute_g( MapNode *node1, MapNode *node2 ) {
    int dx = abs( node1->x - node2->x );
    int dy = abs( node1->y - node2->y );
    if ( dx > dy )
        return G_SKEW * dy + G_DIRECT * ( dx - dy );
    else
        return G_SKEW * dx + G_DIRECT * ( dy - dx );
}

MapNode *map_at( int x, int y ) {
    if ( x < 0 || y < 0 ||
         x >= (int)map_size.width ||
         y >= (int)map_size.height )
        return 0;

    return &map_data[ y * map_size.width + x ];
}

vector<MapNode *> neighbors( MapNode *node ) {
    vector<MapNode *> available;
    MapNode *_node;

    // L
    _node = map_at( node->x-1, node->y );
    if ( _node != 0 )
        available.push_back( _node );

    // T
    _node = map_at( node->x, node->y-1 );
    if ( _node != 0 )
        available.push_back( _node );

    // R
    _node = map_at( node->x+1, node->y );
    if ( _node != 0 )
        available.push_back( _node );

    // B
    _node = map_at( node->x, node->y+1 );
    if ( _node != 0 )
        available.push_back( _node );

    // LT
    _node = map_at( node->x-1, node->y-1 );
    if ( _node != 0 )
        available.push_back( _node );

    // RT
    _node = map_at( node->x+1, node->y-1 );
    if ( _node != 0 )
        available.push_back( _node );

    // RB
    _node = map_at( node->x+1, node->y+1 );
    if ( _node != 0 )
        available.push_back( _node );

    // LB
    _node = map_at( node->x-1, node->y+1 );
    if ( _node != 0 )
        available.push_back( _node );

    return available;
}

void draw_open_list() {
    for ( auto o : open_list ) {
        MapNode *n = o;
        if ( ( n == start_node ) || ( n == goal_node ) )
            continue;
        map_a_star.at<Vec3b>(n->y, n->x) = Vec3b(210,210,210);
    }
}

vector<MapNode *> find() {
    vector<MapNode *> path;
    MapNode *node, *reversed_ptr = 0;

    while ( open_list.size() > 0 ) {
        node = open_list.at(0);

        for ( auto& n : open_list) {
            if ( ( n->f() <= node->f() ) && ( n->h < node->h ) )
                node = n;
        }

        open_list.erase( remove( open_list.begin(), open_list.end(), node ), open_list.end() );

        node->flag = NODE_FLAG_CLOSED;

        if ( node == goal_node ) {
            reversed_ptr = node;
            break;
        }

        vector<MapNode *> neighbor_nodes = neighbors( node );

        for ( auto& n : neighbor_nodes ) {
            if ( ( n->flag == NODE_FLAG_CLOSED ) ||
                 ( n->type == NODE_TYPE_OBSTACLE ))
                continue;

            int g = node->g + compute_g( n, node);

            if ( ( n->flag == NODE_FLAG_UNDEFINED ) ||
                 ( g < n->g ) ) {
                n->g = g;
                n->h = compute_h( n, goal_node );
                n->parent = node;
                if ( n->flag != NODE_FLAG_OPEN ) {
                    n->flag = NODE_FLAG_OPEN;
                    open_list.push_back( n );
                }
            }
        }
        draw_open_list();
        if ( open_list.size() <= 0 )
            break;
    }

    if ( reversed_ptr == 0 ) {
        cout << "Target node is unreachable." << endl;
    }
    else {
        MapNode *_node = reversed_ptr;

        while ( _node->parent != 0 ) {
            path.push_back( _node );
            _node = _node->parent;
        }

        reverse( path.begin(), path.end() );
    }

    return path;
}

void drawPath(Mat &map, vector<MapNode *> path) {
    cvtColor(map, map, COLOR_BGR2HSV);
    for (int i = 0; i < (int)path.size() - 1; i++) {
        MapNode *node = path[i];
        map.at<Vec3b>(node->y, node->x) = Vec3b(20 + (1.0 - ((double) i / path.size())) * 80, 200, 255);
        cout << "->(" << node->x << "," << node->y << ")";
    }
    cout << endl;

    cvtColor(map, map, COLOR_HSV2BGR);
}

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

    src.at<Vec3b>( points[0] ) = Vec3b(255,0,0); // Start point
    src.at<Vec3b>( points[ points.size()-1 ] ) = Vec3b(0,255,0); // End point

    map_size = MapSize( src.cols, src.rows );
    map_data = vector<MapNode>( map_size.size );
    for (int y = 0; y < src.rows; y++) {
        for (int x = 0; x < src.cols; x++) {

            if ( src.at<Vec3b>(y,x) == red )
                map_data[ y * map_size.width + x ] = MapNode( x, y, NODE_TYPE_ZERO );
            else if ( src.at<Vec3b>(y,x) == Vec3b(255,0,0) ) {
                MapNode n( x, y, NODE_TYPE_START );
                map_data[ y * map_size.width + x ] = n;
                start_node = &map_data[ y * map_size.width + x ];
            }
            else if ( src.at<Vec3b>(y,x) == Vec3b(0,255,0) ) {
                MapNode n( x, y, NODE_TYPE_END );
                map_data[ y * map_size.width + x ];
                goal_node = &map_data[ y * map_size.width + x ];
            }
            else {
                map_data[ y * map_size.width + x ] = MapNode( x, y, NODE_TYPE_OBSTACLE);
            }
        }
    }

    open_list.push_back( start_node );
    vector<MapNode *> path = find();
    drawPath( map_a_star, path);
    print_map( map_a_star, "A-star Voronoi" );

    Mat img = big_map.clone();
    cvtColor( img, img, CV_BGR2GRAY);
    Map map(img);
    map.trapezoidalLines( map.cornerDetection() );
    map.calculateCells( map.getUpperTrapezoidalGoals(), map.getLowerTrapezoidalGoals() );

    waitKey(0);
    return 0;
}
