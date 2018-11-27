#ifndef A_STAR_H
#define A_STAR_H

#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/ximgproc.hpp"

#include <iostream>
#include <vector>

using namespace std;
using namespace cv;

/***************************************
 * Constants
 ***************************************/
const int ALLOW_VERTEX_PASSTHROUGH = 1;
const int DRAW_OPEN_LIST = 1;

const int NODE_FLAG_CLOSED = -1;
const int NODE_FLAG_UNDEFINED = 0;
const int NODE_FLAG_OPEN = 1;

const int NODE_TYPE_ZERO = 0;
const int NODE_TYPE_OBSTACLE = 1;
const int NODE_TYPE_START = 2;
const int NODE_TYPE_END = 3;

const int G_DIRECT = 10;
const int G_SKEW = 14;

class Map_Node {
    public:
        int x = -1, y = -1, h = 0, g = 0;
        int type = NODE_TYPE_ZERO, flag = NODE_FLAG_UNDEFINED;
        Map_Node *parent = 0;
        Map_Node() {}
        Map_Node(int x,
                int y,
                int type = NODE_TYPE_ZERO,
                int flag = NODE_FLAG_UNDEFINED,
                Map_Node *parent = 0) {
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

class Map_Size {
    public:
        unsigned long width = 0;
        unsigned long height = 0;
        unsigned long size = 0;

        Map_Size() {}

        Map_Size( unsigned long width, unsigned long height ) {
            this->width = width;
            this->height = height;
            this->size = width * height;
        }
};

class A_Star {
    public:
        A_Star();

        /**
         * @brief get_path
         * @param road_map =>
         *      Must be a roadmap => (Voronoi or boustrophedon decomposition)
         * @param start
         * @param goal
         * @return
         */
        vector<Point> get_path( const cv::Mat &road_map,
                                const cv::Point &start,
                                const cv::Point &goal);

        /**
         * @brief draw_path
         * @param img
         * @param path
         */
        void draw_path(cv::Mat &img, const std::vector<Point> path );

        ~A_Star();

    private:
        Mat map;
        vector<Map_Node *> open_list;
        Map_Node *start_node, *goal_node;
        Map_Size map_size;
        vector<Map_Node> map_data;

        void print_map( const cv::Mat &img,
                        const string &s);

        /**
         * @brief manhatten_distance
         * @param node1
         * @param node2
         * @return
         */
        int manhatten_dist( const Map_Node *node1,
                            const Map_Node *node2 );

        /**
         * @brief diagonal_dist
         * @param node1
         * @param node2
         * @return
         */
        int diagonal_dist( const Map_Node *node1,
                           const Map_Node *node2 );
        /**
         * @brief compute_h
         * @param node1
         * @param node2
         * @return
         */
        int compute_h( const Map_Node *node1,
                       const Map_Node *node2 );
        /**
         * @brief compute_g
         * @param node1
         * @param node2
         * @return
         */
        int compute_g( const Map_Node *node1,
                       const Map_Node *node2 );
        /**
         * @brief map_at
         * @param x
         * @param y
         * @return
         */
        Map_Node *map_at( const int x, const int y );

        /**
         * @brief neighbors
         * @param node
         * @return
         */
        std::vector<Map_Node *> neighbors( const Map_Node *node );

        /**
         * @brief find
         * @return
         */
        std::vector<Map_Node *> find();

        /**
         * @brief draw_open_list
         */
        void draw_open_list(Mat &img);
};

#endif // A_STAR_H
