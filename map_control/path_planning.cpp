#include "path_planning.h"

Path_planning::Path_planning() {}

Path_planning::~Path_planning() {}

int Path_planning::way_around_obstacle(Point start, Point goal, Mat &src) {
    Map grid;
    Mat img = grid.brushfire_img(src);
    if ( obstacle_detected( start, goal, img ) ) {
        LineIterator it( img, start, goal, 8 );
        vector<Point> v = get_points(it);
        Point start_left = get_p_before_obstacle(v, img);
        Point start_right = get_p_before_obstacle(v, img);
        for (int i = 0; i < 30; i++) {
            if ( !obstacle_detected(start_left, goal, img) )
                return 2;
            else if ( !obstacle_detected(start_right, goal, img) )
                return 3;
            
            go_left(start_left, img);
            go_right(start_right, img);
        }
        return 0;
    }
    else
        return 1;
}

void Path_planning::go_left(Point &p, Mat &img) {
    Point left(p.x-1, p.y);
    Point left_up(p.x-1, p.y-1);
    Point left_down(p.x-1, p.y+1);
    Point right(p.x+1, p.y);
    Point right_up(p.x+1, p.y-1);
    Point right_down(p.x+1, p.y+1);
    Point up(p.x, p.y-1);
    Point down(p.x, p.y+1);

    int left_val = (int)img.at<uchar>(left);
    int right_val = (int)img.at<uchar>(right);
    int up_val = (int)img.at<uchar>(up);
    int down_val = (int)img.at<uchar>(down);
    int right_down_val = (int)img.at<uchar>(right_down);
    int left_down_val = (int)img.at<uchar>(left_down);
    int right_up_val = (int)img.at<uchar>(right_up);
    int left_up_val = (int)img.at<uchar>(left_up);

    if ( left_up_val==0 && left_val==0 && up_val==0){
        /*****
         * ##
         * #X
         ****/
        p = down;
    }
    else if ( left_down_val==0 && left_val==0 && down_val==0 ) {
        /*****
         * #X
         * ##
         ****/
        p = right;
    }
    else if ( right_up_val==0 && right_val==0 && up_val==0) {
        /*****
         * ##
         * X#
         ****/
        p = left;
    }
    else if ( right_down_val==0 && right_val==0 && down_val==0) {
        /*****
         * X#
         * ##
         ****/
        p = up;
    }
    else if ( left_val==0 ) {
        /*****
         * #
         * #X
         * #
         ****/
        p = down;
    }
    else if ( up_val==0 ) {
        /*****
         * ###
         *  X
         ****/
        p = left;
    }
    else if ( right_val==0 ) {

        /*****
         *  #
         * X#
         *  #
         ****/
        p = up;
    }
    else if ( down_val==0 ) {
        /*****
         *  X
         * ###
         ****/
        p = right;
    }
    else if ( left_up_val==0 ) {
        /*****
         * #
         *  X
         ****/
        p = left;
    }
    else if ( left_down_val==0) {
        /*****
         *  X
         * #
         ****/
        p = down;
    }
    else if ( right_up_val==0 ) {
        /*****
         *  #
         * X
         ****/
        p = up;
    }
    else if ( right_down_val==0 ) {
        /*****
         * X
         *  #
         ****/
        p = right;
    }
}

void Path_planning::go_right(Point &p, Mat &img) {
    Point left(p.x-1, p.y);
    Point left_up(p.x-1, p.y-1);
    Point left_down(p.x-1, p.y+1);
    Point right(p.x+1, p.y);
    Point right_up(p.x+1, p.y-1);
    Point right_down(p.x+1, p.y+1);
    Point up(p.x, p.y-1);
    Point down(p.x, p.y+1);

    int left_val = (int)img.at<uchar>(left);
    int right_val = (int)img.at<uchar>(right);
    int up_val = (int)img.at<uchar>(up);
    int down_val = (int)img.at<uchar>(down);
    int right_down_val = (int)img.at<uchar>(right_down);
    int left_down_val = (int)img.at<uchar>(left_down);
    int right_up_val = (int)img.at<uchar>(right_up);
    int left_up_val = (int)img.at<uchar>(left_up);

    if ( left_up_val==0 && left_val==0 && up_val==0){
        /*****
         * ##
         * #X
         *****/
        p = right;
    }
    else if ( left_down_val==0 && left_val==0 && down_val==0 ) {
        /******
         * #X
         * ##
         * ***/
        p = up;
    }
    else if ( right_up_val==0 && right_val==0 && up_val==0) {
        /*****
         * ##
         * X#
         *****/
        p = down;
    }
    else if ( right_down_val==0 && right_val==0 && down_val==0) {
        /******
         * X#
         * ##
         *****/
        p = left;
    }
    else if ( left_val==0 ) {
        /*****
         * #
         * #X
         * #
         *****/
        p = up;
    }
    else if ( up_val==0 ) {
        /******
         * ###
         *  X
         *****/
        p = right;
    }
    else if ( right_val==0 ) {
        /****
         *  #
         * X#
         *  #
         ****/
        p = down;
    }
    else if ( down_val==0 ) {
        /****
         *  X
         * ###
         ****/
        p = left;
    }
    else if ( left_up_val==0 ) {
        /****
         * #
         *  X
         ****/
        p = up;
    }
    else if ( left_down_val==0) {
        /****
         *  X
         * #
         ****/
        p = left;
    }
    else if ( right_up_val==0 ) {
        /****
         *  #
         * X
         ****/
        p = right;
    }
    else if ( right_down_val==0 ) {
        /****
         * X
         *  #
         ****/
        p = down;
    }
}

vector<Point> Path_planning::get_points(LineIterator &it) {
    vector<Point> v(it.count);
    for (int i = 0; i < it.count; i++, it++) {
        v[i] = it.pos();
    }
    return v;
}

Point Path_planning::get_p_before_obstacle(vector<Point> &v, Mat &img) {
    Point p;
    for (size_t i = 0; i < v.size(); i++) {
        if ( (int)img.at<uchar>(v[i]) == 0 ) {
            p = v[i-1];
            break;
        }
    }
    return p;
}

// -----------------------------------------------------------------

bool Path_planning::obstacle_detected(Point start, Point goal, Mat &img) {
    LineIterator it(img, start, goal, 8);
    vector<Point> v = get_points(it);
    for (size_t i = 0; i < v.size(); i++) {
        if ( (int)img.at<uchar>(v[i])==0 ) {
            cout << v[i] << endl;
            return true;
        }
    }
    return false;
}

// -----------------------------------------------------------------

Mat Path_planning::make_visibility_map( const cv::Mat &map,
                                        const std::vector<Point> &road_map_points)
{
    Mat result = map.clone();  // Make deep copy of map

    int i = 0;
    for ( auto& point : road_map_points )
    {
        for (int y = 0; y < result.rows; y++)
            for (int x = 0; x < result.cols; x++)
            {
                Point start( point ), goal(x,y);
                obs_detect_color( start, goal, result );
            }

        result.at<Vec3b>( point ) = Vec3b(0,0,255);

        print_map( result, "Visbility" );

        i++;
        if ( i > 10 )
        {
            cv::waitKey(0);
            i = 0;
        }
    }

    for ( auto& point : road_map_points )
        result.at<Vec3b>( point ) = Vec3b(0,0,255);

    return result;
}

// -----------------------------------------------------------------

void Path_planning::obs_detect_color( const cv::Point start,
                                      const cv::Point goal,
                                      cv::Mat &img)
{
    LineIterator it( img, start, goal, 4 );

    std::vector<Point> line_points;
    for (int i = 0; i < it.count; i++, it++)
    {
        Point p( it.pos() );
        if ( img.at<Vec3b>( p ) == Vec3b(0,0,0) )
            break;
        else
            line_points.push_back( p );
    }

    for ( auto& point : line_points )
        if ( img.at<Vec3b>( point ) != Vec3b(0,0,255) )
            img.at<Vec3b>( point ) = Vec3b(50,255,0);
}

// -----------------------------------------------------------------

void Path_planning::print_map(const cv::Mat &img, const string &s)
{
    Mat resizeMap;
    resize( img, resizeMap, img.size()*10, 0, 0, INTER_NEAREST);
    imshow(s, resizeMap);
}

// -----------------------------------------------------------------
