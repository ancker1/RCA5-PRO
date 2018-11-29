#include "A_Star.h"

// -----------------------------------------------------------------------

A_Star::A_Star() {}

// -----------------------------------------------------------------------

A_Star::~A_Star() {}

// -----------------------------------------------------------------------

std::vector<cv::Point> A_Star::get_path(const cv::Mat &road_map,
                                        const cv::Point &start,
                                        const cv::Point &goal) {
    map = road_map.clone();
    map.at<Vec3b>( start ) = Vec3b( 255, 0, 0 );    // Start point = blue
    map.at<Vec3b>( goal ) = Vec3b( 0, 255, 0 );     // End point  = green

    map_size = Map_Size( map.cols, map.rows );
    map_data = vector<Map_Node>( map_size.size );

    for (int y = 0; y < map.rows; y++)
        for (int x = 0; x < map.cols; x++) {
            if ( map.at<Vec3b>(y,x) == Vec3b(0,0,255) )
                map_data[ y * map_size.width + x ] = Map_Node( x, y, NODE_TYPE_ZERO );
            else if ( map.at<Vec3b>(y,x) == Vec3b(255,0,0) ) {
                Map_Node n( x, y, NODE_TYPE_START );
                map_data[ y * map_size.width + x ] = n;
                start_node = &map_data[ y * map_size.width + x ];
            }
            else if ( map.at<Vec3b>(y,x) == Vec3b(0,255,0) ) {
                Map_Node n( x, y, NODE_TYPE_END );
                map_data[ y * map_size.width + x ];
                goal_node = &map_data[ y * map_size.width + x ];
            }
            else {
                map_data[ y * map_size.width + x ] = Map_Node( x, y, NODE_TYPE_OBSTACLE);
            }
        }

    open_list.push_back( start_node );
    vector<Map_Node *> path = find();

    std::vector<cv::Point> result;
    for ( auto p : path ) {
        result.push_back( Point( p->x, p->y ) );
    }

    return result;
}

// -----------------------------------------------------------------------

int A_Star::manhatten_dist(const Map_Node *node1, const Map_Node *node2) {
    return abs( node2->x - node1->x ) + abs( node2->y - node1->y );
}

// -----------------------------------------------------------------------

int A_Star::diagonal_dist(const Map_Node *node1, const Map_Node *node2) {
    return max( abs(node2->x - node1->x ), abs( node2->y - node1->y ) );
}

// -----------------------------------------------------------------------

int A_Star::compute_h(const Map_Node *node1, const Map_Node *node2) {
    if ( ALLOW_VERTEX_PASSTHROUGH )
            return diagonal_dist( node1, node2 ) * G_SKEW;
        else
            return manhatten_dist( node1, node2 ) * G_DIRECT;
}

// -----------------------------------------------------------------------

int A_Star::compute_g(const Map_Node *node1, const Map_Node *node2) {
    int dx = abs( node1->x - node2->x );
    int dy = abs( node1->y - node2->y );
    if ( dx > dy )
        return G_SKEW * dy + G_DIRECT * ( dx - dy );
    else
        return G_SKEW * dx + G_DIRECT * ( dy - dx );
}

// -----------------------------------------------------------------------

Map_Node *A_Star::map_at(const int x, const int y) {
    if ( x < 0 || y < 0 ||
         x >= (int)map_size.width ||
         y >= (int)map_size.height ) {
        return 0;
    }

    return &map_data[ y * map_size.width + x ];
}

// -----------------------------------------------------------------------

std::vector<Map_Node *> A_Star::neighbors(const Map_Node *node) {
    std::vector<Map_Node *> available;
    Map_Node *_node;

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

// -----------------------------------------------------------------------

std::vector<Map_Node *> A_Star::find() {
    vector<Map_Node *> path;
    Map_Node *node, *reversed_ptr = 0;

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

        vector<Map_Node *> neighbor_nodes = neighbors( node );

        for ( auto& n : neighbor_nodes ) {
            if ( ( n->flag == NODE_FLAG_CLOSED ) ||
                 ( n->type == NODE_TYPE_OBSTACLE ))
                continue;

            int g = node->g + compute_g( n, node);

            if ( ( n->flag == NODE_FLAG_UNDEFINED ) || ( g < n->g ) ) {
                n->g = g;
                n->h = compute_h( n, goal_node );
                n->parent = node;
                if ( n->flag != NODE_FLAG_OPEN ) {
                    n->flag = NODE_FLAG_OPEN;
                    open_list.push_back( n );
                }
            }
        }

        if ( open_list.size() <= 0 )
            break;
    }

    if ( reversed_ptr == 0 ) {
        cout << "Target node is unreachable." << endl;
    }
    else {
        Map_Node *_node = reversed_ptr;

        while ( _node->parent != 0 ) {
            path.push_back( _node );
            _node = _node->parent;
        }

        reverse( path.begin(), path.end() );
    }

    return path;
}

// -----------------------------------------------------------------------

void A_Star::draw_path( cv::Mat &img,
                        const std::vector<cv::Point> path) {
    for ( auto& p : path ) {
        img.at<Vec3b>( p ) = Vec3b( 0, 0, 255 );
    }
}

vector<double> A_Star::findAstarPathLengthsForRoadmap(Mat roadmap)
{
    vector<double> pathLengths;
    double tempDist;
    vector<Point> aStarPath;
    Point startPoint = Point(0,0);
    Point endPoint = Point(roadmap.cols, roadmap.rows);
    Point startPointOnRoadmap;
    Point endPointOnRoadmap;
    int startPointi = 0;
    int startPointj = 0;
    int endPointi = roadmap.rows - 1;
    int endPointj = roadmap.cols - 1;
    cout << "TESET: " << findStartWithoutObs(roadmap, 0, 101) << endl;
    // Calculation of distance over all points on map
    /*
    while ( (startPointi != endPointi) && (startPointj != endPointj) )
    {
        cout << "Starti: " << startPointi << " Startj: " << startPointj << endl;
        cout << "Endi: " << endPointi << " Endj: " << endPointj << endl;
        //cout << "Start Point " << startPoint << " End Point " << endPoint << endl;
        //startPoint = findStartWithoutObs(roadmap, startPointi, startPointj);
        //endPoint = findGoalWithoutObs(roadmap, endPointi, endPointj);
        startPointOnRoadmap = findWayToRoadMap(roadmap, startPoint);
        endPointOnRoadmap = findWayToRoadMap(roadmap, endPoint);
        if ( startPointOnRoadmap == Point(roadmap.cols + 1, roadmap.rows + 1) || roadmap.at<Vec3b>(startPointi, startPointj) == Vec3b(0,0,0) ) // Have not found a point without obstacle to roadmap
        {
            if(startPointi == roadmap.rows - 1)
                startPointi = 0;
            if(startPointj == roadmap.cols - 1)
            {
                startPointj = 0;
                startPointi++;
            }
            else
                startPointj++;
        }
        else if (  endPointOnRoadmap == Point(roadmap.cols + 1, roadmap.rows + 1) || roadmap.at<Vec3b>(endPointi, endPointj) == Vec3b(0,0,0) ) // Have not found a point without obstacle to roadmap
        {
            if(endPointi == 0)
                endPointi = roadmap.rows - 1 ;
            if(endPointj == 0)
            {
                endPointj = roadmap.cols - 1 ;
                endPointi--;
            }
            else
                endPointj--;
        }
        else // Have found a startpoint and a endpoint without obs therefore calculate distance
        {
            tempDist = 0;
            aStarPath = get_path(roadmap, startPointOnRoadmap, endPointOnRoadmap);
            tempDist = calculateDiagonalDist(startPoint, startPointOnRoadmap); // From start to start on roadmap
            tempDist += tempDist += calculateDiagonalDist(endPoint, endPointOnRoadmap); // From goal to goal on roadmap
            tempDist += aStarPath.size(); // Path length of astar
            pathLengths.push_back(tempDist);
            // Next point for start and end
            if(startPointi == roadmap.rows - 1)
                startPointi = 0;
            if(startPointj == roadmap.cols - 1)
            {
                startPointj = 0;
                startPointi++;
            }
            else
                startPointj++;

            if(endPointi == 0)
                endPointi = roadmap.rows - 1;
            if(endPointj == 0)
            {
                endPointj = roadmap.cols - 1;
                endPointi--;
            }
            else
                endPointj--;
        }
    }
    */

    for(int i = 0; i < roadmap.rows; i++)
    {
        for(int j = 0; j < roadmap.cols; j++) // i j is startpoint
        {
            cout << j << endl;
            if(roadmap.at<Vec3b>(i, j) == Vec3b(0,0,0)) // If black pixel inside obstacle find new start point
                continue;
            startPointOnRoadmap = findWayToRoadMap(roadmap, Point(j,i));
            if(startPointOnRoadmap == Point(roadmap.cols + 1, roadmap.rows + 1)) // Have not found a starting point without obstacle to roadmap
                continue;

            for(int k = i + 1; k < roadmap.rows; k++)
            {
                for(int n = j + 1; n < roadmap.cols; n++ ) // k n is goalpoint
                {
                    if(roadmap.at<Vec3b>(k, n) == Vec3b(0,0,0)) // If black pixel inside obstacle find new goal point
                        continue;
                    endPointOnRoadmap = findWayToRoadMap(roadmap, Point(n,k));
                    if(startPointOnRoadmap == Point(roadmap.cols + 1, roadmap.rows + 1)) // Have not found a goal point without obstacle to roadmap
                        continue;
                    // Have found a startpoint and a endpoint without obs therefore calculate distance
                    tempDist = 0;
                    //aStarPath = get_path(roadmap, startPointOnRoadmap, endPointOnRoadmap);
                    tempDist = calculateDiagonalDist(Point(j,i), startPointOnRoadmap); // From start to start on roadmap
                    tempDist += tempDist += calculateDiagonalDist(Point(n,k), endPointOnRoadmap); // From goal to goal on roadmap
                    //tempDist += aStarPath.size(); // Path length of astar
                    pathLengths.push_back(tempDist);
                }
            }

        }
        //cout << i << endl;
    }

    return pathLengths;
}

// -----------------------------------------------------------------------

void A_Star::draw_open_list( cv::Mat &img ) {
    for ( auto& o : open_list ) {
        Map_Node *n = o;
        if ( ( n == start_node ) || ( n == goal_node ) )
            continue;
        img.at<Vec3b>(n->y, n->x) = Vec3b(210,210,210);
    }
}

// -----------------------------------------------------------------------

void A_Star::print_map(const Mat &img, const string &s) {
    cv::Mat resize_img;
    resize( img, resize_img, img.size()*7, 0, 0, INTER_NEAREST );
    imshow( s, resize_img );
}


Point A_Star::findWayToRoadMap(Mat roadmap, Point entryExitPoint)
{
    Point roadmapEntryExit = Point(roadmap.cols + 1, roadmap.rows + 1); // Must initlize other than 0
    for(int i = 0; i < roadmap.rows; i++)
    {
        for(int j = 0; j < roadmap.cols; j++)
        {
            if(roadmap.at<Vec3b>(i,j) == Vec3b(0,0,255))
            {
                if(!obstacleDetectedWithLine(roadmap, Point(j,i), entryExitPoint))
                {
                    if(calculateDiagonalDist(roadmapEntryExit, entryExitPoint) > calculateDiagonalDist(Point(j,i), entryExitPoint))
                        roadmapEntryExit = Point(j,i);
                }
            }
        }
    }

    return roadmapEntryExit;
}

double A_Star::calculateDiagonalDist(Point p1, Point p2)
{
    return sqrt(pow(p1.x-p2.x,2) + pow(p1.y-p2.y,2));
}



Point A_Star::findStartWithoutObs(Mat roadmap, int starti, int startj)
{
    Point start = Point(0,0);
    bool foundFirst = false;
    // Find First Starting point without obstacle
    for(int i = starti; i < roadmap.rows; i++)
    {
        for(int j = startj; j < roadmap.cols; j++)
        {
            if(roadmap.at<Vec3b>(i,j) != Vec3b(0,0,0)) // If not black
            {
                foundFirst = true;
                start = Point(j,i);
                break;
            }
        }
        if(foundFirst)
            break;
    }
    return start;
}

Point A_Star::findGoalWithoutObs(Mat roadmap, int endi, int endj)
{
    Point goal = Point(0,0);
    bool foundFirst = false;
    // Finds First ending point without obstacle
    for(int i = endi; i > 0; i--)
    {
        for(int j = endj; j > 0; j--)
        {
            if(roadmap.at<Vec3b>(i,j) != Vec3b(0,0,0)) // If not black
            {
                foundFirst = true;
                goal = Point(j,i);
                break;
            }
        }
        if(foundFirst)
            break;
    }
    return goal;
}

vector<Point> A_Star::get_points(LineIterator &it)
{
    vector<Point> v(it.count);
    for (int i = 0; i < it.count; i++, it++) {
        v[i] = it.pos();
    }
    return v;
}

bool A_Star::obstacleDetectedWithLine(Mat roadmap,Point start, Point end)
{
    LineIterator it(roadmap, start, end, 8);
    vector<Point> v = get_points(it);
    for (size_t i = 0; i < v.size(); i++) {
        if ( roadmap.at<Vec3b>(v[i]) == Vec3b(0,0,0) ) // MEETS BLACK
        {
            return true;
        }
    }
    return false;
}

// -----------------------------------------------------------------------
