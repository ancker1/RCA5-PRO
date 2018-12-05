#include "A_Star.h"

// -----------------------------------------------------------------------

A_Star::A_Star() {}

// -----------------------------------------------------------------------

A_Star::~A_Star() {}

A_Star::A_Star(const cv::Mat &draw)
{
    map_a_star = draw.clone();
}

// -----------------------------------------------------------------------

std::vector<cv::Point> A_Star::get_path(const cv::Mat &road_map,
                                        const cv::Point &start,
                                        const cv::Point &goal) {
    open_list.clear();
    map = road_map.clone();
    map.at<Vec3b>( start ) = Vec3b( 255, 0, 0 );    // Start point = blue
    map.at<Vec3b>( goal ) = Vec3b( 0, 255, 0 );     // End point  = green

    map_size = Map_Size( map.cols, map.rows );
    map_data = vector<Map_Node>( map_size.size );

    for (int y = 0; y < map.rows; y++)
        for (int x = 0; x < map.cols; x++)
        {
            if ( map.at<Vec3b>(y,x) == Vec3b(0,0,255) ) // If pixel red => roadmap
            {
                map_data[ y * map_size.width + x ] = Map_Node( x, y, NODE_TYPE_ZERO );
            }
            else if ( map.at<Vec3b>(y,x) == Vec3b(255,0,0) )    // If pixel blue => start point
            {
                Map_Node n( x, y, NODE_TYPE_START );
                map_data[ y * map_size.width + x ] = n;
                start_node = &map_data[ y * map_size.width + x ];
            }
            else if ( map.at<Vec3b>(y,x) == Vec3b(0,255,0) )    // If pixel green => end point
            {
                Map_Node n( x, y, NODE_TYPE_END );
                map_data[ y * map_size.width + x ] = n;
                goal_node = &map_data[ y * map_size.width + x ];
            }
            else    // If not red, green or blue then obstacle
            {
                map_data[ y * map_size.width + x ] = Map_Node( x, y, NODE_TYPE_OBSTACLE);
            }
        }

    open_list.push_back( start_node );
    vector<Map_Node *> path = find();

    draw_path( path );

    std::vector<cv::Point> result;
    for ( auto& p : path ) {
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
    return max( abs( node2->x - node1->x ), abs( node2->y - node1->y ) );
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

        draw_open_list();

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

void A_Star::draw_path( const std::vector<Map_Node *> path ) {
    for ( auto& n : path )
        map_a_star.at<Vec3b>( n->y, n->x ) = Vec3b(0,0,255);
}

vector<Point> A_Star::calculateRoadmapPoints(Mat roadmap)
{
    // Finds all roadmap points
    vector<Point> roadmapPoints;
    for (int y = 0; y < roadmap.rows; y++)
        for (int x = 0; x < roadmap.cols; x++)
            if ( roadmap.at<Vec3b>(y,x) == Vec3b(0,0,255) )
                roadmapPoints.push_back( Point(x,y) );
    return roadmapPoints;
}

vector<Point> A_Star::calculateTestPoints(Mat roadmap, vector<Point> roadmapPoints)
{
    vector<Point> testPoints;
    for(int i = 0; i < roadmap.rows; i++)
    {
        for(int j = 0; j < roadmap.cols; j++) // i j is startpoint
        {
            if(roadmap.at<Vec3b>(i, j) != Vec3b(0,0,0)) // If black pixel inside obstacle find new start point
            {
                Point startPointOnRoadmap = findWayToRoadMap(roadmap, roadmapPoints, Point(j,i));
                if(startPointOnRoadmap != Point(NULL, NULL)) // Have not found a starting point without obstacle to roadmap
                    testPoints.push_back(Point(j,i));
            }
        }
    }
    return testPoints;
}

vector<Point> A_Star::checkInvalidTestPoints(Mat roadmap, vector<Point> roadmapPoints, vector<Point> checkpoints)
{
    vector<Point> testPoints;
    for(size_t i = 0; i < checkpoints.size(); i++)
    {
        if(roadmap.at<Vec3b>(checkpoints[i]) != Vec3b(0,0,0)) // If black pixel inside obstacle find new start point
        {
            Point startPointOnRoadmap = findWayToRoadMap(roadmap, roadmapPoints, checkpoints[i]);
            if(startPointOnRoadmap != Point(NULL, NULL)) // Have not found a starting point without obstacle to roadmap
                testPoints.push_back(checkpoints[i]);
        }
    }
    return testPoints;
}

vector<Point> A_Star::findNRemoveDiff(vector<Point> testPoint1, vector<Point> testPoint2)
{
    vector<Point> correct;
    bool cor = false;
    for(size_t i = 0; i < testPoint1.size(); i++)
    {
        for(size_t j = 0; j < testPoint2.size(); j++)
        {
            if(testPoint1[i] == testPoint2[j])
            {
                cor = true;
                break;
            }
        }
        if(cor)
            correct.push_back(testPoint1[i]);
    }
    return correct;
}

vector<double> A_Star::findAstarPathLengthsForRoadmap(Mat roadmap) // takes too long time therefore made in main on threads
{
    vector<double> pathLengths;
    double tempDist;
    vector<Point> aStarPath;
    Point startPointOnRoadmap;
    Point endPointOnRoadmap;

    // Finds all roadmap points
    vector<Point> roadmapPoints;
    for (int y = 0; y < roadmap.rows; y++)
        for (int x = 0; x < roadmap.cols; x++)
            if ( roadmap.at<Vec3b>(y,x) == Vec3b(0,0,255) )
                roadmapPoints.push_back( Point(x,y) );

    // Finds test points
    vector<Point> testPoints;
    for(int i = 0; i < roadmap.rows; i++)
    {
        for(int j = 0; j < roadmap.cols; j++) // i j is startpoint
        {
            if(roadmap.at<Vec3b>(i, j) != Vec3b(0,0,0)) // If black pixel inside obstacle find new start point
            {
                startPointOnRoadmap = findWayToRoadMap(roadmap, roadmapPoints, Point(j,i));
                if(startPointOnRoadmap != Point(NULL, NULL)) // Have not found a starting point without obstacle to roadmap
                    testPoints.push_back(Point(j,i));
            }
        }
        //cout << i << endl;
    }
    // GOES TOWARDS EACH OTHER
    Point testPointStart = testPoints[0];
    Point testPointEnd = testPoints[testPoints.size() - 1];
    Point middlePoint = testPoints[testPoints.size() / 2];
    int indexStart = 0;
    int indexEnd = testPoints.size() - 1;
    while(testPointStart != middlePoint || testPointEnd != middlePoint)
    {
        //cout << "MiddlePoint: " << testPoints.size() / 2 << " Start: " << indexStart << endl;
        testPointStart = testPoints[indexStart];
        testPointEnd = testPoints[indexEnd];
        startPointOnRoadmap = findWayToRoadMap(roadmap, roadmapPoints, testPointStart);
        endPointOnRoadmap = findWayToRoadMap(roadmap, roadmapPoints, testPointEnd);
        if(startPointOnRoadmap != endPointOnRoadmap)
        {
            tempDist = 0;
            aStarPath = get_path(roadmap, startPointOnRoadmap, endPointOnRoadmap);
            tempDist = calculateDiagonalDist(testPointStart, startPointOnRoadmap); // From start to start on roadmap
            tempDist += calculateDiagonalDist(testPointEnd, endPointOnRoadmap); // From goal to goal on roadmap
            tempDist += aStarPath.size(); // Path length of astar
            pathLengths.push_back(tempDist);
            // Go towards each other
            indexStart++;
            indexEnd--;
        }
        else if(testPointStart == middlePoint)
            indexEnd--;
        else if(testPointEnd == middlePoint)
            indexStart++;
        else
        {
            indexStart++;
            indexEnd--;
        }
    }
    /*
    for(size_t i = 0; i < testPoints.size() ; i++)
    {
        startPointOnRoadmap = findWayToRoadMap(roadmap, roadmapPoints, testPoints[i]);
        cout << i << endl;
        for(size_t j = i + 1; j < testPoints.size() -1 ; j++)
        {
            cout << j << endl;
            endPointOnRoadmap = findWayToRoadMap(roadmap, roadmapPoints, testPoints[j]);
            if(startPointOnRoadmap == endPointOnRoadmap)
                continue;
            tempDist = 0;
            aStarPath = get_path(roadmap, startPointOnRoadmap, endPointOnRoadmap);
            tempDist = calculateDiagonalDist(testPoints[i], startPointOnRoadmap); // From start to start on roadmap
            tempDist += calculateDiagonalDist(testPoints[j], endPointOnRoadmap); // From goal to goal on roadmap
            tempDist += aStarPath.size(); // Path length of astar
            pathLengths.push_back(tempDist);
        }

    }
    */


    return pathLengths;
}

vector<double> A_Star::findAstarPathLengthsForRoadmapRandom(Mat roadmap, vector<Point> roadmapPoints, vector<Point> startPoints, vector<Point> endPoints)
{
    vector<double> pathLengths;
    vector<Point> aStarPath;
    Point startPointOnRoadmap, endPointOnRoadmap;
    double tempDist = 0;
    for(size_t i = 0; i < startPoints.size(); i++)
    {
        aStarPath.clear();
        startPointOnRoadmap = findWayToRoadMap(roadmap, roadmapPoints, startPoints[i]);
        endPointOnRoadmap = findWayToRoadMap(roadmap, roadmapPoints, endPoints[i]);
        if(startPointOnRoadmap != endPointOnRoadmap)
        {
            tempDist = 0;
            aStarPath = get_path(roadmap, startPointOnRoadmap, endPointOnRoadmap);
            tempDist = calculateDiagonalDist(startPoints[i], startPointOnRoadmap); // From start to start on roadmap
            tempDist += calculateDiagonalDist(endPoints[i], endPointOnRoadmap); // From goal to goal on roadmap
            tempDist += aStarPath.size(); // Path length of astar
            pathLengths.push_back(tempDist);
        }
        else // If equal to eachother the aStarPath = 0
        {
            tempDist = 0;
            tempDist = calculateDiagonalDist(startPoints[i], startPointOnRoadmap); // From start to start on roadmap
            tempDist += calculateDiagonalDist(endPoints[i], endPointOnRoadmap); // From goal to goal on roadmap
            pathLengths.push_back(tempDist);
        }
    }
    return pathLengths;
}

vector<double> A_Star::getResults()
{
    return results;
}

// -----------------------------------------------------------------------

void A_Star::draw_open_list() {
    for ( auto& o : open_list ) {
        Map_Node *n = o;
        if ( ( n == start_node ) || ( n == goal_node ) )
            continue;
        map_a_star.at<Vec3b>(n->y, n->x) = Vec3b(210,210,210);
    }
}

// -----------------------------------------------------------------------

Mat A_Star::get_a_star() {
    return map_a_star;
}


Point A_Star::findWayToRoadMap(Mat roadmap, vector<Point> roadmapPoints, Point entryExitPoint)
{
    Point roadmapEntryExit = Point(NULL, NULL);
    bool first = true;
    for(size_t i = 0; i < roadmapPoints.size(); i++)
    {
        if(!obstacleDetectedWithLine(roadmap, roadmapPoints[i], entryExitPoint))
        {
            if(first)
            {
                roadmapEntryExit = roadmapPoints[i];
                first = false;
            }
            else
            {
                if(calculateDiagonalDist(roadmapEntryExit, entryExitPoint) >= calculateDiagonalDist(roadmapPoints[i], entryExitPoint))
                    roadmapEntryExit = roadmapPoints[i];
            }
        }
    }
    return roadmapEntryExit;
}

double A_Star::calculateDiagonalDist(Point p1, Point p2)
{
    return sqrt(pow(p1.x-p2.x,2) + pow(p1.y-p2.y,2));
}



void A_Star::calculateDistThread(Mat roadmap, vector<Point> testPoints, vector<Point> roadmapPoints, int threadNumber, int amountOfThreads) // is not used because of threads in qt
{
    int amount = testPoints.size()/amountOfThreads;
    int start = amount*(threadNumber -1);
    int stop = amount*(threadNumber);
    Point startPointOnRoadmap;
    Point endPointOnRoadmap;
    int tempDist = 0;
    vector<Point> aStarPath;
    int percentDone = 0;
    cout << "ThreadNumber: " << threadNumber << " start: " << start << " stop: " << stop << " amount: " << amount << endl;
    string file = "Results_Thread_" + to_string(threadNumber) + ".txt";
    ofstream myFile;
    myFile.open(file);
    for(int i = start; i < stop; i++)
    {
        cout << "Tread Number: " << threadNumber << " Percent done: " << (int)(((double)percentDone/(double)amount)*100) << endl;
        percentDone++;
        startPointOnRoadmap = findWayToRoadMap(roadmap, roadmapPoints, testPoints[i]);
        for(size_t j = i + 1; j < testPoints.size() ; j++)
        {

            endPointOnRoadmap = findWayToRoadMap(roadmap, roadmapPoints, testPoints[j]);

            if(startPointOnRoadmap == endPointOnRoadmap)
                continue;
            tempDist = 0;
            aStarPath = get_path(roadmap, startPointOnRoadmap, endPointOnRoadmap);
            tempDist = calculateDiagonalDist(testPoints[i], startPointOnRoadmap); // From start to start on roadmap
            tempDist += calculateDiagonalDist(testPoints[j], endPointOnRoadmap); // From goal to goal on roadmap
            tempDist += aStarPath.size(); // Path length of astar

            myFile << tempDist << endl;

        }
    }

    myFile.close();
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
    LineIterator it(roadmap, start, end, 8); // 8 for diagonal
    for (int i = 0; i < it.count; i++, it++) // next point on line
    {
        if ( roadmap.at<Vec3b>( it.pos() ) == Vec3b(0,0,0) )
            return true;
    }
    return false;
}

// -----------------------------------------------------------------------
