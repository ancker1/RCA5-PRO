#include "A_Star.h"

// -----------------------------------------------------------------------

A_Star::A_Star() {}

// -----------------------------------------------------------------------

A_Star::~A_Star() {}

// -----------------------------------------------------------------------

std::vector<cv::Point> A_Star::get_path(const cv::Mat &road_map,
                                        const cv::Point &start,
                                        const cv::Point &goal)
{
    // Map for displaying a_star
    map_a_star = big_map.clone();

    open_list.clear();

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

// -----------------------------------------------------------------------
