#include "DetectRooms.h"

DetectRooms::DetectRooms() {}

// ---------------------------

DetectRooms::~DetectRooms() {}

// ---------------------------------------------------------

void DetectRooms::printImage(const cv::Mat &img, const string &s)
{
    Mat resizeMap;
    resize(img, resizeMap, img.size()*10, 0, 0, INTER_NEAREST);
    imshow(s, resizeMap);
}

// --------------------------------------------------------------

std::vector<cv::Point> DetectRooms::brushfireFindCenters( const cv::Mat &src )
{
    Mat img = src.clone(), imgBrushfire;

    imgBrushfire = brushfireImage(img);    // Get brushfire grid

    // Get walls
    vector<Point> walls;
    for (int y = 0; y < imgBrushfire.rows; y++)
        for (int x = 0; x < imgBrushfire.cols; x++)
            if ( (int)imgBrushfire.at<uchar>(y,x) == 0 )
                walls.push_back( Point(x,y) );

    // Detected rooms
    Mat imgDilate, kernel = Mat::ones( Size(25,25), 1u);
    dilate( imgBrushfire, imgDilate, kernel );
    imgDilate = ( imgBrushfire >= imgDilate );

    // Remove walls
    for ( auto& point : walls)
        imgDilate.at<uchar>( point ) = 0;

    // Remove outside floor plan
    for (int y = 0; y < imgDilate.rows; y++)
    {
        imgDilate.at<uchar>( y, 0) = 0;
        imgDilate.at<uchar>( y, imgDilate.cols-1 ) = 0;
    }
    for (int x = 0; x < imgDilate.cols; x++)
    {
        imgDilate.at<uchar>( 0, x ) = 0;
        imgDilate.at<uchar>( imgDilate.rows-1, x ) = 0;
    }

    // Find contours
    vector<vector<Point>> contours;
    findContours( imgDilate, contours, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0,0) );

    // Get centers of contours
    vector<Point> centers( contours.size() );
    for (size_t i = 0; i < contours.size(); i++)
    {
        Point p;

        if ( contours[i].size() > 2 )
        {
            Moments mu = moments( contours[i] );
            p.x = (int)( mu.m10/mu.m00 );
            p.y = (int)( mu.m01/mu.m00 );
        }
        else if ( contours[i].size() == 2 )
        {
            Point start = contours[i][0], end = contours[i][1];
            p.x = (start.x + (end.x - start.x)/2);
            p.y = (start.y + (end.y - start.y)/2);
        }
        else
        {
            p.x = contours[i][0].x;
            p.y = contours[i][0].y;
        }

        centers[i] = p;
    }

    return centers;
}

// ------------------------------------------------------

cv::Mat DetectRooms::brushfireImage( const cv::Mat &src )
{
    Mat result = src.clone();

    binarizeImage(result, result);

    makeBrushfireGrid(result, result);

    return result;
}

// -----------------------------------------------------------

std::vector<cv::Point> DetectRooms::squareFindCenters( const cv::Mat &src )
{
    // Convert source image to binary image
    Mat img;
    cvtColor( src, img, CV_BGR2GRAY );
    threshold( img, img, 40, 255, THRESH_BINARY );

    std::vector<cv::Point> centers;
    for (int y = 5; y < img.rows-5; y++)
    {
        for (int x = 5; x < img.cols-5; x++)
        {
            if ( (int)img.at<uchar>(y,x) == 255 )
            {
                makeRectangle(img, Point(x,y), centers);
            }
        }
    }

    return centers;
}

// --------------------------------------------------------------

void DetectRooms::binarizeImage( const cv::Mat &src, cv::Mat &dst )
{
    cvtColor(src, dst, CV_BGR2GRAY);

    threshold(dst, dst, 127, 255, CV_THRESH_BINARY);

    dst = dst > 128;
}

// --------------------------------------------------------------

void DetectRooms::findNeighbors( std::vector<cv::Point> &v,
                                 const cv::Mat &img,
                                 const cv::Point &p )
{
    Point leftTop(p.x-1, p.y-1);
    Point top(p.x, p.y-1);
    Point rightTop(p.x+1, p.y-1);
    Point left(p.x-1, p.y);
    Point right(p.x+1, p.y);
    Point leftBottom(p.x-1, p.y+1);
    Point bottom(p.x, p.y+1);
    Point rightBottom(p.x+1, p.y+1);

    // Left Top
    if ((int)img.at<uchar>(leftTop)==255)
        v.push_back(Point(leftTop));
    // Top
    if ((int)img.at<uchar>(top)==255)
        v.push_back(Point(top));
    // Right Top
    if ((int)img.at<uchar>(rightTop)==255)
        v.push_back(Point(rightTop));
    // Left
    if ((int)img.at<uchar>(left)==255)
        v.push_back(Point(left));
    // Right
    if ((int)img.at<uchar>(right)==255)
        v.push_back(Point(right));
    // Left Bottom
    if ((int)img.at<uchar>(leftBottom)==255)
        v.push_back(Point(leftBottom));
    // Bottom
    if ((int)img.at<uchar>(bottom)==255)
        v.push_back(Point(bottom));
    // Right Bottom
    if ((int)img.at<uchar>(rightBottom)==255)
        v.push_back(Point(rightBottom));
}

// --------------------------------------------------------------

void DetectRooms::makeBrushfireGrid( const cv::Mat &img, cv::Mat &dst )
{
    vector<Point> neighbors;
    for (int y = 0; y < img.rows; y++)
        for (int x = 0; x < img.cols; x++)
            if ((int)img.at<uchar>(y,x) == 0)
                findNeighbors(neighbors, img, Point(x, y));

    dst = img.clone();
    int color = 1;
    while ( !neighbors.empty() )
    {
        vector<Point> new_neighbors;
        for (size_t i = 0; i < neighbors.size(); i++)
            if (dst.at<uchar>(neighbors[i]) == 255)
            {
                findNeighbors(new_neighbors, dst, neighbors[i]);
                dst.at<uchar>(neighbors[i]) = color;
            }

        color++;
        neighbors = new_neighbors;
    }
}

// --------------------------------------------------------------

void DetectRooms::makeRectangle( cv::Mat &img,
                                 const cv::Point &p,
                                 std::vector<cv::Point> &v)
{
    int width, height;

    // Get width
    for (int x = p.x; x < img.cols; x++)
    {
        if ( (int)img.at<uchar>(p.y, x) == 0 )
        {
            width = x;
            break;
        }
    }

    // Get height
    for (int y = p.y; y < img.rows; y++)
    {
        if ( (int)img.at<uchar>(y, p.x) == 0 )
        {
            height = y;
            break;
        }
    }

    int area = (width - p.x) * (height - p.y);
    if (area >= 100)
    {
        Point point( (p.x + width)/2, (p.y + height)/2 );

        while(img.at<uchar>(point) == 0)
        {
            width--;
            point.x = (p.x + width)/2;
        }

        v.push_back( Point( (p.x + width)/2, (p.y + height)/2 ) );
    }

    for (int y = p.y ; (y < height+2) && (y < img.rows); y++ )
        for (int x = p.x; (x < width+2) && (x < img.cols); x++)
            img.at<uchar>( y, x ) = 0;
}

// --------------------------------------------------------------

void DetectRooms::removePointsInCorners(std::vector<cv::Point> &v,
                                        const cv::Mat &img)
{
    for (size_t i = 0; i < v.size(); i++)
    {
        if ( (int)img.at<uchar>(v[i].y-1,v[i].x-1) == 0 )
            v.erase(v.begin()+i);

        if ( (int)img.at<uchar>(v[i].y,v[i].x-1) == 0 )
            v.erase(v.begin()+i);

        if ( (int)img.at<uchar>(v[i].y+1,v[i].x-1) == 0 )
            v.erase(v.begin()+i);

        if ( (int)img.at<uchar>(v[i].y-1,v[i].x) == 0 )
            v.erase(v.begin()+i);

        if ( (int)img.at<uchar>(v[i].y+1,v[i].x) == 0 )
            v.erase(v.begin()+i);

        if ( (int)img.at<uchar>(v[i].y-1,v[i].x+1) == 0 )
            v.erase(v.begin()+i);

        if ( (int)img.at<uchar>(v[i].y,v[i].x+1) == 0)
            v.erase(v.begin()+i);

        if ( (int)img.at<uchar>(v[i].y+1,v[i].x+1) == 0)
            v.erase(v.begin()+i);
    }
}

// --------------------------------------------------------------
