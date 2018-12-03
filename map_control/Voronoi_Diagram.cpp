#include "Voronoi_Diagram.h"

// -------------------------------------------------------------------------

Voronoi_Diagram::Voronoi_Diagram() {}

// -------------------------------------------------------------------------

Voronoi_Diagram::~Voronoi_Diagram() {}

// -------------------------------------------------------------------------

void Voronoi_Diagram::get_voronoi_img( const cv::Mat &src, cv::Mat &dst ) { voronoi( src, dst ); }

// -------------------------------------------------------------------------

void Voronoi_Diagram::get_thinning_img( const cv::Mat &src, cv::Mat &dst ) { opencv_thinning( src, dst ); }

// -------------------------------------------------------------------------

void Voronoi_Diagram::get_skeletinize_img( const cv::Mat &src, cv::Mat &dst) { skeletinize( src, dst ); }

// -------------------------------------------------------------------------

void Voronoi_Diagram::voronoi( const cv::Mat &input,
                               cv::Mat &output_img )
{
    Mat gray;
    cvtColor( input, gray, CV_BGR2GRAY );
    threshold( gray, gray, 10, 1, CV_THRESH_BINARY );
    make_voronoi( gray );
    for (int y = 0; y < gray.rows; y++) {
        gray.at<uchar>(y, 0) = 0;
        gray.at<uchar>(y, gray.cols-1) = 0;
    }
    for (int x = 0; x < gray.cols; x++) {
        gray.at<uchar>(0, x) = 0;
        gray.at<uchar>(gray.rows-1, x) = 0;
    }
    output_img = gray.clone();
}

// -------------------------------------------------------------------------

void Voronoi_Diagram::opencv_thinning(const cv::Mat &input,
                               cv::Mat &output_img )
{
    Mat gray;
    cv::cvtColor( input, gray, CV_BGR2GRAY );
    cv::threshold( gray, gray, 127, 255, CV_THRESH_BINARY );
    for (int y = 0; y < gray.rows; y++) {
        gray.at<uchar>(y, 0) = 0;
        gray.at<uchar>(y, gray.cols-1) = 0;
    }
    for (int x = 0; x < gray.cols; x++) {
        gray.at<uchar>(0, x) = 0;
        gray.at<uchar>(gray.rows-1, x) = 0;
    }
    cv::ximgproc::thinning( gray, output_img, cv::ximgproc::THINNING_ZHANGSUEN );
}

// -------------------------------------------------------------------------

void Voronoi_Diagram::skeletinize( const cv::Mat &input,
                                   cv::Mat &output_img )
{
    Mat gray;
    cv::cvtColor( input, gray, CV_BGR2GRAY );
    cv::threshold( gray, gray, 127, 255, CV_THRESH_BINARY );
    for (int y = 0; y < gray.rows; y++) {
        gray.at<uchar>(y, 0) = 0;
        gray.at<uchar>(y, gray.cols-1) = 0;
    }
    for (int x = 0; x < gray.cols; x++) {
        gray.at<uchar>(0, x) = 0;
        gray.at<uchar>(gray.rows-1, x) = 0;
    }
    Mat skel( gray.size(), CV_8UC1, Scalar(0) ), temp, eroded, element;
    element = cv::getStructuringElement( MORPH_CROSS, Size(3,3) );
    bool done = false;
    do {
        erode( gray, eroded, element );
        dilate( eroded, temp, element );
        subtract( gray, temp, temp );
        bitwise_or( skel, temp, skel );
        eroded.copyTo( gray );
        done = ( cv::countNonZero( gray ) == 0 );
    }
    while ( done == false );
    output_img = skel.clone();
}

// -------------------------------------------------------------------------

void Voronoi_Diagram::thinning_iteration( cv::Mat &img, int iter)
{
    cv::Mat marker = cv::Mat::zeros( img.size(), CV_8UC1 );
    for (int y = 1; y < img.rows-1; y++)
        for (int x = 1; x < img.cols-1; x++)
        {
            int p9 = (int)img.at<uchar>( y-1, x-1 );
            int p2 = (int)img.at<uchar>( y-1, x );
            int p3 = (int)img.at<uchar>( y-1, x+1 );
            int p4 = (int)img.at<uchar>( y, x+1 );
            int p5 = (int)img.at<uchar>( y+1, x+1 );
            int p6 = (int)img.at<uchar>( y+1, x );
            int p7 = (int)img.at<uchar>( y+1, x-1 );
            int p8 = (int)img.at<uchar>( y, x-1 );

            // The number of neighboring white pixels is at least 2 and not greater than 6
            int sum  = p2 + p3 + p4 + p5 + p6 + p7 + p8 + p9;
            if ( sum < 2 || sum > 6 )
                continue;

            // The number of white-to-black transistions around P1 is equal to 1
            int transistions  = (p2 == 0 && p3 == 1) + (p3 == 0 && p4 == 1) +
                     (p4 == 0 && p5 == 1) + (p5 == 0 && p6 == 1) +
                     (p6 == 0 && p7 == 1) + (p7 == 0 && p8 == 1) +
                     (p8 == 0 && p9 == 1) + (p9 == 0 && p2 == 1);
            if ( transistions != 1 )
                continue;

            // if iter = 0 => step one
            //      m1 => at least one of P2, P4 or P6 is black
            //      m2 => at least one of P4, P6 or P8 is black
            // if iter = 1 => step two
            //      m1 => at least one of P2, P4 or P8 is black
            //      m2 => at least one of P2, P6 or P8 is black
            int m1 = ( iter == 0 ) ? (p2 * p4 * p6) : (p2 * p4 * p8);
            int m2 = ( iter == 0 ) ? (p4 * p6 * p8) : (p2 * p6 * p8);
            if (m1 == 0 && m2 == 0)
                marker.at<uchar>(y,x) = 1;
        }

    img &= ~marker;
}

// -------------------------------------------------------------------

void Voronoi_Diagram::make_voronoi( cv::Mat &img )
{
    cv::Mat prev = cv::Mat::zeros( img.size(), CV_8UC1 ), diff;
    do
    {
        thinning_iteration( img, 0 );
        thinning_iteration( img, 1 );
        cv::absdiff( img, prev, diff );
        img.copyTo( prev );
    }
    while ( cv::countNonZero( diff ) > 0 );

    img *= 255;
}

// -------------------------------------------------------------------

void Voronoi_Diagram::print_map( const cv::Mat &img,
                                 const string &s )
{
    Mat resizeMap;
    convertScaleAbs(img, resizeMap, 255);
    resize( resizeMap, resizeMap, img.size()*10, 0, 0, INTER_NEAREST);
    imshow(s, resizeMap);
}

// -------------------------------------------------------------------

void Voronoi_Diagram::imageSegmentation(const Mat &src, Mat &dst)
{
    Mat img = src.clone();

    // Create a binary image from source image
    Mat imgBinary;
    cvtColor( img, imgBinary, COLOR_BGR2GRAY );
    threshold( imgBinary, imgBinary, 127, 255, THRESH_BINARY );

    // Perform the distance transform algorithm
    Mat imgDist;
    distanceTransform( imgBinary, imgDist, DIST_L2, 3 );

    // Normalize the distance image for range = { 0.0, 1.0 }
    normalize( imgDist, imgDist, 0, 1.0, NORM_MINMAX );

    // Threshold to obtain the peaks
    threshold( imgDist, imgDist, 0.4, 1.0, THRESH_BINARY );

    // Dilate the dist image
    Mat kernel1 = Mat::ones( 1, 2, CV_8U );
    dilate( imgDist, imgDist, kernel1 );

    // Create the CV_8U version of the distance image
    Mat imgDist8u;
    imgDist.convertTo( imgDist8u, CV_8U );

    // Find total markers
    vector<vector<Point>> contours;
    findContours( imgDist8u, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE );

    // Create the marker image for the watershed algorithm
    Mat markers = Mat::zeros( imgDist.size(), CV_32S );

    // Draw the markers
    for (size_t i = 0; i < contours.size(); i++)
        drawContours( markers, contours, (int)i, Scalar( ((int)i)+1, -1 ) );

    // Perform the watershed algorithm
    watershed( img, markers );

    // Fill labeled objects with random colors
    dst = src.clone();
    for (int y = 0; y < markers.rows; y++)
        for (int x = 0; x < markers.cols; x++)
            if ( dst.at<Vec3b>(y,x) != Vec3b(0,0,0) )
            {
                if ( markers.at<int>(y,x) != -1 )
                    dst.at<Vec3b>(y,x) = Vec3b(0,0,255);
                else
                    dst.at<Vec3b>(y,x) = Vec3b(0,0,0);
            }

    print_map( dst, "Final" );
}

// -------------------------------------------------------------------
