#include "Zhang_suen.h"
// -----------------------------------------------------------------------
Zhang_suen::Zhang_suen() {}
// -----------------------------------------------------------------------
Zhang_suen::~Zhang_suen() {}
// -----------------------------------------------------------------------
Zhang_suen::Zhang_suen( const Mat &img ) {
    src = img.clone();
}
// -----------------------------------------------------------------------
int Zhang_suen::num_one_pixel_neighbours( const cv::Mat &img,
                                          const Point &p) {
    int result = 0, r = p.y, c = p.x;

    for (int y = r-1; y < r+1; y++)
        for (int x = c-1; x < c+1; x++)
            if ( y!=r || x!=c )
                if ( (int)img.at<uchar>( y, x) >= 1 )
                    result++;

    return result;
}
// -----------------------------------------------------------------------
int Zhang_suen::num_zero_pixel_neighbours( const cv::Mat &img,
                                           const Point &p ) {
    int result = 0, r = p.y, c = p.x;

    for (int y = r-1; y < r+1; y++)
        for (int x = c-1; x < c+1; x++)
            if ( y!=r || x!=c )
                if ( (int)img.at<uchar>( y, x ) == 0 )
                    result++;

    return result;
}
// -----------------------------------------------------------------------
int Zhang_suen::connectivity( const cv::Mat &img,
                              const Point &p ) {
    int result = 0, r = p.x, c = p.y;

    if ( (int)img.at<uchar>(r, c+1) >= 1 &&
         (int)img.at<uchar>(r-1, c+1) == 0)
        result++;

    if ( (int)img.at<uchar>(r-1, c+1) >= 1 &&
         (int)img.at<uchar>(r-1, c) == 0)
        result++;

    if ( (int)img.at<uchar>(r-1, c) >= 1 &&
         (int)img.at<uchar>(r-1, c-1) == 0)
        result++;

    if ( (int)img.at<uchar>(r-1, c-1) >= 1 &&
         (int)img.at<uchar>(r, c-1) == 0)
        result++;

    if ( (int)img.at<uchar>(r, c-1) >= 1 &&
         (int)img.at<uchar>(r+1, c-1) == 0)
        result++;

    if ( (int)img.at<uchar>(r+1, c-1) >= 1 &&
         (int)img.at<uchar>(r+1, c) == 0)
        result++;

    if ( (int)img.at<uchar>(r+1, c) >= 1 &&
         (int)img.at<uchar>(r+1, c+1) == 0)
        result++;

    if ( (int)img.at<uchar>(r+1, c+1) >= 1 &&
         (int)img.at<uchar>(r, c+1) == 0)
        result++;

    return result;
}
// -----------------------------------------------------------------------
int Zhang_suen::yokoi_connectivity( const cv::Mat &img,
                                    const Point &p ) {
    int result = 0, r = p.y, c = p.x;

    std::vector<int> N = {
        (int)img.at<uchar>( r   , c+1 ) != 0,
        (int)img.at<uchar>( r-1 , c+1 ) != 0,
        (int)img.at<uchar>( r-1 , c )   != 0,
        (int)img.at<uchar>( r-1 , c-1 ) != 0,
        (int)img.at<uchar>( r   , c-1 ) != 0,
        (int)img.at<uchar>( r+1 , c-1 ) != 0,
        (int)img.at<uchar>( r+1 , c )   != 0,
        (int)img.at<uchar>( r+1 , c+1 ) != 0
    };

    for (std::vector<int>::size_type i = 1; i < N.size(); i += 2) {
        int i1 = ( i+1 ) % 8, i2 = (i+2) % 8;
        result += N[i] - N[i] * N[i1] * N[i2];
    }

    return result;
}
// -----------------------------------------------------------------------
void Zhang_suen::delete_pixels( const cv::Mat &img,
                                const std::vector<Point> &v) {
    for ( auto& p : v ) {
        src.at<uchar>( p.y, p.x ) = 0;
    }
}
// -----------------------------------------------------------------------
void Zhang_suen::remove_staircases( cv::MAat &img ) {
    std::vector<Point> points;

    for (int i = 0; i < 2; i++) {
        for (int y = 1; y < img.rows; y++) {
            for (int x = 1; x < img.cols; x++) {

                int c = (int)img.at<uchar>(y,x);

                if (!c)
                    continue;

                int e = (int)img.at<uchar>( y, x+1);
                int ne = (int)img.at<uchar>( y-1, x+1);
                int n = (int)img.at<uchar>( y-1, x);
                int nw = (int)img.at<uchar>( y-1, x-1);
                int w = (int)img.at<uchar>( y, x-1);
                int sw = (int)img.at<uchar>( y+1, x-1);
                int s = (int)img.at<uchar>( y+1, x);
                int se = (int)img.at<uchar>( y+1, x+1);

                if ( i == 0 ) {
                    // North biased staircase removal
                    if ( !(c && !(n &&
                         ((e && !ne && !sw && (!w || !s)) ||
                         (w && !nw && !se && (!e || !s)))))) {
                        points.push_back( Point(x,y) );
                    }
                }
                else {
                    // South bias staircase removal
                    if ( !(c && !(s &&
                         ((e && !se && !nw && (!w || !n)) ||
                          (w && !sw && !ne && (!e || !n)))))) {
                        points.push_back( Point(x,y) );
                    }
                }
            }
        }

        delete_pixels( img, points );
    }
}
// -----------------------------------------------------------------------
void Zhang_suen::thin( cv::Mat &img,
                       bool need_boundary_smoothing = false,
                       bool need_acute_angle_emphasis = false,
                       bool destair = false ) {
    for (int y = 0; y < img.rows; y++) {
        uchar *itr = img.ptr<uchar>(y);
        for (int x = 0; x < img.cols; x++) {
            *iter = (uchar)( *iter != 0 );
        }
    }

    cv::Mat image = cv::Mat::ones( img.rows+2, img.cols+2, CV_8U );

    for (int y = 0; y < img.rows; y++) {
        uchar *src_iter = img.ptr<uchar>( y );
        uchar *dst_iter = image.ptr<uchar>( y+1 );
        ++dst_iter;
        for (int x = 0; x < img.cols; x++) {
            *dst_iter++ = *src_iter++;
        }
    }

    if ( need_boundary_smoothing == true )
        boundary_smooth();

    if ( need_acute_angle_emphasis == true )
        acute_angle_emphasis();

    for (int y = 0; y < image.rows; y++) {
        uchar *iter = image.ptr<uchar>( y );
        for (int x = 0; x < image.cols; x++) {
            if ( *iter > 0 )
                *iter = 0;
            else
                *iter = 1;
        }
    }

    zhang_suen_thin(image);

    if ( destair == true )
        remove_staircases(image);

    for (int y = 0; y < img.rows; y++) {
        uchar *dst_iter = img.ptr<uchar>( y );
        uchar *src_iter = image.ptr<uchar>( y+1 );
        ++src_iter;
        for (int x = 0; x < img.cols; x++) {
            *dst_iter = ( *src_iter++ > 0 ) ? 0 : 255;
        }
    }
}
// -----------------------------------------------------------------------
