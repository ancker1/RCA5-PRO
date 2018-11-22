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
void Zhang_suen::delete_pixels( cv::Mat& img,
                                std::vector<Point> &v) {

    for ( auto& p : v ) {
        img.at<uchar>( p ) = 0;
    }
    v.clear();
}
// -----------------------------------------------------------------------
void Zhang_suen::remove_staircases( cv::Mat &img ) {
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
        uchar *iter = img.ptr<uchar>(y);
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
        boundary_smooth( image );

    if ( need_acute_angle_emphasis == true )
        acute_angle_emphasis( image );

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
void Zhang_suen::zhang_suen_thin(Mat &img) {
    while (true) {
        std::vector<Point> points;
        for (int y = 1; y < img.rows; y++) {
            for (int x = 1; x < img.cols; x++) {
                if ( (int)img.at<uchar>( y, x) != 1 )
                    continue;
                Point p(x,y);
                int k = num_one_pixel_neighbours( img, p );
                if ( ( k >= 2 && k <= 6 )  && ( connectivity( img, p ) == 1 ) ) {
                    int p1 = (int)img.at<uchar>( y, x+1 ) *
                             (int)img.at<uchar>( y-1, x ) *
                             (int)img.at<uchar>( y, x-1 );

                    int p2 = (int)img.at<uchar>( y-1, x ) *
                             (int)img.at<uchar>( y+1, x ) *
                             (int)img.at<uchar>( y, x-1 );

                    if ( p1==0 && p2==0 )
                        points.push_back( p );
                }
            }
        }

        if ( points.size() == 0 )
            break;

        delete_pixels( img, points );

        for (int y = 1; y < img.rows; y++) {
            for (int x = 1; x < img.cols; x++) {
                if ( (int)img.at<uchar>(y,x) != 1 )
                    continue;
                Point p(x,y);
                int k = num_one_pixel_neighbours(img, p);
                if ( ( k >= 2 && k <= 6 ) && ( connectivity(img, p) == 1 ) ) {
                    int p1 = (int)img.at<uchar>(y-1, x) *
                             (int)img.at<uchar>(y, x+1) *
                             (int)img.at<uchar>(y+1, x);

                    int p2 = (int)img.at<uchar>(y,x+1) *
                             (int)img.at<uchar>(y+1,x) *
                             (int)img.at<uchar>(y,x-1);
                    if ( p1 == 0 && p2 == 0 ) {
                        points.push_back( p );
                    }
                }
            }
        }

        if ( points.size() == 0 )
            break;

        delete_pixels( img, points );
    }
}
// -----------------------------------------------------------------------
void Zhang_suen::boundary_smooth( cv::Mat &img ) {
    std::vector<Point> points;
    for (int y = 0; y < img.rows; y++ ) {
        for (int x = 0; x < img.cols; x++) {
            Point p(x,y);
            if ( (int)img.at<uchar>(y,x) == 0 ) {
                if ( num_zero_pixel_neighbours(img, p) <= 2 &&
                     yokoi_connectivity(img, p) < 2) {
                    points.push_back( p );
                }
            }
        }
    }

    for ( auto& p : points) {
        img.at<uchar>(p.y, p.x) = 1;
    }
}
// -----------------------------------------------------------------------
void Zhang_suen::acute_angle_emphasis(Mat &img) {
    for (int k = 5; k >= 1; k -= 2) {
        std::vector<Point> points;
        for (int y = 2; y < img.rows-2; y++) {
            for (int x = 2; x < img.cols-2; x++) {
                if ( (int)img.at<uchar>(y,x) == 0 ) {
                    Point p(x,y);
                    if ( match_templates(img, p, k) ) {
                        img.at<uchar>(y,x) = 2;
                        points.push_back( p );
                    }
                }
            }
        }

        if ( points.empty() )
            break;

        for ( auto& p : points ) {
            img.at<uchar>(p.y, p.x) = 1;
        }

        points.clear();
    }
}
// -----------------------------------------------------------------------
bool Zhang_suen::match( const std::vector<uchar> &points,
                        const std::vector<uchar> &values ) {
    bool m = true;
    for (std::vector<uchar>::size_type i = 0; i < points.size(); i++) {
        if ( values[i] == 2 )
            continue;

        switch ( values[i] ) {
            case 0:
                if ( points[i] != 0 )
                    m = false;
                break;
            case 1:
                if ( points[i] != 1 )
                    m = false;
                break;
        }

        if ( m == false )
            break;
    }
    return m;
}
// -----------------------------------------------------------------------
bool Zhang_suen::match_templates(const cv::Mat &img,
                                 const Point &point,
                                 int k) {
    int r = point.y, c = point.x;
    std::vector<uchar> points {
        img.at<uchar>(r - 2, c - 2), img.at<uchar>(r - 2, c - 1),
        img.at<uchar>(r - 2, c), img.at<uchar>(r - 2, c + 1),
        img.at<uchar>(r - 2, c + 2), img.at<uchar>(r - 1, c - 2),
        img.at<uchar>(r - 1, c - 1), img.at<uchar>(r - 1, c),
        img.at<uchar>(r - 1, c + 1), img.at<uchar>(r - 1, c + 2),
        img.at<uchar>(r, c - 2), img.at<uchar>(r, c - 1),
        img.at<uchar>(r, c), img.at<uchar>(r, c + 1),
        img.at<uchar>(r, c + 2), img.at<uchar>(r + 1, c - 2),
        img.at<uchar>(r + 1, c - 1), img.at<uchar>(r + 1, c),
        img.at<uchar>(r + 1, c + 1), img.at<uchar>(r + 1, c + 2),
        img.at<uchar>(r + 2, c - 2), img.at<uchar>(r + 2, c - 1),
        img.at<uchar>(r + 2, c), img.at<uchar>(r + 2, c + 1),
        img.at<uchar>(r + 2, c + 2)
    };

    std::vector<uchar> values = {
        0, 0, 1, 0, 0,
        0, 0, 1, 0, 0,
        0, 0, 0, 0, 0,
        0, 0, 0, 0, 0,
        2, 0, 0, 0, 2
    };

    if ( match( points, values ) )
        return true;

    if ( k >= 2 ) {
        values[1] = 1;
        if ( match( points, values ) )
            return true;
    }

    if ( k >= 3 ) {
        values[1] = 0;
        values[3] = 1;
        if ( match( points, values ) )
            return true;
    }

    if ( k >= 4 ) {
        values[3] = 0;
        values[1] = values[6] = 1;
        if ( match( points, values ) )
            return true;
    }

    if ( k >= 5 ) {
        values[1] = values[6] = 0;
        values[3] = values[8] = 1;
        if ( match( points, values ) )
            return true;
    }

    values = {
        2, 0, 0, 0, 2,
        0, 0, 0, 0, 0,
        0, 0, 0, 0, 0,
        0, 0, 1, 0, 0,
        0, 0, 1, 0, 0
    };

    if ( match( points, values ) )
        return true;

    if ( k >= 2 ) {
        values[21] = 1;
        if ( match( points, values ) )
            return true;
    }

    if ( k >= 3 ) {
        values[21] = 0;
        values[23] = 1;
        if ( match( points, values ) )
            return true;
    }

    if ( k >= 4 ) {
        values[23] = 0;
        values[16] = values[21] = 1;
        if ( match( points, values ) )
            return true;
    }

    if ( k >= 5 ) {
        values[16] = values[21] = 0;
        values[18] = values[23] = 1;
        if ( match( points, values ) )
            return true;
    }

    return false;
}
// -----------------------------------------------------------------------


















