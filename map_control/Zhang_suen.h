#ifndef ZHANG_SUEN_H
#define ZHANG_SUEN_H

#include <iostream>
#include <vector>
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/ximgproc.hpp"

using namespace std;
using namespace cv;

class Zhang_suen {
    public:
        // Constructors
        Zhang_suen();
        Zhang_suen( const Mat &img );
        ~Zhang_suen();

        Mat get_img();

    protected:
        Mat src;

        int num_one_pixel_neighbours( const cv::Mat &img,
                                      const Point &p );

        int num_zero_pixel_neighbours( const cv::Mat &img,
                                       const Point &p );

        int connectivity( const cv::Mat &img,
                          const Point &p );

        int yokoi_connectivity( const cv::Mat &img,
                                const Point &p );

        void delete_pixels( cv::Mat &img,
                            std::vector<Point> &v);

        void remove_staircases( cv::Mat &img );

        void zhang_suen_thin(cv::Mat &img);

        void thin( cv::Mat &img,
                   bool need_boundary_smoothing = false,
                   bool need_acute_angle_emphasis = false,
                   bool destair = false );

        void boundary_smooth( cv::Mat &img );

        void acute_angle_emphasis( cv::Mat &img );

        bool match( const std::vector<uchar> &points,
                    const std::vector<uchar> &values);

        bool match_templates( const cv::Mat &img,
                              const Point &point,
                              int k );
};

#endif // ZHANG_SUEN_H
