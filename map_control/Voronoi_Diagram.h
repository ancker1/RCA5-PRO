#ifndef VORONI_DIAGRAM_H
#define VORONI_DIAGRAM_H

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

class Voronoi_Diagram {
    public:
        Voronoi_Diagram();
        ~Voronoi_Diagram();

        void get_voronoi_img( const cv::Mat &src, cv::Mat &dst );
        void get_thinning_img( const cv::Mat &src, cv::Mat &dst );
        void get_skeletinize_img( const cv::Mat &src, cv::Mat &dst );

    protected:

        void voronoi( const cv::Mat &input,
                      cv::Mat &output );

        void opencv_thinning( const cv::Mat &input,
                             cv::Mat &output );

        void skeletinize( const cv::Mat &input,
                          cv::Mat &output );

        void thin_subiteration_1( const cv::Mat &input,
                                  cv::Mat &output);

        void thin_subiteration_2( const cv::Mat &input,
                                  cv::Mat &output );

        void make_voronoi( cv::Mat &input,
                           cv::Mat &output );
};

#endif // VORONI_DIAGRAM_H
