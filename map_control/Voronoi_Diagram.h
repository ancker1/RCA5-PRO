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
        /*************************************************************
         *                      ZHANG-SUEN Algorithm:
         *  Grid
         *  ----------------
         *  | P9 | P2 | P3 |
         *  ----------------
         *  | P8 | P1 | P4 |
         *  ----------------
         *  | P7 | P6 | P5 |
         *  ----------------
         *
         *  Step one
         *      Search through the image and check if all conditions are fulfilled
         *          1. P1 is white
         *          2. The number of neighboring black pixels is at least 2 and not greater than 6
         *          3. The number of white-to-black transitions around P1 is equal to 1
         *          4. At least one of P2, P4 or P6 is white
         *          5. At least one of P4, P6 or P8 is white
         *              All P1 that meets the criteria is set to white
         *
         *  Step two
         *      Search through the image and check if all conditions are fulfilled
         *          1. (same as step one)
         *          2. (same as step one)
         *          3. (same as step one)
         *          4. At least one of P2, P4 or P8 is white
         *          5. At least one of P2, P6 or P8 is white
         *              All P1 that meets the criteria is set to white
         *
         *  Step one and two are reapeated until image pixels are no longer changing
         ************************************************************/
        void voronoi( const cv::Mat &input,
                      cv::Mat &output );

        void opencv_thinning( const cv::Mat &input,
                             cv::Mat &output );

        void skeletinize( const cv::Mat &input,
                          cv::Mat &output );

        // Step one
        void thin_subiteration_1( const cv::Mat &input,
                                  cv::Mat &output);

        // Step two
        void thin_subiteration_2( const cv::Mat &input,
                                  cv::Mat &output );

        // cycling through until convergence has been reached
        void make_voronoi( cv::Mat &input,
                           cv::Mat &output );
};

#endif // VORONI_DIAGRAM_H
