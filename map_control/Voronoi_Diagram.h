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
     *          1. The number of neighboring black pixels is at least 2 and not greater than 6
     *          2. The number of white-to-black transitions around P1 is equal to 1
     *          3. At least one of P2, P4 or P6 is white
     *          4. At least one of P4, P6 or P8 is white
     *              All P1 that meets the criteria is set to white
     *
     *  Step two
     *      Search through the image and check if all conditions are fulfilled
     *          1. (same as step one)
     *          2. (same as step one)
     *          3. At least one of P2, P4 or P8 is white
     *          4. At least one of P2, P6 or P8 is white
     *              All P1 that meets the criteria is set to white
     *
     *  Step one and two are reapeated until image pixels are no longer changing
     ************************************************************/
    public:
        Voronoi_Diagram();
        ~Voronoi_Diagram();

        /**
         * @brief get_voronoi_img
         * @param src
         * @param dst
         */
        void get_voronoi_img( const cv::Mat &src, cv::Mat &dst );

        /**
         * @brief get_thinning_img
         * @param src
         * @param dst
         */
        void get_thinning_img( const cv::Mat &src, cv::Mat &dst );

        /**
         * @brief get_skeletinize_img
         * @param srce
         * @param dst
         */
        void get_skeletinize_img( const cv::Mat &src, cv::Mat &dst );

    protected:
        /**
         * @brief voronoi
         *      Generate the voronoi diagram
         * @param input
         * @param output
         */
        void voronoi( const cv::Mat &input,
                      cv::Mat &output );

        /**
         * @brief opencv_thinning
         *      Generates voronoi diagram using opencv funtion thinning
         * @param input
         * @param output
         */
        void opencv_thinning( const cv::Mat &input,
                             cv::Mat &output );

        /**
         * @brief skeletinize
         *      Generates skelet of map
         * @param input
         * @param output
         */
        void skeletinize( const cv::Mat &input,
                          cv::Mat &output );

        /**
         * @brief thinning_iteration -> Perform one thinning iteration
         * @param img -> Binary image with range = 0-1
         * @param iter -> 0 = step one, 1 = step two
         */
        void thinning_iteration( cv::Mat &img, int iter );

        /**
         * @brief make_voronoi -> Function for thinning the given binary image
         * @param img -> binary image with range = 0 - 255
         */
        void make_voronoi( cv::Mat &img );

        void print_map( const cv::Mat &img, const string &s );
};

#endif // VORONI_DIAGRAM_H
