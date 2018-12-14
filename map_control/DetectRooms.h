#ifndef DETECTROOMS_H
#define DETECTROOMS_H

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

class DetectRooms
{
    public:

        DetectRooms();

        /**
         * @brief   : Finds centers of rooms with the brushfire grid
         * @param   : Source image
         * @return  : Vector with center points
         */
        std::vector<Point> brushfireFindCenters( const cv::Mat &src );

        /**
         * @brief   : Finds centers of rooms by detecting squares in the image
         * @param   : Source image
         * @return  : vector with center points
         */
        std::vector<cv::Point> squareFindCenters( const cv::Mat &src );

        ~DetectRooms();

    private:

        /**
         * @brief   : Makes a brushfire(distance map) of the source image
         * @param   : Source image
         * @return  : brushfire gird
         */
        cv::Mat brushfireImage( const cv::Mat &src );

        /**
         * @brief   : Makes a binary image of src on dst
         * @param   : Source image
         * @param   : Destination image
         */
        void binarizeImage( const cv::Mat &src, cv::Mat &dst );

        /**
         * @brief   : Finds white neighbors of input pixel(x,y)
         * @param   : Vector with white neighbors
         * @param   : Binary image of map
         * @param   : Input pixel x coordinate
         * @param   : Input pixel y coordinate
         */
        void findNeighbors( std::vector<cv::Point> &v,
                             const cv::Mat &img,
                             const cv::Point &p );

        /**
         * @brief   : Make a brushfire grid
         * @param   : Source image
         * @param   : Destination image
         */
        void makeBrushfireGrid( const cv::Mat &img, cv::Mat &dst );

        /**
         * @brief   : Remove points in corners of image
         * @param   : Vector with points
         * @param   : Binary image of map
         */
        void removePointsInCorners( std::vector<Point> &v,
                                       const cv::Mat &img);

        /**
         * @brief   : Finds the center of a rectangle
         * @param   : Binary image of map
         * @param   : Start point in map
         * @param   : Vector where center is stored
         */
        void makeRectangle( cv::Mat &img,
                            const cv::Point &p,
                            std::vector<cv::Point> &v );

        void printImage( const cv::Mat &img, const std::string &s );
};

#endif // DETECTROOMS_H
