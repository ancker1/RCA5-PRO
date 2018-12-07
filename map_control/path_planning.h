#ifndef PATH_PLANNING_H
#define PATH_PLANNING_H

#include <iostream>
#include <vector>
#include <math.h>
#include <thread>
#include <chrono>

// Opencv
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/ximgproc.hpp"

#include "Map.h"

using namespace std;
using namespace cv;

class Path_planning {
public:
    Path_planning();
    ~Path_planning();

    /**
     * @brief way_around_obstacle -> Determines is there is a obstacle between
     * start and goal pixel and which way to take around the obstacle
     * @param start -> Start pixel
     * @param goal -> Target pixel
     * @param src -> map
     * @return -> Which way to take arounde obstacle (Straight, left or right)
     */
    int way_around_obstacle( const cv::Point start,
                             const cv::Point goal,
                             const cv::Mat &src );

    /**
     * @brief make_visibility_map -> Makes a visibility map of the roadmap
     * @param img -> big_map or small_map
     * @return -> Visibility map (Green pixels = visibility)
     */
    Mat make_visibility_map( const cv::Mat &img,
                             const std::vector<cv::Point> &road_map_points);

private:
    void print_map( const cv::Mat &img, const std::string &s );

    /**
     * @brief go_left
     * @param p
     * @param img
     */
    void go_left( cv::Point &p, const cv::Mat &img);

    /**
     * @brief go_right
     * @param p
     * @param img
     */
    void go_right( cv::Point &p, const cv::Mat &img);

    /**
     * @brief get_points
     * @param it
     * @return
     */
    std::vector<cv::Point> get_points( cv::LineIterator &it );

    /**
     * @brief get_p_before_obstacle
     * @param v
     * @param img
     * @return
     */
    cv::Point get_p_before_obstacle( const std::vector<cv::Point> &v,
                                     const cv::Mat &img );

    /**
     * @brief obstacle_detected
     * @param start
     * @param goal
     * @param img
     * @return
     */
    bool obstacle_detected( const cv::Point &start,
                            const cv::Point &goal,
                            const cv::Mat &img );

    /**
     * @brief obs_detect_color
     * @param start
     * @param goal
     * @param img
     */
    void obs_detect_color( const cv::Point start,
                           const cv::Point goal,
                           cv::Mat &img );

    /**
     * @brief convertToImageCoordinates:
     *  Converts gazebo coordinates to image coordinates
     * @param start
     * @param goal
     */
    void convertToImageCoordinates( cv::Point &start, cv::Point &goal );
};

#endif // PATH_PLANNING_H
