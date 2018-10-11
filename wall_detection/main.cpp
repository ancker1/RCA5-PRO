#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/ximgproc.hpp"

#include <iostream>
#include <algorithm>
using namespace cv;
using namespace std;
using namespace cv::ximgproc;

int main(int argc, char** argv)
{
    // Declare the output variables
    Mat dst, cdst, cdstP;
    const char* default_file = "../wall_detection/floor_plan.png";
//    const char* default_file = "../wall_detection/big_floor_plan.png";
    const char* filename = argc >=2 ? argv[1] : default_file;
    // Loads an image
    Mat src = imread( filename, IMREAD_GRAYSCALE );
    // Check if image is loaded fine
    if(src.empty()){
        printf(" Error opening image\n");
        printf(" Program Arguments: [image_name -- default %s] \n", default_file);
        return -1;
    }
    // Edge detection
    Canny(src, dst, 50, 200, 3);
    // Copy edges to the images that will display the results in BGR
    cvtColor(dst, cdst, COLOR_GRAY2BGR);
    cdstP = src.clone();

    // Standard Hough Line Transform
    vector<Vec2f> lines; // will hold the results of the detection
    HoughLines(dst, lines, 1, CV_PI/180, 60, 0, 0); // runs the actual detection
    // Draw the lines
    for( size_t i = 0; i < lines.size(); i++ )
    {
        float rho = lines[i][0], theta = lines[i][1];
        Point pt1, pt2;
        double a = cos(theta), b = sin(theta);
        double x0 = a*rho, y0 = b*rho;
        pt1.x = cvRound(x0 + 1000*(-b));
        pt1.y = cvRound(y0 + 1000*(a));
        pt2.x = cvRound(x0 - 1000*(-b));
        pt2.y = cvRound(y0 - 1000*(a));
        line( cdst, pt1, pt2, Scalar(0,0,255), 3, CV_AA);
    }

    // Probabilistic Line Transform
    vector<Vec4i> linesP; // will hold the results of the detection

    /*  HoughLinesP
     *      dst = output
     *      lines = vector for storing the parameters of the detected lines
     *      rho = resolution of r (in pixels)
     *      theta = resolution of theta in radians
     *      threshold = the minimum number of intersections to detect a line
     *      minLinLength = the minimum number of points that can form a line
     *      maxLinGap = the maximum gap between two points */
    bitwise_not(src,src);
    HoughLinesP(src, linesP, 1, CV_PI/180, 0, 0, 2 ); // runs the actual detection

    // Draw the lines
    Mat test1(Size(src.cols,src.rows),CV_64FC1);

    for( size_t i = 0; i < linesP.size(); i++ )
    {
        Vec4i l = linesP[i];

        line( test1, Point(l[0], l[1]), Point(l[2], l[3]), 255, 1, LINE_AA);
    }
    // Create LSD detector
    Ptr<LineSegmentDetector> lsd = createLineSegmentDetector();
    vector<Vec4f> lines_lsd;

    // Create FLD detector
    // Param               Default value   Description
    // length_threshold    10            - Segments shorter than this will be discarded
    // distance_threshold  1.41421356    - A point placed from a hypothesis line
    //                                     segment farther than this will be
    //                                     regarded as an outlier
    // canny_th1           50            - First threshold for
    //                                     hysteresis procedure in Canny()
    // canny_th2           50            - Second threshold for
    //                                     hysteresis procedure in Canny()
    // canny_aperture_size 3             - Aperturesize for the sobel
    //                                     operator in Canny()
    // do_merge            false         - If true, incremental merging of segments
    //                                     will be perfomred
    int length_threshold = 10;
    float distance_threshold = 1.41421356f;
    double canny_th1 = 50.0;
    double canny_th2 = 50.0;
    int canny_aperture_size = 3;
    bool do_merge = false;
    Ptr<FastLineDetector> fld = createFastLineDetector(length_threshold,
            distance_threshold, canny_th1, canny_th2, canny_aperture_size,
            do_merge);
    vector<Vec4f> lines_fld;

    // Because of some CPU's power strategy, it seems that the first running of
    // an algorithm takes much longer. So here we run both of the algorithmes 10
    // times to see each algorithm's processing time with sufficiently warmed-up
    // CPU performance.
    /*
    for(int run_count = 0; run_count < 10; run_count++) {
        lines_lsd.clear();
        int64 start_lsd = getTickCount();
        lsd->detect(src, lines_lsd);
        // Detect the lines with LSD
        double freq = getTickFrequency();
        double duration_ms_lsd = double(getTickCount() - start_lsd) * 1000 / freq;
        std::cout << "Elapsed time for LSD: " << duration_ms_lsd << " ms." << std::endl;
        lines_fld.clear();
        int64 start = getTickCount();
        // Detect the lines with FLD
        fld->detect(src, lines_fld);
        double duration_ms = double(getTickCount() - start) * 1000 / freq;
        std::cout << "Ealpsed time for FLD " << duration_ms << " ms." << std::endl;
    }
    */
    // Show results
    imshow("Source", cdst);
    //imshow("Detected Lines (in red) - Standard Hough Line Transform", cdst);
    Size size(100,100);//the dst image size,e.g.100x100
    Mat temppic;
    resize(cdstP,temppic,size);//resize image

    imshow("Detected Lines (in red) - Probabilistic Line Transform", test1);

    // Show found lines with LSD
    Mat line_image_lsd(src);
    lsd->drawSegments(line_image_lsd, lines_lsd);
    //imshow("LSD result", line_image_lsd);
    // Show found lines with FLD
    Mat line_image_fld(src);
    fld->drawSegments(line_image_fld, lines_fld);
    //imshow("FLD result", line_image_fld);

    struct myclass {
        bool operator() (cv::Vec4i ystart, cv::Vec4i yend) { return (ystart[0] < yend[2]) ;}
    } myobject;
    Mat square(Size(src.cols,src.rows),CV_64FC1);;
    vector<Vec4i> horisontal;

    for(int i = 0; i < linesP.size(); i++)
    {
        Vec4i l = linesP[i];
        if(l[0] != l[2] || l[1] != l[3])
        {
            horisontal.push_back(l);
        }
    }

    sort(horisontal.begin(), horisontal.end(), myobject);

    for(int i = 0; i< horisontal.size(); i++)
    {
        Vec4i l = horisontal[i];

            cout << "start x: " << l[0] << " start y: " << l[1] << endl;
            cout << "end x: " << l[2] << " end y: " << l[3] << endl;


    }
    int tempi = 0;
    Vec4i l1;
    for(int i = 0; i < horisontal.size(); i++)
    {
        Vec4i l = linesP[i];

            l1 = linesP[++tempi];

        rectangle(square, Point(l[0],l[1]), Point(l1[2],l1[3]), 255, 1, 8, 0);
    }
    imshow("Tegnede firkanter", square);
    sort(linesP.begin(), linesP.end(), myobject);
    //Show the coordinates for houghlines
    /*
    for(int i = 0; i< linesP.size(); i++)
    {
        Vec4i l = linesP[i];
        if(l[0] != l[2] || l[1] != l[3])
        {
            cout << "horisontal" << endl;
            cout << "start x: " << l[0] << " start y: " << l[1] << endl;
            cout << "end x: " << l[2] << " end y: " << l[3] << endl;
        }
        else
        {
            cout << "vertical" << endl;
            cout << "start x: " << l[0] << " start y: " << l[1] << endl;
            cout << "end x: " << l[2] << " end y: " << l[3] << endl;
        }

    }
    */
    // Wait and Exit
    waitKey();
    return 0;
}
