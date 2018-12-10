#include <gazebo/gazebo_client.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/transport/transport.hh>

#include <opencv2/opencv.hpp>
#include "fl/Headers.h"
#include "fuzzycontroller.h"
#include "path_planning.h"

#include <iostream>
#include <random>

#include <math.h>
//#include "circledetection.h"

static boost::mutex mutex_cam;
float arr[7] = { 0 };
coordinate robot;
double robot_oz;
//Mat map;
int detections[4];

std::default_random_engine generator(time(NULL));

void statCallback(ConstWorldStatisticsPtr& _msg) {
    (void)_msg;

    // Dump the message contents to stdout.
    //  std::cout << _msg->DebugString();
    //  std::cout << std::flush;
}

void poseCallback(ConstPosesStampedPtr& _msg) {
    // Dump the message contents to stdout.
    //  std::cout << _msg->DebugString();

    for (int i = 0; i < _msg->pose_size(); i++) {
        if (_msg->pose(i).name() == "pioneer2dx") {
        //	std::cout << std::setprecision(2) << std::fixed << std::setw(6)
        //						<< _msg->pose(i).position().x() << std::setw(6)
        //						<< _msg->pose(i).position().y() << std::setw(6) << std::endl;
                                     /*
                                << _msg->pose(i).position().z() << std::setw(6)
                                << _msg->pose(i).orientation().w() << std::setw(6)
                                << _msg->pose(i).orientation().x() << std::setw(6)
                                << _msg->pose(i).orientation().y() << std::setw(6)
                                << _msg->pose(i).orientation().z() << std::endl;
                                */
            //std::cout << "_________________________________" << std::endl;
            //std::cout << "X: " << _msg->pose(i).position().x() << std::endl << "Y: " << _msg->pose(i).position().y() << std::endl << "Z: " << _msg->pose(i).position().z() << std::endl;
            //std::cout << "O_X: " << _msg->pose(i).orientation().x() << std::endl << "O_Y: " << _msg->pose(i).orientation().y() << std::endl << "O_Z: " << _msg->pose(i).orientation().z() << std::endl << "O_W: " << _msg->pose(i).orientation().w() << std::endl;
        //std::cout << "O_Z: " << _msg->pose(i).orientation().z() << std::endl;
        //std::cout << "O_W: " << _msg->pose(i).orientation().w() << std::endl;

        double x = double(_msg->pose(i).orientation().x());
        double y = double(_msg->pose(i).orientation().y());
        double z = double(_msg->pose(i).orientation().z());
        double w = double(_msg->pose(i).orientation().w());

        double siny_cosp = 2*(w*z+x*y);
        double cosy_cosp = 1-2*(y*y+z*z);

        double yaw = atan2(siny_cosp,cosy_cosp); // Convert quaternion to roll-pitch-yaw.
        robot_oz = yaw; // Testing purposes.     MAKE A CLASS CALLED ROBOT?
        //std::cout << "yaw: " << yaw << std::endl;  // This is the rotation about the z-axis. Going from -Pi to Pi.
        //std::cout << "x: " << _msg->pose(i).position().x() << std::endl;
        //std::cout << "y: " << _msg->pose(i).position().y() << std::endl;
        robot.x = double(_msg->pose(i).position().x()); // This will get the current coordinate of the robot
        robot.y = double(_msg->pose(i).position().y()); // This will get the current coordinate of the robot


        }
    }
}

void cameraCallback(ConstImageStampedPtr& msg) {
    std::size_t width  = msg->image().width();
    std::size_t height = msg->image().height();
    const char *data   = msg->image().data().c_str();
    cv::Mat     im(int(height), int(width), CV_8UC3, const_cast<char *>(data));

//	CircleDetection    cd;
//    vector<circleInfo> circles = cd.detectCircles(im, CD_HOUGH);
//	cd.drawCircles(im, circles);
//	cd.mapMarbles(map, robot.x, robot.y, robot_oz, circles, detections);

    im = im.clone();
    cvtColor(im, im, CV_BGR2RGB);

    mutex_cam.lock();
    imshow("camera", im);
    mutex_cam.unlock();
}

void lidarCallback(ConstLaserScanStampedPtr &msg) {
    //  std::cout << ">> " << msg->DebugString() << std::endl;
    float angle_min = float(msg->scan().angle_min());

    //  double angle_max = msg->scan().angle_max();
    float angle_increment = float(msg->scan().angle_step());

    float range_min = float(msg->scan().range_min());
    float range_max = float(msg->scan().range_max());

    int sec  = msg->time().sec();
    int nsec = msg->time().nsec();

    int nranges      = msg->scan().ranges_size();
    int nintensities = msg->scan().intensities_size();

    assert(nranges == nintensities);

    int   width    = 400;
    int   height   = 400;
    float px_per_m = 200 / range_max;

    float c_range = range_max;
    float c_angle;
    cv::Mat im(height, width, CV_8UC3);
    im.setTo(0);

    for (int i = 0; i < nranges; i++) {
        float angle = angle_min + i * angle_increment;
        float range = std::min(float(msg->scan().ranges(i)), range_max);

        //    double intensity = msg->scan().intensities(i);
        cv::Point2f startpt(200.5f + range_min *px_per_m * std::cos(angle),
                                                200.5f - range_min *px_per_m * std::sin(angle));
        cv::Point2f endpt(200.5f + range *px_per_m * std::cos(angle),
                                            200.5f - range *px_per_m * std::sin(angle));
        cv::line(im, startpt * 16, endpt * 16, cv::Scalar(255, 255, 255, 255), 1,
                         cv::LINE_AA, 4);

        if (c_range > range)
        {
            c_range = range;
            c_angle = angle;
        }

        //    std::cout << angle << " " << range << " " << intensity << std::endl;
    }
    cv::circle(im, cv::Point(200, 200), 2, cv::Scalar(0, 0, 255));
    cv::putText(im, std::to_string(sec) + ":" + std::to_string(nsec),
                            cv::Point(10, 20), cv::FONT_HERSHEY_PLAIN, 1.0,
                            cv::Scalar(255, 0, 0));

    arr[0] = float(msg->scan().angle_max());
    arr[1] = float(msg->scan().angle_min());
    arr[2] = float(msg->scan().range_min());
    arr[3] = float(msg->scan().range_max());
    arr[4] = float(msg->scan().angle_step()); // angle_increment
    arr[5] = c_angle;
    arr[6] = c_range;

    mutex_cam.lock();
    cv::imshow("lidar", im);
    mutex_cam.unlock();
}



void init_robot(cv::Point2f * start, float * orient, cv::Point2f * goal, gazebo::transport::PublisherPtr movP)
{
    /* GENERATE RANDOM START AND GOAL HERE */
    // Generate a pose
    ignition::math::Pose3d pose(double(0), 0, 0, 0, 0, double(0));
    // Convert to a pose message
    gazebo::msgs::Pose msg;
    gazebo::msgs::Set(&msg, pose);
    movP->Publish(msg);


    /* Test PATH = R */
        /* Start: -6 < x < 2 , 1 < y < 4*/

    std::uniform_real_distribution<float> distribution(-6.0, 1.9);
    start->x = distribution(generator);
    distribution.param(std::uniform_real_distribution<float>::param_type(-1.0, -4.0));
    start->y = distribution(generator);
    distribution.param(std::uniform_real_distribution<float>::param_type(-3.14, 3.14));
    *orient = distribution(generator);
    distribution.param(std::uniform_real_distribution<float>::param_type(3.2, 5.8));
    goal->x = distribution(generator);
    distribution.param(std::uniform_real_distribution<float>::param_type(-1.0, -4.0));
    goal->y =  distribution(generator);

    std::string command = "/home/mikkel/Desktop/RCA5-PRO/gazebo_spawn.sh";
    command += " -x ";
    command += std::to_string(start->x);
    command += " -y ";
    command += std::to_string(start->y);
    command += " -t ";
    command += std::to_string(*orient);
    std::cout << command << endl;
    //command += " -x -2 -y 2 -t 1.57";
    system(command.c_str());

    while( !(  robot.x > start->x - 0.1 && robot.x < start->x + 0.1 && robot.y < start->y + 0.1 && robot.y > start->y - 0.1 ) )
    {
        std::cout << "x" << start->x << std::endl;
        std::cout << "x" << robot.x << std::endl;
        std::cout << "y" << start->y << std::endl;
        std::cout << "y" << robot.y << std::endl;
    }

}


int main(int _argc, char **_argv) {


    // Load gazebo
    gazebo::client::setup(_argc, _argv);

    // Create our node for communication
    gazebo::transport::NodePtr node(new gazebo::transport::Node());
    node->Init();

    // Listen to Gazebo topics
    gazebo::transport::SubscriberPtr statSubscriber =
        node->Subscribe("~/world_stats", statCallback);


    gazebo::transport::SubscriberPtr poseSubscriber =
        node->Subscribe("~/pose/info", poseCallback);

    gazebo::transport::SubscriberPtr cameraSubscriber =
        node->Subscribe("~/pioneer2dx/camera/link/camera/image", cameraCallback);

    gazebo::transport::SubscriberPtr lidarSubscriber =
        node->Subscribe("~/pioneer2dx/hokuyo/link/laser/scan", lidarCallback);

    // Publish to the robot vel_cmd topic
    gazebo::transport::PublisherPtr movementPublisher =
        node->Advertise<gazebo::msgs::Pose>("~/pioneer2dx/vel_cmd");

    // Publish a reset of the world
    gazebo::transport::PublisherPtr worldPublisher =
        node->Advertise<gazebo::msgs::WorldControl>("~/world_control");
    gazebo::msgs::WorldControl controlMessage;
    controlMessage.mutable_reset()->set_all(true);
    worldPublisher->WaitForConnection();
    worldPublisher->Publish(controlMessage);

    float angle_max = arr[0];
    float angle_min = arr[1];
    float range_min = arr[2];
    float range_max = arr[3];


    float left_border = M_PI/6;
    float left_border2 = M_PI/2;
    float right_border = -M_PI/6;
    float right_border2 = -M_PI/2;
    float range_border = 0.35;

    const int key_left  = 81;
    const int key_up    = 82;
    const int key_down  = 84;
    const int key_right = 83;
    const int key_esc   = 27;

    float speed = 0.0;
    float dir   = 0.0;

        FuzzyController* controller = new FuzzyController();

        coordinate goal;    // TEST


        controller->calcRelativeVectorToGoal(robot, goal);          // TEST
        _vec ans = controller->getRelativeVectorToGoal();         // TEST
    //  std::cout << "Vector length: " << ans.length << std::endl;  // TEST
    //  std::cout << "Vector angle: " <<  ans.angle << std::endl;   // TEST


        int count = 0;
        double time = 0;
        double relang = 0;


        cv::Point2f startii;
        float orientation;
        cv::Point2f goalii;
        /*   */
        init_robot(&startii, &orientation, &goalii, movementPublisher);

        int fuzzy_iterator = 0;
        string filex;
        filex = "/home/mikkel/Documents/fuzzytest/ft";
        filex += std::to_string(fuzzy_iterator);
        filex += ".txt";

        string filey;
        filey = "/home/mikkel/Documents/fuzzytest/ft";
        filey += std::to_string(fuzzy_iterator);
        filey += ".txt";


        std::vector<float> pos_x;
        std::vector<float> pos_y;

        goal.x = goalii.x;        // TEST
        goal.y = goalii.y;        // TEST

        goalii.x = round( ( 7   + goalii.x ) * 20/14  );
        goalii.y = round( ( 5.5 - goalii.y ) * 15/11  );

        startii.x = round( ( 7   + startii.x ) * 20/14  );
        startii.y = round( ( 5.5 - startii.y ) * 15/11  );

        Mat small_map = cv::imread( "../map_control/floor_plan.png", IMREAD_COLOR );
        resize(small_map, small_map, small_map.size()*2,0,0,INTER_NEAREST);
        Mat mapclone = small_map.clone();
        mapclone.at<Vec3b>(startii) = Vec3b(0, 255, 0);
        mapclone.at<Vec3b>(goalii) = Vec3b(0, 0, 255);
        resize(mapclone, mapclone, mapclone.size()*10,0,0,INTER_NEAREST);
        imshow("Tekst",mapclone);

        Path_planning* plann = new Path_planning;
        controller->setPath(plann->way_around_obstacle(startii, goalii, small_map));

    // Loop
    while (true)
    {
        gazebo::common::Time::MSleep(10);   // MSleep(10) = 10 [ms] sleep.
        //std::cout << plann->way_around_obstacle(startii, goalii, small_map) << std::endl;
        count++;
        time += 0.010;

        float angle = arr[5];
        float range = arr[6];

        /*************************************************************/
        /*       Input variables of Fuzzy Controller is set          */
        /*************************************************************/


        controller->setDistanceToClosestObstacle(range);
        controller->setAngleToClosestObstacle(angle);

        controller->calcRelativeVectorToGoal(robot, goal);          // Create method that takes robot orient, pos and goal pos.
        _vec ans = controller->getRelativeVectorToGoal();         // ^ should calculate the rel dist and angle directly.


       // std::cout << "R-G: " << ans.angle << std::endl;
       // std::cout << "Robot: " << robot_oz << std::endl;



        relang = ans.angle - robot_oz;
        std::cout << "Relang: " << relang << std::endl;

                                                    // Comment out when not using GO_TO_GOAL
        std::cout << "Obstacle: " << controller->getAngleToClosestObstacle()     << std::endl               // Comment out when not using GO_TO_GOAL
                  << "Distance: " << controller->getDistanceToClosestObstacle()  << std::endl               // Comment out when not using GO_TO_GOAL
                  << "RelDist: "  << controller->getRelativeDistanceToGoal()     << std::endl// Comment out when not using GO_TO_GOAL
                  << "Speed:" << speed << std::endl
                  << "Dir:" << dir << std::endl;
        controller->setRelativeAngleToGoal(relang);                 // Comment out when not using GO_TO_GOAL
        controller->setRelativeDistanceToGoal(ans.length);          // Comment out when not using GO_TO_GOAL

        controller->process();


        /*************************************************************/
        /*       Output variables of Fuzzy Controller is set         */
        /*************************************************************/
        speed = controller->getSpeed();
        dir = controller->getDirection();


        /*************************************************************/
        /*       TEST                                                */
        /*************************************************************/
        if ( ans.length < 0.35 )
        {
            // New file
            fuzzy_iterator++;
            filex = "/home/mikkel/Documents/fuzzytest/ftx";
            filex += std::to_string(fuzzy_iterator);
            filex += ".txt";

            filey = "/home/mikkel/Documents/fuzzytest/fty";
            filey += std::to_string(fuzzy_iterator);
            filey += ".txt";

            ofstream datax;
            ofstream datay;
            datax.open(filex);
            datay.open(filey);
            for( int i = 0; i < pos_x.size(); i++ )
            {
                datax << pos_x[i] << endl;
                datay << pos_y[i] << endl;
            }
            datax.close();
            datay.close();

            pos_x.clear();
            pos_y.clear();

            init_robot(&startii, &orientation, &goalii, movementPublisher);
            goal.x = goalii.x;        // TEST
            goal.y = goalii.y;        // TEST

            goalii.x = round( ( 7   + goalii.x ) * 20/14  );
            goalii.y = round( ( 5.5 - goalii.y ) * 15/11  );

            startii.x = round( ( 7   + startii.x ) * 20/14  );
            startii.y = round( ( 5.5 - startii.y ) * 15/11  );

            controller->setPath(plann->way_around_obstacle(startii, goalii, small_map));
        }

        /*************************************************************/
        /*                        PRINT DATA                         */
        /*************************************************************/
        if (count >= 10)
        {
            pos_x.push_back(robot.x);
            pos_y.push_back(robot.y);
            count = 0;
        }

        /*************************************************************/
        /*       SEND TO GAZEBO                                      */
        /*************************************************************/
        // Generate a pose
        ignition::math::Pose3d pose(double(speed), 0, 0, 0, 0, double(dir));

        // Convert to a pose message
        gazebo::msgs::Pose msg;
        gazebo::msgs::Set(&msg, pose);
        movementPublisher->Publish(msg);

        mutex_cam.lock();
        int key = cv::waitKey(1);
        mutex_cam.unlock();

    }

    // Make sure to shut everything down.
    gazebo::client::shutdown();
}
