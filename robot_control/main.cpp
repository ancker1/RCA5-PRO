#include <gazebo/gazebo_client.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/transport/transport.hh>

#include <opencv2/opencv.hpp>
#include "fl/Headers.h"
#include "fuzzycontroller.h"

#include <iostream>

#include <math.h>
#include "circledetection.h"

static boost::mutex mutex;
float arr[7] = { 0 };
coordinate robot;
double robot_oz;
Mat map;
CircleDetection cd;
vector<circleInfo> circles, h_circles, spr_circles;

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
			std::cout << std::setprecision(2) << std::fixed << std::setw(6)
								<< _msg->pose(i).position().x() << std::setw(6)
								<< _msg->pose(i).position().y() << std::setw(6) << std::endl;
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

	static vector<circleInfo> g_circles;
	if (g_circles.empty()) {
		g_circles.push_back(circleInfo());
		g_circles[g_circles.size() - 1].map_x = -23.012800;
		g_circles[g_circles.size() - 1].map_y = -7.302660;
		g_circles.push_back(circleInfo());
		g_circles[g_circles.size() - 1].map_x = -13.204500;
		g_circles[g_circles.size() - 1].map_y = -21.795400;
		g_circles.push_back(circleInfo());
		g_circles[g_circles.size() - 1].map_x = 9.727870;
		g_circles[g_circles.size() - 1].map_y = -13.224000;
		g_circles.push_back(circleInfo());
		g_circles[g_circles.size() - 1].map_x = -15.406200;
		g_circles[g_circles.size() - 1].map_y = 8.608350;
		g_circles.push_back(circleInfo());
		g_circles[g_circles.size() - 1].map_x = 12.738000;
		g_circles[g_circles.size() - 1].map_y = 1.017760;
		g_circles.push_back(circleInfo());
		g_circles[g_circles.size() - 1].map_x = -22.296800;
		g_circles[g_circles.size() - 1].map_y = 13.679100;
		g_circles.push_back(circleInfo());
		g_circles[g_circles.size() - 1].map_x = 24.768400;
		g_circles[g_circles.size() - 1].map_y = -10.048700;
		g_circles.push_back(circleInfo());
		g_circles[g_circles.size() - 1].map_x = -31.299400;
		g_circles[g_circles.size() - 1].map_y = 1.089590;
		g_circles.push_back(circleInfo());
		g_circles[g_circles.size() - 1].map_x = -8.881620;
		g_circles[g_circles.size() - 1].map_y = 6.703360;
		g_circles.push_back(circleInfo());
		g_circles[g_circles.size() - 1].map_x = 1.470190;
		g_circles[g_circles.size() - 1].map_y = -13.739300;
		g_circles.push_back(circleInfo());
		g_circles[g_circles.size() - 1].map_x = -5.166110;
		g_circles[g_circles.size() - 1].map_y = -19.252900;
		g_circles.push_back(circleInfo());
		g_circles[g_circles.size() - 1].map_x = -29.747200;
		g_circles[g_circles.size() - 1].map_y = -4.606750;
		g_circles.push_back(circleInfo());
		g_circles[g_circles.size() - 1].map_x = 30.763600;
		g_circles[g_circles.size() - 1].map_y = 13.046700;
		g_circles.push_back(circleInfo());
		g_circles[g_circles.size() - 1].map_x = 22.635500;
		g_circles[g_circles.size() - 1].map_y = 16.109500;
		g_circles.push_back(circleInfo());
		g_circles[g_circles.size() - 1].map_x = 6.910670;
		g_circles[g_circles.size() - 1].map_y = -12.045200;
		g_circles.push_back(circleInfo());
		g_circles[g_circles.size() - 1].map_x = -14.677000;
		g_circles[g_circles.size() - 1].map_y = -6.956180;
		g_circles.push_back(circleInfo());
		g_circles[g_circles.size() - 1].map_x = -14.137900;
		g_circles[g_circles.size() - 1].map_y = -20.387100;
		g_circles.push_back(circleInfo());
		g_circles[g_circles.size() - 1].map_x = -30.211900;
		g_circles[g_circles.size() - 1].map_y = 16.560600;
		g_circles.push_back(circleInfo());
		g_circles[g_circles.size() - 1].map_x = 21.987800;
		g_circles[g_circles.size() - 1].map_y = -5.655060;
		g_circles.push_back(circleInfo());
		g_circles[g_circles.size() - 1].map_x = 20.138100;
		g_circles[g_circles.size() - 1].map_y = 17.396600;

		for (int i = 0; i < g_circles.size(); i++) {
			g_circles[i].map_x = map.cols / 2 + g_circles[i].map_x * M_TO_PIX;
			g_circles[i].map_y = map.rows / 2 - g_circles[i].map_y * M_TO_PIX;
		}
	}




	//double h_err = 0;
	//double spr_err = 0;
	//double mod_err = 0;

	vector<circleInfo> h_spottedCircles = cd.detectCircles(im, CD_HOUGH);
	if (!h_spottedCircles.empty()) {
		cd.calcCirclePositions(h_spottedCircles, im, map, robot.x, robot.y, robot_oz);
		cd.mergeMarbles(h_circles, h_spottedCircles);
		//h_err = cd.error(h_spottedCircles, g_circles);
	}

	vector<circleInfo> spr_spottedCircles = cd.detectCircles(im, CD_SPR);
	if (!spr_spottedCircles.empty()) {
		cd.calcCirclePositions(spr_spottedCircles, im, map, robot.x, robot.y, robot_oz);
		cd.mergeMarbles(spr_circles, spr_spottedCircles);
		//spr_err = cd.error(spr_spottedCircles, g_circles);
	}

	vector<circleInfo> spottedCircles = cd.detectCircles(im, CD_SPR_MOD);
	if (!spottedCircles.empty()) {
		cd.calcCirclePositions(spottedCircles, im, map, robot.x, robot.y, robot_oz);
		cd.mergeMarbles(circles, spottedCircles);
		//mod_err = cd.error(spottedCircles, g_circles);
	}

	double d = sqrt(pow(g_circles[0].map_x - (map.cols / 2 + M_TO_PIX * robot.x), 2) + pow(g_circles[0].map_y - (map.rows / 2 - M_TO_PIX * robot.y), 2));

	/*putText(im, format("d = %5.2f", d), Point(20, 20), FONT_HERSHEY_SIMPLEX, 0.3, Scalar(255, 255, 255));
	putText(im, format("Hough error: %f", h_err), Point(20, 40), FONT_HERSHEY_SIMPLEX, 0.3, Scalar(255, 255, 255));
	putText(im, format("SPR error: %f", spr_err), Point(20, 60), FONT_HERSHEY_SIMPLEX, 0.3, Scalar(255, 255, 255));
	putText(im, format("Modded SPR error: %f", mod_err), Point(20, 80), FONT_HERSHEY_SIMPLEX, 0.3, Scalar(255, 255, 255));*/

	//cd.drawCircles(im, spottedCircles);
	cd.mapMarbles(map, g_circles, h_circles, h_spottedCircles, spr_circles, spr_spottedCircles, circles, spottedCircles);

	im = im.clone();
	cvtColor(im, im, CV_BGR2RGB);

	mutex.lock();
	imshow("camera", im);
	imshow("map", map);
	mutex.unlock();
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

	mutex.lock();
	//cv::imshow("lidar", im);
	mutex.unlock();
}

int main(int _argc, char **_argv) {
	map = imread("../../map_control/big_floor_plan.png");
	if (!map.data) return 1;
	resize(map, map, map.size() * MAP_ENLARGEMENT, 0, 0, INTER_NEAREST);

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
/*
	std::cout << "angle_max: " << angle_max << std::endl;
	std::cout << "angle_min: " << angle_min << std::endl;
	std::cout << "range_min: " << range_min << std::endl;
	std::cout << "range_max: " << range_max << std::endl;
*/

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

		FuzzyController* controller = new FuzzyController(GO_TO_GOAL);

		coordinate goal;    // TEST

		goal.x = -7;        // TEST
		goal.y = 8;         // TEST


		controller->calcRelativeVectorToGoal(robot, goal);          // TEST
		_vec ans = controller->getRelativeVectorToGoal();         // TEST
	//  std::cout << "Vector length: " << ans.length << std::endl;  // TEST
	//  std::cout << "Vector angle: " <<  ans.angle << std::endl;   // TEST

		std::ofstream xTXT("x.txt");
		std::ofstream yTXT("y.txt");
		std::ofstream tTXT("t.txt");
		std::ofstream lTXT("l.txt");
		std::ofstream aTXT("a.txt");
		xTXT.close();
		yTXT.close();
		tTXT.close();
		lTXT.close();
		aTXT.close();

		int count = 0;
		double time = 0;
		double relang = 0;

		controller->setPath(PATH_L);

	// Loop
	while (true) {
		gazebo::common::Time::MSleep(10);   // MSleep(10) = 10 [ms] sleep.

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


		std::cout << "ang. bw: " << ans.angle << std::endl;
		std::cout << "robot orient: " << robot_oz << std::endl;



		if (robot_oz > 0 && ans.angle > 0)              // Optimize this.
				relang = ans.angle - robot_oz;
		else if (robot_oz > 0 && ans.angle < 0)
				relang = ans.angle + robot_oz;
		else if (robot_oz < 0 && ans.angle > 0)
				relang = ans.angle + robot_oz;
		else if (robot_oz < 0 && ans.angle < 0)
				relang = ans.angle - robot_oz;

													// Comment out when not using GO_TO_GOAL
		std::cout << "Obstacle: " << controller->getAngleToClosestObstacle() << std::endl               // Comment out when not using GO_TO_GOAL
							<< "Distance: " << controller->getDistanceToClosestObstacle() << std::endl               // Comment out when not using GO_TO_GOAL
							<< "RelDist: " << controller->getRelativeDistanceToGoal() << std::endl;// Comment out when not using GO_TO_GOAL
		controller->setRelativeAngleToGoal(relang);                 // Comment out when not using GO_TO_GOAL
		controller->setRelativeDistanceToGoal(ans.length);          // Comment out when not using GO_TO_GOAL

		controller->process();
		/*************************************************************/
		/*       Output variables of Fuzzy Controller is set         */
		/*************************************************************/
		//speed = controller->getSpeed();
		//dir = controller->getDirection();
/*
		std::cout << "RelAngle: " << controller->getRelativeAngleToGoal() << std::endl
							<< "RelDist: "  << controller->getRelativeDistanceToGoal() << std::endl;

		std::cout << "Dir: " << dir << std::endl;
		*/
		/*************************************************************/
		/*       The following is used for testing purposes          */
		/*************************************************************/


		if (controller->is_at_goal())
		{
				controller->setPath(PATH_S);
				goal.x = -7;        // TEST
				goal.y = 12;         // TEST
		}


		mutex.lock();
		int key = cv::waitKey(1);
		mutex.unlock();

		if (key == key_esc) {
			// Display map
			/*for (int i = 0; i < 3; i++) {
				putText(map, format("%2d-%2d: %d", i * 10, i * 10 + 9, detections[i]), Point(10, (i + 2) * 15), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255, 0, 0));
			}
			putText(map, format("  30+: %d", detections[3]), Point(10, 5 * 15), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255, 0, 0));

			imshow("Map", map);*/
			break;
		}

		if ((key == key_up) && (speed <= 1.2f)) speed += 0.05;
		else if ((key == key_down) && (speed >= -1.2f)) speed -= 0.05;
		else if ((key == key_right) && (dir <= 0.4f)) dir += 0.05;
		else if ((key == key_left) && (dir >= -0.4f)) dir -= 0.05;
		else {
			// slow down
			//      speed *= 0.1;
			//      dir *= 0.1;
		}

		// Generate a pose
		ignition::math::Pose3d pose(double(speed), 0, 0, 0, 0, double(dir));

		// Convert to a pose message
		gazebo::msgs::Pose msg;
		gazebo::msgs::Set(&msg, pose);
		movementPublisher->Publish(msg);


		if ( count == 10 && ans.length > 0.1 )
		{
				std::cout << "0.1S" << std::endl;
				xTXT.open("x.txt", std::ios_base::app);
				yTXT.open("y.txt", std::ios_base::app);
				if (xTXT.is_open() && yTXT.is_open())
				{
						xTXT << robot.x << "\n";
						yTXT << robot.y << "\n";
				}
				else
						std::cout << "FAIL" << std::endl;
				xTXT.close();
				yTXT.close();

				std::cout << "ALT" << std::endl;
				tTXT.open("t.txt", std::ios_base::app);
				lTXT.open("l.txt", std::ios_base::app);
				aTXT.open("a.txt", std::ios_base::app);
				if (tTXT.is_open() && lTXT.is_open() && aTXT.is_open())
				{
						tTXT << time << "\n";
						lTXT << ans.length << "\n";
						aTXT << relang << "\n";
				}
				else
						std::cout << "FAIL" << std::endl;
				count = 0;
				tTXT.close();
				lTXT.close();
				aTXT.close();
		}


	}
	// Make sure to shut everything down.
	gazebo::client::shutdown();
}
