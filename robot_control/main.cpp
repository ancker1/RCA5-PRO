#include <gazebo/gazebo_client.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/transport/transport.hh>

#include <opencv2/opencv.hpp>
#include "fl/Headers.h"

#include <iostream>

#include <math.h>

static boost::mutex mutex;

void statCallback(ConstWorldStatisticsPtr &_msg) {
  (void)_msg;
  // Dump the message contents to stdout.
  //  std::cout << _msg->DebugString();
  //  std::cout << std::flush;
}

void poseCallback(ConstPosesStampedPtr &_msg) {
  // Dump the message contents to stdout.
  //  std::cout << _msg->DebugString();

  for (int i = 0; i < _msg->pose_size(); i++) {
    if (_msg->pose(i).name() == "pioneer2dx") {

      std::cout << std::setprecision(2) << std::fixed << std::setw(6)
                << _msg->pose(i).position().x() << std::setw(6)
                << _msg->pose(i).position().y() << std::setw(6)
                << _msg->pose(i).position().z() << std::setw(6)
                << _msg->pose(i).orientation().w() << std::setw(6)
                << _msg->pose(i).orientation().x() << std::setw(6)
                << _msg->pose(i).orientation().y() << std::setw(6)
                << _msg->pose(i).orientation().z() << std::endl;
    }
  }
}

void cameraCallback(ConstImageStampedPtr &msg) {

  std::size_t width = msg->image().width();
  std::size_t height = msg->image().height();
  const char *data = msg->image().data().c_str();
  cv::Mat im(int(height), int(width), CV_8UC3, const_cast<char *>(data));

  im = im.clone();
  cv::cvtColor(im, im, CV_BGR2RGB);

  mutex.lock();
  cv::imshow("camera", im);
  mutex.unlock();
}

float arr[7] = { 0 };

void lidarCallback(ConstLaserScanStampedPtr &msg) {

  //  std::cout << ">> " << msg->DebugString() << std::endl;
  float angle_min = float(msg->scan().angle_min());
  //  double angle_max = msg->scan().angle_max();
  float angle_increment = float(msg->scan().angle_step());

  float range_min = float(msg->scan().range_min());
  float range_max = float(msg->scan().range_max());

  int sec = msg->time().sec();
  int nsec = msg->time().nsec();

  int nranges = msg->scan().ranges_size();
  int nintensities = msg->scan().intensities_size();

  assert(nranges == nintensities);

  int width = 400;
  int height = 400;
  float px_per_m = 200 / range_max;

  float c_range = range_max;
  float c_angle;

  cv::Mat im(height, width, CV_8UC3);
  im.setTo(0);
  for (int i = 0; i < nranges; i++) {
    float angle = angle_min + i * angle_increment;
    float range = std::min(float(msg->scan().ranges(i)), range_max);
    //    double intensity = msg->scan().intensities(i);
    cv::Point2f startpt(200.5f + range_min * px_per_m * std::cos(angle),
                        200.5f - range_min * px_per_m * std::sin(angle));
    cv::Point2f endpt(200.5f + range * px_per_m * std::cos(angle),
                      200.5f - range * px_per_m * std::sin(angle));
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
  arr[4] = float(msg->scan().angle_step());   // angle_increment
  arr[5] = c_angle;
  arr[6] = c_range;

  mutex.lock();
  cv::imshow("lidar", im);
  mutex.unlock();
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
  using namespace fl;
  Engine* engine = new Engine;
  engine->setName("SimpleTest");
  engine->setDescription("First test of a Fuzzy Controller");

  /*   INPUT VARIABLES   */
  InputVariable* obstacle = new InputVariable;
  obstacle->setName("Obstacle");
  obstacle->setDescription("Indicates relative position of nearest obstacle");
  obstacle->setEnabled(true);
  obstacle->setRange(angle_min, angle_max);
  obstacle->setLockValueInRange(false);
  /*
  obstacle->addTerm(new Rectangle("Left", left_border, left_border2));
  obstacle->addTerm(new Rectangle("OKLeft", left_border2, angle_max));
  obstacle->addTerm(new Rectangle("Front", right_border, left_border));
  obstacle->addTerm(new Rectangle("Right", right_border2, right_border));
  obstacle->addTerm(new Rectangle("OKRight", angle_min, right_border2));
  */
  obstacle->addTerm(new Rectangle("OKLeft", left_border2, angle_max));
  obstacle->addTerm(new Ramp("Left", float(M_PI/60), left_border2));

  obstacle->addTerm(new Rectangle("OKRight", angle_min, right_border2));
  obstacle->addTerm(new Rectangle("Right", right_border2, float(-M_PI/60)));

  obstacle->addTerm(new Trapezoid("Front", right_border, float(-M_PI/60), float(M_PI/60), left_border));

  engine->addInputVariable(obstacle);

  InputVariable* distance = new InputVariable;
  distance->setName("Distance");
  distance->setDescription("Distance to nearest obstacle");
  distance->setEnabled(true);
  distance->setRange(range_min, range_max);
  distance->setLockValueInRange(false);
  /*
  distance->addTerm(new Rectangle("Close", range_min, range_border));
  distance->addTerm(new Rectangle("Far", range_border, range_max));
  */
  distance->addTerm(new Rectangle("Close", range_min, range_border));
  distance->addTerm(new Triangle("Medium", range_border, 1));
  distance->addTerm(new Ramp("Far", 0.8, range_max));
  engine->addInputVariable(distance);

  /*   OUTPUT VARIABLES  */
  OutputVariable* Speed = new OutputVariable;
  Speed->setName("Speed");
  Speed->setDescription("Translational speed of robot");
  Speed->setEnabled(true);
  Speed->setRange(0,1);
  Speed->setLockValueInRange(false);
  Speed->setAggregation(new Maximum);
  Speed->setDefuzzifier(new Centroid(100));
  Speed->setDefaultValue(0);
  Speed->addTerm(new Rectangle("Go", 0.01, 1));
  Speed->addTerm(new Rectangle("Stop", 0, 0.01));
  engine->addOutputVariable(Speed);

  OutputVariable* direction = new OutputVariable;
  direction->setName("Direction");
  direction->setDescription("Rotational speed of the robot");
  direction->setEnabled(true);
  direction->setRange(0,0.5);
  direction->setLockValueInRange(false);
  direction->setAggregation(new Maximum);
  direction->setDefuzzifier(new Centroid(100));
  direction->setDefaultValue(0);
  direction->addTerm(new Rectangle("Right", 0.01, 0.5));
  direction->addTerm(new Rectangle("Front", 0, 0.01));
  engine->addOutputVariable(direction);

  /*   Ruleblock   */
  RuleBlock* mamdani = new RuleBlock;
  mamdani->setName("Mamdani");
  mamdani->setDescription("");
  mamdani->setEnabled(true);
  mamdani->setConjunction(new Minimum);
  mamdani->setDisjunction(fl::null);
  mamdani->setImplication(new AlgebraicProduct);
  mamdani->setActivation(new General);
  mamdani->addRule(Rule::parse("if Distance is Far then Speed is Go and Direction is Front", engine));

  mamdani->addRule(Rule::parse("if Distance is Medium and Obstacle is Front then Speed is Go and Direction is Right", engine));
  mamdani->addRule(Rule::parse("if Distance is Medium and Obstacle is Left then Speed is Go and Direction is Front", engine)); // small_right
  mamdani->addRule(Rule::parse("if Distance is Medium and Obstacle is Right then Speed is Go and Direction is Front", engine)); // small_left
  mamdani->addRule(Rule::parse("if Distance is Medium and Obstacle is OKLeft then Speed is Go and Direction is Front", engine));
  mamdani->addRule(Rule::parse("if Distance is Medium and Obstacle is OKRight then Speed is Go and Direction is Front", engine));

  mamdani->addRule(Rule::parse("if Distance is Close and Obstacle is Front then Speed is Stop and Direction is Right", engine));
  mamdani->addRule(Rule::parse("if Distance is Close and Obstacle is Left then Speed is Stop and Direction is Right", engine));

  engine->addRuleBlock(mamdani);


  const int key_left = 81;
  const int key_up = 82;
  const int key_down = 84;
  const int key_right = 83;
  const int key_esc = 27;

  float speed = 0.0;
  float dir = 0.0;

  // Loop
  while (true) {
    gazebo::common::Time::MSleep(10);

    float angle = arr[5];
    float range = arr[6];

    distance->setValue(range);
    obstacle->setValue(angle);
    engine->process();

    speed = Speed->getValue();
    dir = direction->getValue();

    //FL_LOG("Distance.input = " << Op::str(distance->getValue()) << " Obstacle.input = " << Op::str(obstacle->getValue()) << " Speed.output =  " << Op::str(Speed->getValue()) << "Direction.output = " << Op::str(direction->getValue()) );

    std::cout << "Speed: " << speed << std::endl;
    std::cout << "Direction: " << dir << std::endl;
    std::cout << "angle: " << angle << std::endl;
    std::cout << "range: " << range << std::endl;


    mutex.lock();
    int key = cv::waitKey(1);
    mutex.unlock();

    if (key == key_esc)
      break;

    if ((key == key_up) && (speed <= 1.2f))
      speed += 0.05;
    else if ((key == key_down) && (speed >= -1.2f))
      speed -= 0.05;
    else if ((key == key_right) && (dir <= 0.4f))
      dir += 0.05;
    else if ((key == key_left) && (dir >= -0.4f))
      dir -= 0.05;
    else {
      // slow down
      //      speed *= 0.1;
      //      dir *= 0.1;
    }

    if ( key == 'w' )
        speed += 0.5;
    else if ( key == 's' )
        speed -= 0.5;
    else if (key == 'a')
        dir -= 0.5;
    else if (key == 'd')
        dir += 0.5;

    // Generate a pose
    ignition::math::Pose3d pose(double(speed), 0, 0, 0, 0, double(dir));

    // Convert to a pose message
    gazebo::msgs::Pose msg;
    gazebo::msgs::Set(&msg, pose);
    movementPublisher->Publish(msg);
  }

  // Make sure to shut everything down.
  gazebo::client::shutdown();
}