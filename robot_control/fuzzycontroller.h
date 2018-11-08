#ifndef FUZZYCONTROLLER_H
#define FUZZYCONTROLLER_H

#include "fl/Headers.h"
#include <math.h>
#include "helpstructs.h"

using namespace fl;

class FuzzyController
{
public:
    FuzzyController();
    FuzzyController(int mode);
    void setDistanceToClosestObstacle(float distance);
    void setAngleToClosestObstacle(float angle);
    void setRelativeAngleToGoal(double angle);
    void setRelativeDistanceToGoal(double dist);
    void setPath(double path);
    double getRelativeAngleToGoal();
    double getRelativeDistanceToGoal();
    double getAngleToClosestObstacle();
    double getDistanceToClosestObstacle();
    float getDirection();
    float getSpeed();
    void calcRelativeVectorToGoal(coordinate robot, coordinate goal);
		_vec getRelativeVectorToGoal();
    void process();
    bool is_at_goal();

private:
		_vec relativeVector;
    float left_border = M_PI/6;
    float left_border2 = M_PI/2;
    float right_border = -M_PI/6;
    float right_border2 = -M_PI/2;
    float range_border = 0.20;
    float angle_max = 2.2689;
    float angle_min = -2.26889;
    float range_min = 0.08;
    float range_max = 10;

    Engine* engine                  = nullptr;

    InputVariable* obstacle         = nullptr;  // This is used to define the angle to the closest obstacle     |   LIDAR
    InputVariable* distance         = nullptr;  // Distance to the closest obstacle                             |   LIDAR
    InputVariable* rel_angle        = nullptr;  // Relative angle between the robot and the goal                |   CALCULATED
    InputVariable* rel_dist         = nullptr;  // Relative distance between the robot and the goal             |   CALCULATED
    InputVariable* path             = nullptr;  // Tell the Fuzzy Controller, what way to go around obstacle.

    OutputVariable* Speed           = nullptr;
    OutputVariable* direction       = nullptr;
    RuleBlock* mamdani              = nullptr;

};

#endif // FUZZYCONTROLLER_H
