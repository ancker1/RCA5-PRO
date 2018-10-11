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
    float getDirection();
    float getSpeed();
    //void setRobotPosition(float x, float y);
    //void setGoalPosition(float x, float y);
    //int getRelativeOrientationToGoal(float robot_x, float robot_y, float goal_x, float goal_y);
    //int getRelativeDistanceToGoal(float robot_x, float robot_y, float goal_x, float goal_y);
    void calcRelativeVectorToGoal(float robot_x, float robot_y, float goal_x, float goal_y);
    vector getRelativeVectorToGoal();
    void process();
private:
    float speed;
    float dir;
    float distanceToClosestObstacle;
    float angleToClosestObstacle;
    vector relativeVector;
    float left_border = M_PI/6;
    float left_border2 = M_PI/2;
    float right_border = -M_PI/6;
    float right_border2 = -M_PI/2;
    float range_border = 0.35;
    float angle_max = 2.2689;
    float angle_min = -2.26889;
    float range_min = 0.08;
    float range_max = 10;

    Engine* engine              = nullptr;
    InputVariable* obstacle     = nullptr;
    InputVariable* distance     = nullptr;
    OutputVariable* Speed       = nullptr;
    OutputVariable* direction   = nullptr;
    RuleBlock* mamdani          = nullptr;

};

#endif // FUZZYCONTROLLER_H
