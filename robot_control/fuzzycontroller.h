#ifndef FUZZYCONTROLLER_H
#define FUZZYCONTROLLER_H

#include "fl/Headers.h"
#include <math.h>

struct vector
{
    float length;
    float angle;
};

struct coordinate
{
    float x;
    float y;
};

class FuzzyController
{
public:
    FuzzyController();
    //FuzzyController(int mode);
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
private:
    float speed;
    float direction;
    float distanceToClosestObstacle;
    float angleToClosestObstacle;
    vector relativeVector;

};

#endif // FUZZYCONTROLLER_H
