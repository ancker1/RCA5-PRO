#include "fuzzycontroller.h"

FuzzyController::FuzzyController()
{

}


void FuzzyController::calcRelativeVectorToGoal(float robot_x, float robot_y, float goal_x, float goal_y)
{
    vector RelativeVector;
    float DeltaX = goal_x - robot_x;
    float DeltaY = goal_y - robot_y;
    RelativeVector.angle = (float)atan((DeltaY)/(DeltaX));
    RelativeVector.length = (float)sqrt(pow(DeltaX,2)+pow(DeltaY,2));
    this->relativeVector = RelativeVector;
}

vector FuzzyController::getRelativeVectorToGoal()
{
    return this->relativeVector;
}

float FuzzyController::getSpeed()
{
    return this->speed;
}

float FuzzyController::getDirection()
{
    return this->direction;
}

void FuzzyController::setDistanceToClosestObstacle(float distance)
{
    this->distanceToClosestObstacle = distance;
}

void FuzzyController::setAngleToClosestObstacle(float angle)
{
    this->angleToClosestObstacle = angle;
}
