#include "fuzzycontroller.h"

FuzzyController::FuzzyController()
{
    engine = new Engine;
    engine->setName("Default engine");
    engine->setDescription("Default engine. Fuzzy Controller without mode set.");

    obstacle = new InputVariable;
    obstacle->setName("Obstacle");
    obstacle->setDescription("Indicates relative position of nearest obstacle");
    obstacle->setEnabled(true);
    obstacle->setRange(angle_min, angle_max);
    obstacle->setLockValueInRange(false);
    obstacle->addTerm(new Rectangle("OKLeft", left_border2, angle_max));
    obstacle->addTerm(new Ramp("Left", float(M_PI/60), left_border2));
    obstacle->addTerm(new Rectangle("OKRight", angle_min, right_border2));
    obstacle->addTerm(new Rectangle("Right", right_border2, float(-M_PI/60)));
    obstacle->addTerm(new Trapezoid("Front", right_border, float(-M_PI/60), float(M_PI/60), left_border));
    engine->addInputVariable(obstacle);

    distance = new InputVariable;
    distance->setName("Distance");
    distance->setDescription("Distance to nearest obstacle");
    distance->setEnabled(true);
    distance->setRange(range_min, range_max);
    distance->setLockValueInRange(false);
    distance->addTerm(new Rectangle("Close", range_min, range_border));
    distance->addTerm(new Triangle("Medium", range_border, 1));
    distance->addTerm(new Ramp("Far", 0.8, range_max));
    engine->addInputVariable(distance);

    Speed = new OutputVariable;
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

    direction = new OutputVariable;
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

    mamdani = new RuleBlock;
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

}

FuzzyController::FuzzyController(int mode)
{
    switch ( mode )
    {
    case GO_TO_GOAL:

        break;
    case FOLLOW_WALL:

        break;
    default:
        break;
    }

}

void FuzzyController::process()
{
    engine->process();
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
    speed = (float)Speed->getValue();
    return this->speed;
}

float FuzzyController::getDirection()
{
    this->dir = (float)direction->getValue();
    return this->dir;
}

void FuzzyController::setDistanceToClosestObstacle(float dist)
{
    this->distanceToClosestObstacle = dist;
    distance->setValue(dist);
}

void FuzzyController::setAngleToClosestObstacle(float angle)
{
    this->angleToClosestObstacle = angle;
    obstacle->setValue(angle);
}
