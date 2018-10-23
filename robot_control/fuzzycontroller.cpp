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
        engine = new Engine;
        engine->setName("Default engine");
        engine->setDescription("Default engine. Fuzzy Controller without mode set.");

        rel_angle = new InputVariable;
        rel_angle->setName("RelAngle");
        rel_angle->setEnabled(true);
        rel_angle->setRange(-M_PI, M_PI);
        rel_angle->setLockValueInRange(false);
        rel_angle->addTerm(new Ramp("L", double(M_PI/60), M_PI));            // Left is defined as 180 [degrees] to 3 [degrees]
        //rel_angle->addTerm(new Rectangle("SL", double(M_PI/60), double(M_PI/50)));            // Left is defined as 180 [degrees] to 3 [degrees]
        rel_angle->addTerm(new Rectangle("R", -M_PI, double(-M_PI/60)));    // Right is defined as -180 [degrees] to -3 [degrees]
        //rel_angle->addTerm(new Rectangle("SR", double(-M_PI/50), double(-M_PI/60)));            // Right is defined as -180 [degrees] to -3 [degrees]
        rel_angle->addTerm(new Triangle("F", double(-M_PI/60), 0, double(M_PI/60))); // Front is defined as being between -3 and 3 [degrees] with peak at 0 [degrees]
        engine->addInputVariable(rel_angle);

        rel_dist = new InputVariable;
        rel_dist->setName("RelDist");
        rel_dist->setEnabled(true);
        rel_dist->setRange(0, 100);
        rel_dist->setLockValueInRange(false);
        rel_dist->addTerm(new Ramp("Large", 0.8, 100));
        rel_dist->addTerm(new Rectangle("Medium", 0.4, 0.8));
        rel_dist->addTerm(new Rectangle("Small", 0.1, 0.4));
        rel_dist->addTerm(new Ramp("Zero", 0, 0.1));
        engine->addInputVariable(rel_dist);

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
        direction->setRange(-0.5,0.5);
        direction->setLockValueInRange(false);
        direction->setAggregation(new Maximum);
        direction->setDefuzzifier(new Centroid(100));
        direction->setDefaultValue(0);
        direction->addTerm(new Rectangle("R",  0.40,  0.50));
        direction->addTerm(new Rectangle("SR", 0.10,  0.40));
        direction->addTerm(new Triangle("F", -0.10, 0, 0.10));
        direction->addTerm(new Rectangle("SL",-0.40, -0.10));
        direction->addTerm(new Rectangle("L", -0.50, -0.40));
        engine->addOutputVariable(direction);

        mamdani = new RuleBlock;
        mamdani->setName("Mamdani");
        mamdani->setDescription("");
        mamdani->setEnabled(true);
        mamdani->setConjunction(new Minimum);
        mamdani->setDisjunction(fl::null);
        mamdani->setImplication(new AlgebraicProduct);
        mamdani->setActivation(new General);

        mamdani->addRule(Rule::parse("if RelAngle is L  then Direction is L ", engine));
        mamdani->addRule(Rule::parse("if RelAngle is R  then Direction is R ", engine));
        //mamdani->addRule(Rule::parse("if RelAngle is SL then Direction is SL ", engine));
        //mamdani->addRule(Rule::parse("if RelAngle is SR then Direction is SR ", engine));
        mamdani->addRule(Rule::parse("if RelAngle is F  then Direction is F ", engine));

        mamdani->addRule(Rule::parse("if RelAngle is F and RelDist is Large then Speed is Go ", engine));
        mamdani->addRule(Rule::parse("if RelAngle is F and RelDist is Medium then Speed is Go ", engine));
        mamdani->addRule(Rule::parse("if RelAngle is F and RelDist is Small then Speed is Go ", engine));
        mamdani->addRule(Rule::parse("if RelAngle is F and RelDist is Zero then Speed is Stop ", engine));


        engine->addRuleBlock(mamdani);

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

void FuzzyController::calcRelativeVectorToGoal(coordinate robot, coordinate goal)
{
    vector RelativeVector;
    double DeltaX = goal.x - robot.x;
    double DeltaY = goal.y - robot.y;
    RelativeVector.angle = (double)atan2(DeltaY,DeltaX);
    RelativeVector.length = (double)sqrt(pow(DeltaX,2)+pow(DeltaY,2));
    this->relativeVector = RelativeVector;
}

vector FuzzyController::getRelativeVectorToGoal()
{
    return this->relativeVector;
}

float FuzzyController::getSpeed()
{
    return (double)Speed->getValue();
}

float FuzzyController::getDirection()
{
    return (double)direction->getValue();
}

void FuzzyController::setDistanceToClosestObstacle(float dist)
{
    distance->setValue(dist);
}

void FuzzyController::setAngleToClosestObstacle(float angle)
{
    obstacle->setValue(angle);
}

void FuzzyController::setRelativeAngleToGoal(double angle)
{
    rel_angle->setValue(angle);
}
void FuzzyController::setRelativeDistanceToGoal(double dist)
{
    rel_dist->setValue(dist);
}

double FuzzyController::getRelativeAngleToGoal()
{
    return (double)rel_angle->getValue();
}
double FuzzyController::getRelativeDistanceToGoal()
{
    return (double)rel_dist->getValue();
}
