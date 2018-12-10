#include "fuzzycontroller.h"

FuzzyController::FuzzyController()
{
    engine = new Engine;
    engine->setName("Default engine");
    engine->setDescription("Default engine. Fuzzy Controller without mode set.");

    path = new InputVariable;
    path->setName("Path");
    path->setDescription("Which way to go around obstacle");
    path->setEnabled(true);
    path->setRange(0, 5);
    path->setLockValueInRange(false);
    path->addTerm(new Rectangle("R",    0, 1));
    path->addTerm(new Rectangle("L",    1, 2));
    path->addTerm(new Rectangle("S",    2, 3));
    path->addTerm(new Rectangle("NON",  3, 4));
    path->addTerm(new Rectangle("C",    4, 5));
    engine->addInputVariable(path);

    rel_angle = new InputVariable;
    rel_angle->setName("RelAngle");
    rel_angle->setEnabled(true);
    rel_angle->setRange(-M_PI, M_PI);
    rel_angle->setLockValueInRange(false);
    rel_angle->addTerm(new Ramp("L", M_PI/60, M_PI/4));             // 0 at  3 degree, 1 at  45 degree
    rel_angle->addTerm(new Ramp("R", -M_PI/60, -M_PI/4));           // 0 at -3 degree, 1 at -45 degree
    rel_angle->addTerm(new Triangle("F", -M_PI/30, 0, M_PI/30));    // 0 at -6 degree, 1 at   0 degree, 0 at 6 degree
    engine->addInputVariable(rel_angle);

    rel_dist = new InputVariable;
    rel_dist->setName("RelDist");
    rel_dist->setEnabled(true);
    rel_dist->setRange(0, 100);
    rel_dist->setLockValueInRange(false);
    rel_dist->addTerm(new Ramp("Large", 0.1, 100));
    rel_dist->addTerm(new Ramp("Zero", 0.5, 0));
    engine->addInputVariable(rel_dist);

    obstacle = new InputVariable;
    obstacle->setName("Obstacle");
    obstacle->setDescription("Indicates relative position of nearest obstacle");
    obstacle->setEnabled(true);
    obstacle->setRange(angle_min, angle_max);
    obstacle->setLockValueInRange(false);
    obstacle->addTerm(new Rectangle("OKL", left_border2, angle_max));
    obstacle->addTerm(new Rectangle("OKR", angle_min, right_border2));
    obstacle->addTerm(new Trapezoid("L", M_PI/60, M_PI/4, left_border2, left_border2));
    obstacle->addTerm(new Trapezoid("R", right_border2, right_border2, -M_PI/4, -M_PI/60));
    obstacle->addTerm(new Triangle("F", -M_PI/30, 0, M_PI/30));
    engine->addInputVariable(obstacle);

    distance = new InputVariable;
    distance->setName("Distance");
    distance->setDescription("Distance to nearest obstacle");
    distance->setEnabled(true);
    distance->setRange(range_min, range_max);
    distance->setLockValueInRange(false);
    distance->addTerm(new Ramp("C", 0.5, 0.3));
    distance->addTerm(new Triangle("M", 0.4, 0.7 ,1));
    distance->addTerm(new Ramp("F", 0.75, 1.5));
    engine->addInputVariable(distance);

    Speed = new OutputVariable;
    Speed->setName("Speed");
    Speed->setDescription("Translational speed of robot");
    Speed->setEnabled(true);
    Speed->setRange(0,1.5);
    Speed->setLockValueInRange(false);
    Speed->setAggregation(new Maximum);
    Speed->setDefuzzifier(new Centroid(100));
    Speed->setDefaultValue(0);
    Speed->addTerm(new Triangle("Fast", 1.3, 1.5));
    Speed->addTerm(new Triangle("Go", 0.5, 1.0, 1.3));
    Speed->addTerm(new Triangle("Slow", 0.01, 0.3, 0.5));
    Speed->addTerm(new Ramp("Stop", 0.01, 0));
    engine->addOutputVariable(Speed);

    direction = new OutputVariable;
    direction->setName("Direction");
    direction->setDescription("Rotational speed of the robot");
    direction->setEnabled(true);
    direction->setRange(-0.8,0.8);
    direction->setLockValueInRange(false);
    direction->setAggregation(new Maximum);
    direction->setDefuzzifier(new Centroid(100));
    direction->setDefaultValue(0);
    direction->addTerm(new Ramp("VR",       0.50,  0.80));
    direction->addTerm(new Triangle("R",    0.40,  0.45,  0.50));
    direction->addTerm(new Triangle("SR",   0.10,  0.30,  0.40));
    direction->addTerm(new Triangle("F",   -0.10,  0.00,  0.10));
    direction->addTerm(new Triangle("SL",  -0.40, -0.30, -0.10));
    direction->addTerm(new Triangle("L",   -0.50, -0.45, -0.40));
    direction->addTerm(new Ramp("VL",      -0.50, -0.80));
    engine->addOutputVariable(direction);

    mamdani = new RuleBlock;
    mamdani->setName("Mamdani");
    mamdani->setDescription("");
    mamdani->setEnabled(true);
    mamdani->setConjunction(new Minimum);
    mamdani->setDisjunction(fl::null);
    mamdani->setImplication(new AlgebraicProduct);
    mamdani->setActivation(new General);


    std::vector<std::string> rules;
    rules.push_back("if RelDist is Zero then Direction is F and Speed is Stop");

    /*  CASE: PATH is R  */
        /* ALL FAR CASES */
    rules.push_back("if Path is R and Distance is F and RelAngle is L and RelDist is Large then Direction is VL");
    rules.push_back("if Path is R and Distance is F and RelAngle is R and RelDist is Large then Direction is VR");
    rules.push_back("if Path is R and Distance is F and RelAngle is F and RelDist is Large then Speed is Fast and Direction is F");

        /* ALL MEDIUM CASES */
            /* Speed is always Slow */
    rules.push_back("if Path is R and Distance is M and RelDist is Large then Speed is Slow");
            /* Direction is dependent on obstacle location */
    rules.push_back("if Path is R and Distance is M and Obstacle is R and RelDist is Large then Direction is VR");
    rules.push_back("if Path is R and Distance is M and Obstacle is F and RelDist is Large then Direction is VR");
    rules.push_back("if Path is R and Distance is M and Obstacle is L and RelDist is Large then Direction is VR");
    rules.push_back("if Path is R and Distance is M and Obstacle is OKL and RelAngle is L and RelDist is Large then Direction is VL");


        /* ALL CLOSE CASES */
    rules.push_back("if Path is R and Distance is C and Obstacle is R and RelDist is Large then Direction is VR");
    rules.push_back("if Path is R and Distance is C and Obstacle is F and RelDist is Large then Direction is VR");
    rules.push_back("if Path is R and Distance is C and Obstacle is L and RelDist is Large then Direction is VR");
    rules.push_back("if Path is R and Distance is C and Obstacle is OKL and RelAngle is L and RelDist is Large then Direction is VL and Speed is Slow");
    rules.push_back("if Path is R and Distance is C and Obstacle is OKL and RelAngle is R and RelDist is Large then Direction is VR and Speed is Slow");

        /* GENERAL CASES */
    rules.push_back("if Path is R and Obstacle is OKL and RelAngle is F and RelDist is Large then Speed is Fast");
    rules.push_back("if Path is R and Obstacle is OKR and RelDist is Large then Speed is Go");

    /* CASE: PATH is L */
        /* ALL FAR CASES */
    rules.push_back("if Path is L and Distance is F and RelAngle is L and RelDist is Large then Direction is VL");
    rules.push_back("if Path is L and Distance is F and RelAngle is R and RelDist is Large then Direction is VR");
    rules.push_back("if Path is L and Distance is F and RelAngle is F and RelDist is Large then Speed is Fast and Direction is F");

    /* ALL MEDIUM CASES */
            /* Speed is always Slow */
    rules.push_back("if Path is L and Distance is M and RelDist is Large then Speed is Slow");
            /* Direction is dependent on obstacle location */
    rules.push_back("if Path is L and Distance is M and Obstacle is R and RelDist is Large then Direction is VL");
    rules.push_back("if Path is L and Distance is M and Obstacle is F and RelDist is Large then Direction is VL");
    rules.push_back("if Path is L and Distance is M and Obstacle is L and RelDist is Large then Direction is VL");
    rules.push_back("if Path is L and Distance is M and Obstacle is OKR and RelAngle is R and RelDist is Large then Direction is VR");


        /* ALL CLOSE CASES */
    rules.push_back("if Path is L and Distance is C and Obstacle is R and RelDist is Large then Direction is VL");
    rules.push_back("if Path is L and Distance is C and Obstacle is F and RelDist is Large then Direction is VL");
    rules.push_back("if Path is L and Distance is C and Obstacle is L and RelDist is Large then Direction is VL");
    rules.push_back("if Path is L and Distance is C and Obstacle is OKR and RelAngle is L and RelDist is Large then Direction is VL and Speed is Slow");
    rules.push_back("if Path is L and Distance is C and Obstacle is OKR and RelAngle is R and RelDist is Large then Direction is VR and Speed is Slow");

        /* GENERAL CASES */
    rules.push_back("if Path is L and Obstacle is OKR and RelAngle is F and RelDist is Large then Speed is Fast");
    rules.push_back("if Path is L and Obstacle is OKL and RelDist is Large then Speed is Go");




    /* CASE: PATH is S -> If obstacle is detected, then it will go R around. */
        /* ALL FAR CASES */
    rules.push_back("if Path is S and Distance is F and RelAngle is L and RelDist is Large then Direction is VL");
    rules.push_back("if Path is S and Distance is F and RelAngle is R and RelDist is Large then Direction is VR");
    rules.push_back("if Path is S and Distance is F and RelAngle is F and RelDist is Large then Speed is Fast and Direction is F");

        /* ALL MEDIUM CASES */
            /* Speed is always Slow */
    rules.push_back("if Path is S and Distance is M and RelDist is Large then Speed is Slow");
            /* Direction is dependent on obstacle location */
    rules.push_back("if Path is S and Distance is M and Obstacle is OKL and RelAngle is L and RelDist is Large then Direction is VL");

        /* ALL CLOSE CASES */
    rules.push_back("if Path is S and Distance is C and Obstacle is R and RelDist is Large then Direction is VR");
    rules.push_back("if Path is S and Distance is C and Obstacle is F and RelDist is Large then Direction is VR");
    rules.push_back("if Path is S and Distance is C and Obstacle is L and RelDist is Large then Direction is VR");
    rules.push_back("if Path is S and Distance is C and Obstacle is OKL and RelAngle is L and RelDist is Large then Direction is VL and Speed is Slow");
    rules.push_back("if Path is S and Distance is C and Obstacle is OKL and RelAngle is R and RelDist is Large then Direction is VR and Speed is Slow");

        /* GENERAL CASES */
    rules.push_back("if Path is S and Obstacle is OKL and RelAngle is F and RelDist is Large then Speed is Fast");
    rules.push_back("if Path is S and Obstacle is OKR and RelDist is Large then Speed is Go");



    /* CASE: PATH is C -> Collect marble */
    rules.push_back("if Path is C and RelAngle is L and RelDist is Large then Direction is VL");
    rules.push_back("if Path is C and RelAngle is R and RelDist is Large then Direction is VR");
    rules.push_back("if Path is C and RelAngle is F and RelDist is Large then Speed is Go");




    for( std::string s : rules )
        mamdani->addRule(Rule::parse(s, engine));



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

        path = new InputVariable;
        path->setName("Path");
        path->setDescription("Which way to go around obstacle");
        path->setEnabled(true);
        path->setRange(0, 4);
        path->setLockValueInRange(false);
        path->addTerm(new Rectangle("R",    0, 1));
        path->addTerm(new Rectangle("L",    1, 2));
        path->addTerm(new Rectangle("S",    2, 3));
        path->addTerm(new Rectangle("NON",  3, 4));
        engine->addInputVariable(path);


        rel_angle = new InputVariable;
        rel_angle->setName("RelAngle");
        rel_angle->setEnabled(true);
        rel_angle->setRange(-M_PI, M_PI);
        rel_angle->setLockValueInRange(false);
        rel_angle->addTerm(new Ramp("L", double(M_PI/60), M_PI));            // Left is defined as 180 [degrees] to 3 [degrees]
        rel_angle->addTerm(new Rectangle("R", -M_PI, double(-M_PI/60)));    // Right is defined as -180 [degrees] to -3 [degrees]
        rel_angle->addTerm(new Triangle("F", double(-M_PI/60), 0, double(M_PI/60))); // Front is defined as being between -3 and 3 [degrees] with peak at 0 [degrees]
        engine->addInputVariable(rel_angle);

        rel_dist = new InputVariable;
        rel_dist->setName("RelDist");
        rel_dist->setEnabled(true);
        rel_dist->setRange(0, 100);
        rel_dist->setLockValueInRange(false);
        rel_dist->addTerm(new Ramp("Large", 0.6, 100));
        rel_dist->addTerm(new Triangle("Med", 0.1, 0.4, 0.8));
        rel_dist->addTerm(new Ramp("Zero", 0.1, 0));
        engine->addInputVariable(rel_dist);

        obstacle = new InputVariable;
        obstacle->setName("Obstacle");
        obstacle->setDescription("Indicates relative position of nearest obstacle");
        obstacle->setEnabled(true);
        obstacle->setRange(angle_min, angle_max);
        obstacle->setLockValueInRange(false);
        obstacle->addTerm(new Ramp("OKL", left_border2, angle_max));
        obstacle->addTerm(new Rectangle("L", float(M_PI/60), left_border2));
        obstacle->addTerm(new Ramp("OKR", right_border2, angle_min));
        obstacle->addTerm(new Rectangle("R", right_border2, float(-M_PI/60)));
        obstacle->addTerm(new Trapezoid("F", right_border, float(-M_PI/60), float(M_PI/60), left_border));
        engine->addInputVariable(obstacle);

        distance = new InputVariable;
        distance->setName("Distance");
        distance->setDescription("Distance to nearest obstacle");
        distance->setEnabled(true);
        distance->setRange(range_min, range_max);
        distance->setLockValueInRange(false);
        distance->addTerm(new Rectangle("Clo", range_min, range_border));
        distance->addTerm(new Triangle("Med", range_border, 0.4, 0.8));
        distance->addTerm(new Rectangle("Far", 0.6, range_max));
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
        Speed->addTerm(new Ramp("Go", 0.4, 1));
        Speed->addTerm(new Triangle("Slow", 0.01, 0.3, 0.5));
        Speed->addTerm(new Ramp("Stop", 0.01, 0));
        engine->addOutputVariable(Speed);

        direction = new OutputVariable;
        direction->setName("Direction");
        direction->setDescription("Rotational speed of the robot");
        direction->setEnabled(true);
        direction->setRange(-0.8,0.8);
        direction->setLockValueInRange(false);
        direction->setAggregation(new Maximum);
        direction->setDefuzzifier(new Centroid(100));
        direction->setDefaultValue(0);
        direction->addTerm(new Ramp("VR",       0.50,  0.80));
        direction->addTerm(new Triangle("R",    0.40,  0.45,  0.50));
        direction->addTerm(new Triangle("SR",   0.10,  0.30,  0.40));
        direction->addTerm(new Triangle("F",   -0.10,  0.00,  0.10));
        direction->addTerm(new Triangle("SL",  -0.40, -0.30, -0.10));
        direction->addTerm(new Triangle("L",   -0.50, -0.45, -0.40));
        direction->addTerm(new Ramp("VL",      -0.50, -0.80));
        engine->addOutputVariable(direction);

        mamdani = new RuleBlock;
        mamdani->setName("Mamdani");
        mamdani->setDescription("");
        mamdani->setEnabled(true);
        mamdani->setConjunction(new Minimum);
        mamdani->setDisjunction(fl::null);
        mamdani->setImplication(new AlgebraicProduct);
        mamdani->setActivation(new General);


        mamdani->addRule(Rule::parse("if RelDist is Zero then Direction is F and Speed is Stop", engine));

        /*  CASE:   Path is NON -> make robot face goal  */
        mamdani->addRule(Rule::parse("if Path is NON and RelAngle is L then Direction is L and Speed is Stop", engine));
        mamdani->addRule(Rule::parse("if Path is NON and RelAngle is R then Direction is R and Speed is Stop", engine));

        /*  CASE:   PATH is S  :  Obstacle are far away, we are not currently at the goal: go towards goal or shift orientation toward goal   */
        mamdani->addRule(Rule::parse("if Path is S and RelAngle is L and RelDist is Large then Direction is L and Speed is Go", engine));
        mamdani->addRule(Rule::parse("if Path is S and RelAngle is R and RelDist is Large then Direction is R and Speed is Go", engine));
        mamdani->addRule(Rule::parse("if Path is S and RelAngle is F and RelDist is Large then Direction is F and Speed is Go", engine));
        mamdani->addRule(Rule::parse("if Path is S and RelAngle is L and RelDist is Med then Direction is L and Speed is Stop", engine));
        mamdani->addRule(Rule::parse("if Path is S and RelAngle is R and RelDist is Med then Direction is R and Speed is Stop", engine));
        mamdani->addRule(Rule::parse("if Path is S and RelAngle is F and RelDist is Med then Direction is F and Speed is Slow", engine));



        /*  CASE: PATH is R  */
        // Distance far -> Go toward goal
        mamdani->addRule(Rule::parse("if Path is R and Distance is Far and RelAngle is L and RelDist is Large then Direction is L and Speed is Go", engine));
        mamdani->addRule(Rule::parse("if Path is R and Distance is Far and RelAngle is R and RelDist is Large then Direction is R and Speed is Go", engine));
        mamdani->addRule(Rule::parse("if Path is R and Distance is Far and RelAngle is F and RelDist is Large then Direction is F and Speed is Go", engine));
        // Coming nearer to goal
        mamdani->addRule(Rule::parse("if Path is R and Distance is Far and RelAngle is L and RelDist is Med then Direction is L and Speed is Go", engine));
        mamdani->addRule(Rule::parse("if Path is R and Distance is Far and RelAngle is R and RelDist is Med then Direction is R and Speed is Go", engine));
        mamdani->addRule(Rule::parse("if Path is R and Distance is Far and RelAngle is F and RelDist is Med then Direction is F and Speed is Go", engine));

        // Coming med to front obstacle
        mamdani->addRule(Rule::parse("if Path is R and Distance is Med and Obstacle is F and RelDist is not Zero then Direction is R and Speed is Slow", engine));
        // Coming close to front obstacle
        mamdani->addRule(Rule::parse("if Path is R and Distance is Clo and Obstacle is F and RelDist is not Zero then Direction is R and Speed is Stop", engine));


        mamdani->addRule(Rule::parse("if Path is R and Distance is Med and Obstacle is L and RelAngle is L and RelDist is not Zero then Direction is VR and Speed is Slow", engine));
        mamdani->addRule(Rule::parse("if Path is R and Distance is Med and Obstacle is OKL and RelAngle is L and RelDist is not Zero then Direction is VL and Speed is Slow", engine));
        mamdani->addRule(Rule::parse("if Path is R and Distance is Med and Obstacle is OKL and RelAngle is F and RelDist is not Zero then Direction is F and Speed is Slow", engine));
        mamdani->addRule(Rule::parse("if Path is R and Distance is Med and Obstacle is L and RelAngle is F and RelDist is not Zero then Direction is VR and Speed is Slow", engine));

        mamdani->addRule(Rule::parse("if Path is R and Distance is Clo and Obstacle is L and RelAngle is F and RelDist is not Zero then Direction is R and Speed is Stop", engine));
        mamdani->addRule(Rule::parse("if Path is R and Distance is Clo and Obstacle is L and RelAngle is L and RelDist is not Zero then Direction is R and Speed is Stop", engine));
        mamdani->addRule(Rule::parse("if Path is R and Distance is Clo and Obstacle is OKL and RelAngle is L and RelDist is not Zero then Direction is F and Speed is Slow", engine));
        mamdani->addRule(Rule::parse("if Path is R and Distance is Clo and Obstacle is OKL and RelAngle is F and RelDist is not Zero then Direction is F and Speed is Slow", engine));


        mamdani->addRule(Rule::parse("if Path is R and Distance is Med and Obstacle is L and RelAngle is R and RelDist is Large then Direction is VR and Speed is Slow", engine));
        mamdani->addRule(Rule::parse("if Path is R and Distance is Clo and Obstacle is L and RelAngle is R and RelDist is Large then Direction is R and Speed is Stop", engine));
        mamdani->addRule(Rule::parse("if Path is R and Distance is Med and Obstacle is OKL and RelAngle is R and RelDist is Large then Direction is F and Speed is Go", engine));
        mamdani->addRule(Rule::parse("if Path is R and Distance is Clo and Obstacle is OKL and RelAngle is R and RelDist is Large then Direction is SR and Speed is Go", engine));
        mamdani->addRule(Rule::parse("if Path is R and Distance is Clo and Obstacle is OKL and RelAngle is F and RelDist is Large then Direction is F and Speed is Go", engine));

        mamdani->addRule(Rule::parse("if Path is R and Distance is Clo and Obstacle is R and RelAngle is R and RelDist is Large then Direction is R and Speed is Stop", engine));
        mamdani->addRule(Rule::parse("if Path is R and Distance is Med and Obstacle is R and RelAngle is R and RelDist is Large then Direction is R and Speed is Go", engine));

        mamdani->addRule(Rule::parse("if Path is R and Distance is Clo and Obstacle is OKR and RelAngle is F and RelDist is not Zero then Direction is F and Speed is Slow", engine));
        mamdani->addRule(Rule::parse("if Path is R and Distance is Med and Obstacle is OKR and RelAngle is F and RelDist is not Zero then Direction is F and Speed is Slow", engine));

        mamdani->addRule(Rule::parse("if Path is R and Distance is Clo and Obstacle is R and RelAngle is L and RelDist is Large then Direction is R and Speed is Stop", engine));
        mamdani->addRule(Rule::parse("if Path is R and Distance is Med and Obstacle is R and RelAngle is L and RelDist is Large then Direction is R and Speed is Slow", engine));
        mamdani->addRule(Rule::parse("if Path is R and Distance is Clo and Obstacle is R and RelAngle is F and RelDist is Large then Direction is R and Speed is Stop", engine));
        mamdani->addRule(Rule::parse("if Path is R and Distance is Med and Obstacle is R and RelAngle is F and RelDist is Large then Direction is R and Speed is Slow", engine));

        /*  CASE: PATH is L  */
        mamdani->addRule(Rule::parse("if Path is L and Distance is Far and RelAngle is L and RelDist is Large then Direction is L and Speed is Go", engine));
        mamdani->addRule(Rule::parse("if Path is L and Distance is Far and RelAngle is R and RelDist is Large then Direction is R and Speed is Go", engine));
        mamdani->addRule(Rule::parse("if Path is L and Distance is Far and RelAngle is F and RelDist is Large then Direction is F and Speed is Go", engine));
        // Coming nearer to goal
        mamdani->addRule(Rule::parse("if Path is L and Distance is Far and RelAngle is L and RelDist is Med then Direction is L and Speed is Go", engine));
        mamdani->addRule(Rule::parse("if Path is L and Distance is Far and RelAngle is R and RelDist is Med then Direction is R and Speed is Go", engine));
        mamdani->addRule(Rule::parse("if Path is L and Distance is Far and RelAngle is F and RelDist is Med then Direction is F and Speed is Go", engine));
        // Coming med to front obstacle
        mamdani->addRule(Rule::parse("if Path is L and Distance is Med and Obstacle is F and RelDist is not Zero then Direction is L and Speed is Slow", engine));
        // Coming close to front obstacle
        mamdani->addRule(Rule::parse("if Path is L and Distance is Clo and Obstacle is F and RelDist is not Zero then Direction is L and Speed is Stop", engine));

        mamdani->addRule(Rule::parse("if Path is L and Distance is Clo and Obstacle is R and RelAngle is R and RelDist is Large then Direction is L and Speed is Stop", engine));
        mamdani->addRule(Rule::parse("if Path is L and Distance is Med and Obstacle is R and RelAngle is R and RelDist is Large then Direction is L and Speed is Go", engine));

        mamdani->addRule(Rule::parse("if Path is L and Distance is Clo and Obstacle is OKR and RelAngle is F and RelDist is not Zero then Direction is F and Speed is Slow", engine));
        mamdani->addRule(Rule::parse("if Path is L and Distance is Med and Obstacle is OKR and RelAngle is F and RelDist is not Zero then Direction is F and Speed is Slow", engine));

        mamdani->addRule(Rule::parse("if Path is L and Distance is Clo and Obstacle is OKR and RelAngle is R and RelDist is not Zero then Direction is F and Speed is Slow", engine));
        mamdani->addRule(Rule::parse("if Path is L and Distance is Med and Obstacle is OKR and RelAngle is R and RelDist is not Zero then Direction is VR and Speed is Slow", engine));

        mamdani->addRule(Rule::parse("if Path is L and Distance is Clo and Obstacle is R and RelAngle is L and RelDist is Large then Direction is L and Speed is Stop", engine));
        mamdani->addRule(Rule::parse("if Path is L and Distance is Med and Obstacle is R and RelAngle is L and RelDist is Large then Direction is L and Speed is Slow", engine));
        mamdani->addRule(Rule::parse("if Path is L and Distance is Clo and Obstacle is R and RelAngle is F and RelDist is Large then Direction is L and Speed is Stop", engine));
        mamdani->addRule(Rule::parse("if Path is L and Distance is Med and Obstacle is R and RelAngle is F and RelDist is Large then Direction is L and Speed is Slow", engine));



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
		_vec RelativeVector;
    double DeltaX = goal.x - robot.x;
    double DeltaY = goal.y - robot.y;
    RelativeVector.angle = (double)atan2(DeltaY,DeltaX);
    RelativeVector.length = (double)sqrt(pow(DeltaX,2)+pow(DeltaY,2));
    this->relativeVector = RelativeVector;
}

void FuzzyController::setPath(double path_d)
{
    path->setValue(path_d);
}

_vec FuzzyController::getRelativeVectorToGoal()
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

double FuzzyController::getAngleToClosestObstacle()
{
    return (double)obstacle->getValue();
}
double FuzzyController::getDistanceToClosestObstacle()
{
    return (double)distance->getValue();
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
bool FuzzyController::is_at_goal()
{
    if ( rel_dist->getValue() < 0.1 )
        return true;
    else
        return false;
}
