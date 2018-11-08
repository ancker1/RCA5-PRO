#ifndef HELPSTRUCTS_H
#define HELPSTRUCTS_H

struct _vec
{
    double length;
    double angle;
};

struct coordinate
{
    double x;
    double y;
};

#define GO_TO_GOAL  0
#define FOLLOW_WALL 1

#define PATH_R      0.5
#define PATH_L      1.5
#define PATH_S      2.5
#define PATH_NON    3.5


#endif // HELPSTRUCTS_H
