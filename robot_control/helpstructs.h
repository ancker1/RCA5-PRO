#ifndef HELPSTRUCTS_H
#define HELPSTRUCTS_H

namespace poolS
{
    struct vector {
        vector(lengthIn,angleIn)
        {
            length = lengthIn;
            angle = angleIn;
        }
        float length;
        float angle;
    };

    struct coordinate {
        float x;
        float y;
    };
}

poolS::vector vec(1.5,3);
float x = vec.angle;


#endif // HELPSTRUCTS_H
