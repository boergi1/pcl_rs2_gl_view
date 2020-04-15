#include "customtypes.h"



int roundToInt(double val)
{
    val = val + 0.5 - (val<0);
    return static_cast<int>(val);
}

bool areSameD(double a, double b)
{
    return fabs(a - b) < std::numeric_limits<double>::epsilon();
}

bool areSameF(float a, float b)
{
    return fabs(a - b) < std::numeric_limits<float>::epsilon();
}
