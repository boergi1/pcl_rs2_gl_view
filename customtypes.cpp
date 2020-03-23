#include "customtypes.h"



int roundToInt(double val)
{
    val = val + 0.5 - (val<0);
    return (int)val;
}
