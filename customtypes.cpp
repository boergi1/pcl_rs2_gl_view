#include "customtypes.h"

std::string rs2PositionToString(Rs2Position_t pos)
{
    std::string tmp;
    switch (pos) {
    case Rs2Position_t::CENTRAL: tmp = "Central"; break;
    case Rs2Position_t::FRONT: tmp = "Front"; break;
    case Rs2Position_t::REAR: tmp = "Rear"; break;
    case Rs2Position_t::OTHER: tmp = "Other"; break;
    }
    return tmp;
}

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
