#include "customtypes.h"

std::string rs2PositionToString(CameraType_t pos)
{
    std::string tmp;
    switch (pos) {
    case CameraType_t::CENTRAL: tmp = "Central"; break;
    case CameraType_t::FRONT: tmp = "Front"; break;
    case CameraType_t::REAR: tmp = "Rear"; break;
    case CameraType_t::OTHER: tmp = "Other"; break;
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
