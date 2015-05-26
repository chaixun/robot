#ifndef ROBOT_VISION_H
#define ROBOT_VISION_H

#include "Hexapod_Robot.h"
#include "Aris_DynKer.h"
#include "Aris_Plan.h"

using namespace Aris::Plan;
using namespace Hexapod_Robot;
using namespace Aris::DynKer;

class Vision_Robot:public Hexapod_Robot::ROBOT
{
public:
    Vision_Robot();
    ~Vision_Robot();
    int MotionPlanWithKinect(double* currentH, double *nextH, double *data);
};

#endif // ROBOT_VISION_H

