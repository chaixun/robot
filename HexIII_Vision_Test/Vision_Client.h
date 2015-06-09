#ifndef VISION_CLIENT_H
#define VISION_CLIENT_H

#include <Aris_Socket.h>
#include <Aris_Message.h>
#include "math.h"

#include "Kinect_Test.h"

using namespace Aris::Core;

extern Kinect visionsensor;
extern CONN *pVisualSystem;

enum ClientMessage
{
    VisualSystemDataNeeded,
    VisualSystemLost,
};

enum ControlCommand
{
    NeedUpperControl = 30,
    NeedStepUp = 31,
    NeedStepDown = 32,
    NeedStepOver= 33,


    Move = 34,
    Turn = 35,
    StepUp = 36,
    StepDown = 37,
    StepOver = 38,
    BeginDiscover = 39,
    EndDiscover = 40,
};

int OnVisualSystemLost(Aris::Core::MSG &msg);
int OnVisualSystemDataNeeded(Aris::Core::MSG &msg);

int OnConnDataReceived(Aris::Core::CONN *pConn, Aris::Core::MSG &data);
int OnConnectionLost(Aris::Core::CONN *pConn);

int OnUpperControl(Aris::Core::MSG &msg);
int OnStepUp(Aris::Core::MSG &msg);
int OnStepDown(Aris::Core::MSG &msg);
int OnStepOver(Aris::Core::MSG &msg);


#endif // VISION_CLENT_H
