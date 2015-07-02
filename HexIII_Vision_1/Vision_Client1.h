#ifndef VISION_CLIENT_H
#define VISION_CLIENT_H

#include <Aris_Socket.h>
#include <Aris_Message.h>

#include "Kinect_Test1.h"

using namespace Aris::Core;

extern Kinect visionsensor;
extern CONN *pVisualSystem;

enum ClientMessage
{
    VisualSystemDataNeeded,
    VisualSystemLost,
};


int OnVisualSystemLost(Aris::Core::MSG &msg);
int OnVisualSystemDataNeeded(Aris::Core::MSG &msg);

int OnConnDataReceived(Aris::Core::CONN *pConn, Aris::Core::MSG &data);
int OnConnectionLost(Aris::Core::CONN *pConn);


#endif // VISION_CLENT_H
