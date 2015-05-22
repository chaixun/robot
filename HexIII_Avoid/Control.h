#ifndef CONTROL_H
#define CONTROL_H

#include <iostream>
#include <Aris_Message.h>
#include <Aris_Socket.h>
#include <Aris_Core.h>

#include "Gait.h"
#include "Server.h"

using namespace std;
using namespace Aris::RT_CONTROL;
using namespace Aris::Core;

extern ACTUATION cs;

enum MACHINE_CMD
{
    NOCMD=1000,
    POWEROFF=1001,
    STOP=1002,
    ENABLE=1003,
    RUNNING=1004,
    GOHOME_1=1005,
    GOHOME_2=1006,
    HOME2START_1=1007,
    HOME2START_2=1008,
    FORWARD=1009,
    BACKWARD=1010,
    FAST_FORWARD=1011,
    FAST_BACKWARD=1012,
    LEGUP=1013,
    TURNLEFT=1014,
    TURNRIGHT=1015,
    WALKAVOID=1016,
};

static CGait gait;
static EGAIT gaitcmd[AXIS_NUMBER];

void* Thread1(void *);

int initFun(CSysInitParameters& param);

int tg(CMachineData& machineData, RT_MSG& msg);

int OnGetControlCommand(MSG &msg);

#endif // CONTROL_H
