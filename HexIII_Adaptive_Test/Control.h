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

static CGait gait;
static EGAIT gaitcmd[AXIS_NUMBER];

void* Thread1(void *);

int initFun(CSysInitParameters& param);

int tg(CMachineData& machineData, RT_MSG& msg);

#endif // CONTROL_H
