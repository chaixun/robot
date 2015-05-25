#ifndef SERVER_H_
#define SERVER_H_

#include <iostream>
#include <cstring>
#include <Aris_Message.h>
#include <Aris_Socket.h>
#include "Gait.h"
#include "Robot_Vision.h"
 
using namespace std;
using namespace Aris::Core;

extern Aris::RT_CONTROL::ACTUATION cs;
extern Vision_Robot  HexIII;

enum Client_Msg
{
	CS_Connected=0,
	CS_CMD_Received=1,
	CS_Lost=2,

    VS_Connected=3,
    VS_Capture=4,
    VS_Lost=5,

};

enum MACHINE_CMD
{
    NOCMD = 1000,
    POWEROFF = 1001,
    STOP = 1002,
    ENABLE = 1003,
    RUNNING = 1004,
    GOHOME_1 = 1005,
    GOHOME_2 = 1006,
    HOME2START_1 = 1007,
    HOME2START_2 = 1008,
    FORWARD = 1009,
    BACKWARD = 1010,
    TURNLEFT = 1011,
    TURNRIGHT = 1012,
    LEGUP = 1013,
    BEGINDISCOVER = 1014,
    ENDDISCOVER = 1015,
    WALKADAPTIVE = 1016,
};


enum RobotCMD_Msg
{
	GetControlCommand=100,
};

//CS
//MSG callback
int On_CS_Connected(Aris::Core::MSG &msg);
int On_CS_CMD_Received(Aris::Core::MSG &msg);
int On_CS_Lost(Aris::Core::MSG &msg);
int OnGetControlCommand(MSG &msg);

int On_VS_Connected(Aris::Core::MSG &msg);
int On_VS_Capture(Aris::Core::MSG &msg);
int On_VS_Lost(Aris::Core::MSG &msg);

// CONN callback
int On_CS_ConnectionReceived(Aris::Core::CONN *pConn, const char* addr,int port);
int On_CS_DataReceived(Aris::Core::CONN *pConn, Aris::Core::MSG &data);
int On_CS_ConnectionLost(Aris::Core::CONN *pConn);

int On_VS_ConnectionReceived(Aris::Core::CONN *pConn, const char* addr,int port);
int On_VS_DataReceived(Aris::Core::CONN *pConn, Aris::Core::MSG &data);
int On_VS_ConnectionLost(Aris::Core::CONN *pConn);

extern Aris::Core::CONN ControlSystem;
extern Aris::Core::CONN VisionSystem;

#endif //SERVER_H_
