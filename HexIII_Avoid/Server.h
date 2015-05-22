#ifndef SERVER_H_
#define SERVER_H_

#include <iostream>
#include <cstring>
#include <Aris_Message.h>
#include <Aris_Socket.h>
#include "Gait.h"
 
using namespace std;
using namespace Aris::Core;

extern Aris::RT_CONTROL::ACTUATION cs;

enum Client_Msg
{
	CS_Connected=0,
	CS_CMD_Received=1,
	CS_Lost=2,

    VS_Connected=3,
    VS_Capture=4,
    VS_Lost=5,

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
