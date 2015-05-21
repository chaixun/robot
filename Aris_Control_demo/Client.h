#ifndef CLIENT_H_
#define CLIENT_H_

#include <iostream>
#include <cstring>
#include <Aris_Socket.h>
#include <Aris_Message.h>

using namespace std;
using namespace Aris::Core;

extern Aris::Core::CONN ControlSysClient;
 
enum ClienMsg
{
	ControlCommandNeeded,
	SystemLost,
};

//MSG call back
int OnControlCommandNeeded(Aris::Core::MSG &msg);
int OnSystemLost(Aris::Core::MSG &msg);

//CONN call back
int OnConnDataReceived(Aris::Core::CONN *pConn, Aris::Core::MSG &data);
int OnConnectLost(Aris::Core::CONN *pConn);

#endif //CLIENT_H_





 
