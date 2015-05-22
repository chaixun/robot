#include "Server.h"

CONN ControlSystem, VisionSystem;

// CONN call back functions
int On_CS_ConnectionReceived(Aris::Core::CONN *pConn, const char* addr,int port)
{
    Aris::Core::MSG msg;

    msg.SetMsgID(CS_Connected);
    msg.SetLength(sizeof(port));
    msg.Copy(&port,sizeof(port));
    msg.CopyMore(addr,strlen(addr));
    PostMsg(msg);
    return 0;
}
int On_CS_DataReceived(Aris::Core::CONN *pConn, Aris::Core::MSG &data)
{
    int cmd=data.GetMsgID();
    MSG CMD=CS_CMD_Received;
    CMD.SetLength(sizeof(int));
    CMD.Copy(&cmd,sizeof(int));
    cout<<"received CMD is "<<cmd<<endl;
    PostMsg(CMD);
    return 0;
}
int On_CS_ConnectionLost(Aris::Core::CONN *pConn)
{
    PostMsg(Aris::Core::MSG(CS_Lost));
    return 0;
}

//MSG call back functions
int On_CS_Connected(Aris::Core::MSG &msg)
{
    cout<<"Received Connection from Control System:"<<endl;
    cout<<"   Remote IP is: "<<msg.GetDataAddress()+sizeof(int)<<endl;
    cout<<"   Port is     : "<<*((int*)msg.GetDataAddress()) << endl << endl;

    Aris::Core::MSG data(0,0);
    ControlSystem.SendData(data);
    return 0;
}

int On_CS_CMD_Received(Aris::Core::MSG &msg)
{

    MSG Command(msg);
    Command.SetMsgID(GetControlCommand);
    PostMsg(Command);
    MSG data(0,0);
    ControlSystem.SendData(data);
    return 0;
}

int On_CS_Lost(Aris::Core::MSG &msg)
{
    cout << "Control system connection lost" << endl;
    ControlSystem.StartServer("5690");
    return 0;
}

int On_VS_ConnectionReceived(Aris::Core::CONN *pConn, const char* addr,int port)
{
    Aris::Core::MSG msg;
    msg.SetMsgID(VS_Connected);
    msg.SetLength(sizeof(port));
    msg.Copy(&port,sizeof(port));
    msg.CopyMore(addr,strlen(addr));
    PostMsg(msg);
    return 0;
}

int On_VS_DataReceived(Aris::Core::CONN *pConn, Aris::Core::MSG &data)
{

    cout<<"Vision Control Singnal Recieved"<<endl;
    int controlcmd;
    memcpy(&controlcmd,data.GetDataAddress(),data.GetLength());

    Aris::Core::MSG command;
    switch (controlcmd)
    {
    case 0:
        CGait::IsWalkAvoidRegistered = false;
        break;
    case 1:
    {
        command.SetMsgID(FORWARD);
        cout<<"Send GAIT_MOVE Message to CS"<<endl;
        cs.NRT_PostMsg(command);
    }
        break;
    case 2:
    {
        command.SetMsgID(BACKWARD);
        cout<<"Send GAIT_MOVE_BACK Message to CS"<<endl;
        cs.NRT_PostMsg(command);
    }
        break;
    case 3:
    {
        command.SetMsgID(TURNLEFT);
        cout<<"Send GAIT_TURN_LEFT Message to CS"<<endl;
        cs.NRT_PostMsg(command);
    }
        break;
    case 4:
    {
        command.SetMsgID(TURNRIGHT);
        cout<<"Send GAIT_TURN_RIGHT Message to CS"<<endl;
        cs.NRT_PostMsg(command);
    }
        break;
    default:
        break;
    }
    return 0;
}

int On_VS_ConnectionLost(Aris::Core::CONN *pConn)
{
    CGait::IsWalkAvoidRegistered = false;
    PostMsg(Aris::Core::MSG(VS_Lost));
    return 0;
}

int On_VS_Connected(Aris::Core::MSG &msg)
{
    cout<<"Received Connection from Vision System:"<<endl;
    cout<<"   Remote IP is: "<<msg.GetDataAddress()+sizeof(int)<<endl;
    cout<<"   Port is     : "<<*((int*)msg.GetDataAddress()) << endl << endl;

    return 0;

}

int On_VS_Capture(Aris::Core::MSG &msg)
{
    MSG data(0,0);
    VisionSystem.SendData(data);
    return 0;
}

int On_VS_Lost(Aris::Core::MSG &msg)
{
    cout << "Visual system connection lost" << endl;
    VisionSystem.StartServer("5691");
    return 0;
}

