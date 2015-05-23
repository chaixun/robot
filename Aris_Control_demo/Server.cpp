#include "Server.h"

CONN ControlSystem, VisualSystem;

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
    cout<<"received CMD is"<<cmd<<endl;
    PostMsg(CMD);
    return 0;
}
int On_CS_ConnectionLost(Aris::Core::CONN *pConn)
{
    PostMsg(Aris::Core::MSG(CS_Lost));
    return 0;
}

int OnGetControlCommand(MSG &msg)
{
    int CommandID;
    msg.Paste(&CommandID,sizeof(int));
    Aris::Core::MSG data;

    switch(CommandID)
    {
    case 1:
        data.SetMsgID(POWEROFF);
        cs.NRT_PostMsg(data);
        break;
    case 2:
        data.SetMsgID(STOP);
        cs.NRT_PostMsg(data);
        break;
    case 3:
        data.SetMsgID(ENABLE);
        cs.NRT_PostMsg(data);
        break;
    case 4:
        data.SetMsgID(RUNNING);
        cs.NRT_PostMsg(data);
        break;
    case 5:
        data.SetMsgID(GOHOME_1);
        cs.NRT_PostMsg(data);
        break;
    case 6:
        data.SetMsgID(GOHOME_2);
        cs.NRT_PostMsg(data);
        break;
    case 7:
        data.SetMsgID(HOME2START_1);
        cs.NRT_PostMsg(data);
        break;
    case 8:
        data.SetMsgID(HOME2START_2);
        cs.NRT_PostMsg(data);
        break;
    case 9:
        data.SetMsgID(FORWARD);
        cs.NRT_PostMsg(data);
        break;
    case 10:
        data.SetMsgID(BACKWARD);
        cs.NRT_PostMsg(data);
        break;
    case 11:
        data.SetMsgID(TURNLEFT);
        cs.NRT_PostMsg(data);
        break;
    case 12:
        data.SetMsgID(TURNRIGHT);
        cs.NRT_PostMsg(data);
        break;
    case 13:
        data.SetMsgID(LEGUP);
        cs.NRT_PostMsg(data);
        break;
    default:
        cout<<"Do Not Get Validate CMD"<<endl;
        break;
    }
    return CommandID;
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


