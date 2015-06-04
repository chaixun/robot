#include "Server.h"

CONN ControlSystem, VisionSystem;
double Gait_Calculated_From_Map[GAIT_ADAPTIVEWALK_LEN][GAIT_WIDTH];

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
    case 14:
        data.SetMsgID(BEGINDISCOVER);
        cs.NRT_PostMsg(data);
        break;
    case 15:
        data.SetMsgID(ENDDISCOVER);
        cs.NRT_PostMsg(data);
        break;
    case 16:
        CGait::IsWalkAdaptiveRegistered=true;
        Aris::Core::PostMsg(Aris::Core::MSG(VS_Capture));
        break;
    case 17:
        CGait::IsWalkAvoidRegistered = true;
        Aris::Core::PostMsg(Aris::Core::MSG(VS_Capture));
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
    cout<<"receive data from visual system"<<endl;

    if(data.GetLength() == 4)
    {
        int controlcmd;
        memcpy(&controlcmd,data.GetDataAddress(),data.GetLength());

        switch (controlcmd)
        {
        case 1:
        {
            Aris::Core::MSG controldata;
            controldata.SetMsgID(FORWARD);
            cout<<"Send FORWARD Message to CS"<<endl;
            cs.NRT_PostMsg(controldata);
        }
            break;
        case 2:
        {
            Aris::Core::MSG controldata;
            controldata.SetMsgID(BACKWARD);
            cout<<"Send BACKWARD Message to CS"<<endl;
            cs.NRT_PostMsg(controldata);
        }
            break;
        case 3:
        {
            Aris::Core::MSG controldata;
            controldata.SetMsgID(TURNLEFT);
            cout<<"Send TURNLEFT Message to CS"<<endl;
            cs.NRT_PostMsg(controldata);
        }
            break;
        case 4:
        {
            Aris::Core::MSG controldata;
            controldata.SetMsgID(TURNRIGHT);
            cout<<"Send TURNRIGHT Message to CS"<<endl;
            cs.NRT_PostMsg(controldata);
        }
            break;
        default:
            break;
        }
    }
    else
    {
        double *map = new double [data.GetLength()/sizeof(double)];
        memcpy(map, data.GetDataAddress(), data.GetLength()  );

        static double currentH[6],nextH[6];

        memcpy(nextH,map,sizeof(nextH));

        if(currentH[0] == 0&&currentH[1] == 0&&currentH[2] == 0&&currentH[3] == 0&&currentH[4] == 0&&currentH[5] == 0&&
                nextH[0] == 0&&nextH[1] == 0&&nextH[2] == 0&&nextH[3] == 0&&nextH[4] == 0&&nextH[5] == 0)
        {
            delete[] map;
            CGait::IsWalkAdaptiveRegistered = false;

            Aris::Core::MSG controldata;
            controldata.SetMsgID(ENDDISCOVER);
            cout<<"Send ENDDISCOVER Message to CS"<<endl;
            cs.NRT_PostMsg(controldata);
        }
        else
        {
            HexIII.RobotStepUp(currentH,nextH,*Gait_Calculated_From_Map);

            memcpy(currentH,nextH,sizeof(nextH));

            delete[] map;

            for(int j=0;j<GAIT_ADAPTIVEWALK_LEN;j++)
            {
                for(int i=0;i<GAIT_WIDTH;i++)
                {
                    CGait::GaitAdaptiveWalk[j][i]=-(int)(Gait_Calculated_From_Map[j][i]);
                }
            }

            Aris::Core::MSG controldata;
            controldata.SetMsgID(WALKADAPTIVE);
            cs.NRT_PostMsg(controldata);
        }
    }
    return 0;
}

int On_VS_ConnectionLost(Aris::Core::CONN *pConn)
{
    CGait::IsWalkAdaptiveRegistered = false;
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
    if(CGait::IsWalkAdaptiveRegistered == true)
    {
        Vision_Msg vis_msg = Walk_Adaptive;
        Aris::Core::MSG data;
        data.SetLength(sizeof(vis_msg));
        data.Copy(&vis_msg, sizeof(vis_msg));
        VisionSystem.SendData(data);
    }
    else if(CGait::IsWalkAvoidRegistered == true)
    {
        Vision_Msg vis_msg = Walk_Avoid;
        Aris::Core::MSG data;
        data.SetLength(sizeof(vis_msg));
        data.Copy(&vis_msg, sizeof(vis_msg));
        VisionSystem.SendData(data);
    }
    return 0;
}

int On_VS_Lost(Aris::Core::MSG &msg)
{
    cout << "Visual system connection lost" << endl;
    VisionSystem.StartServer("5691");
    return 0;
}

