#include "Server.h"

CONN ControlSystem, VisionSystem;
double Gait_StepUp__Map[GAIT_STEPUPANDDOWN_LEN][GAIT_WIDTH];
double Gait_StepDown__Map[GAIT_STEPUPANDDOWN_LEN][GAIT_WIDTH];
double Gait_Move_Map[GAIT_MOVE_MAP_LEN][GAIT_WIDTH];
double Gait_Turn_Map[GAIT_TURN_MAP_LEN][GAIT_WIDTH];

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
    {
        CGait::IsVisionWalk = true;
        Vision_Msg visioncmd = Vision_UpperControl;
        Aris::Core::MSG visionmsg;
        visionmsg.SetMsgID(VS_Capture);
        visionmsg.SetLength(sizeof(visioncmd));
        visionmsg.Copy(&visioncmd, sizeof(visioncmd));
        PostMsg(visionmsg);
    }
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

    switch (data.GetMsgID())
    {
    case 34:
    {
        cout<<"MOVE!"<<endl;
        CGait::IsMove = true;
        double *move_data = new double [data.GetLength()/sizeof(double)];
        memcpy(move_data, data.GetDataAddress(), data.GetLength());
        cout<<move_data[0]<<" "<<move_data[1]<<" "<<move_data[2]<<endl;
        HexIII.RobotMove(move_data, *Gait_Move_Map);
        for(int j=0;j<GAIT_MOVE_MAP_LEN;j++)
        {
            for(int i=0;i<GAIT_WIDTH;i++)
            {
                CGait::GaitMoveMap[j][i]=-(int)(Gait_Move_Map[j][i]);
            }
        }
        Aris::Core::MSG controlcmd;
        controlcmd.SetMsgID(MOVE);
        cout<<"Send MOVE Message to CS"<<endl;
        cs.NRT_PostMsg(controlcmd);
    }
        break;
    case 35:
    {
        cout<<"TURN!"<<endl;
        CGait::IsTurn = true;
        double *turn_data = new double [data.GetLength()/sizeof(double)];
        memcpy(turn_data, data.GetDataAddress(), data.GetLength());
        *turn_data = *turn_data*180/M_PI;
        cout<<*turn_data<<endl;
        HexIII.RobotTurn(turn_data, *Gait_Turn_Map);
        for(int j=0;j<GAIT_TURN_MAP_LEN;j++)
        {
            for(int i=0;i<GAIT_WIDTH;i++)
            {
                CGait::GaitTurnMap[j][i]=-(int)(Gait_Turn_Map[j][i]);
            }
        }
        Aris::Core::MSG controlcmd;
        controlcmd.SetMsgID(TURN);
        cout<<"Send TURN Message to CS"<<endl;
        cs.NRT_PostMsg(controlcmd);
    }
        break;
    case 36:
    {
        cout<<"STEP UP!"<<endl;
        CGait::IsStepUp = true;
        double *map = new double [data.GetLength()/sizeof(double)];
        memcpy(map, data.GetDataAddress(), data.GetLength());

        static double StepUP_currentH[6] = {-1.05, -1.05, -1.05, -1.05, -1.05, -1.05};
        static double StepUP_nextH[6];

        memcpy(StepUP_nextH,map,sizeof(StepUP_nextH));

        if(StepUP_currentH[0] == -0.85&&StepUP_currentH[1] == -0.85&&StepUP_currentH[2] == -0.85
                &&StepUP_currentH[3] == -0.85&&StepUP_currentH[4] == -0.85&&StepUP_currentH[5] == -0.85
                &&StepUP_nextH[0] == -0.85&&StepUP_nextH[1] == -0.85&&StepUP_nextH[2] == -0.85
                &&StepUP_nextH[3] == -0.85&&StepUP_nextH[4] == -0.85&&StepUP_nextH[5] == -0.85)
        {
            cout<<"STEPUP FINISHED"<<endl;
            CGait::IsStepUp = false;
            delete[] map;
            Vision_Msg visioncmd = Vision_UpperControl;
            Aris::Core::MSG visionmsg;
            visionmsg.SetMsgID(VS_Capture);
            visionmsg.SetLength(sizeof(visioncmd));
            visionmsg.Copy(&visioncmd, sizeof(visioncmd));
            PostMsg(visionmsg);
        }
        else
        {
            double StepUpLen = 0.325;

            HexIII.RobotStepUp(&StepUpLen,StepUP_currentH,StepUP_nextH,*Gait_StepUp__Map);

            memcpy(StepUP_currentH,StepUP_nextH,sizeof(StepUP_nextH));

            delete[] map;

            for(int j=0;j<GAIT_STEPUPANDDOWN_LEN;j++)
            {
                for(int i=0;i<GAIT_WIDTH;i++)
                {
                    CGait::GaitStepUpMap[j][i]=-(int)(Gait_StepUp__Map[j][i]);
                }
            }

            Aris::Core::MSG controldata;
            controldata.SetMsgID(STEPUP);
            cs.NRT_PostMsg(controldata);
        }
    }
        break;
    case 37:
    {
        if(data.GetLength() == sizeof(int))
        {
            Vision_Msg visioncmd = Vision_StepDown;
            Aris::Core::MSG visionmsg;
            visionmsg.SetMsgID(VS_Capture);
            visionmsg.SetLength(sizeof(visioncmd));
            visionmsg.Copy(&visioncmd, sizeof(visioncmd));
            PostMsg(visionmsg);
        }
        else
        {
            CGait::IsStepDown = true;
            cout<<"STEP DOWN!"<<endl;
            double *map = new double [data.GetLength()/sizeof(double)];
            memcpy(map, data.GetDataAddress(), data.GetLength());

            static double StepDown_currentH[6] = {-0.85, -0.85, -0.85, -0.85, -0.85, -0.85};
            static double StepDown_nextH[6];

            memcpy(StepDown_nextH,map,sizeof(StepDown_nextH));

            if(StepDown_currentH[0] == -1.05&&StepDown_currentH[1] == -1.05&&StepDown_currentH[2] == -1.05
                    &&StepDown_currentH[3] == -1.05&&StepDown_currentH[4] == -1.05&&StepDown_currentH[5] == -1.05
                    &&StepDown_nextH[0] == -1.05&&StepDown_nextH[1] == -1.05&&StepDown_nextH[2] == -1.05
                    &&StepDown_nextH[3] == -1.05&&StepDown_nextH[4] == -1.05&&StepDown_nextH[5] == -1.05)
            {
                delete[] map;
                CGait::IsStepDown = false;
                cout<<"STEPDOWN FINISHED!"<<endl;
                cout<<"ENDISCOVER!"<<endl;
                CGait::IsEndDiscoverStart = true;
                Aris::Core::MSG controlcmd;
                controlcmd.SetMsgID(ENDDISCOVER);
                cout<<"Send ENDDISCOVER Message to CS"<<endl;
                cs.NRT_PostMsg(controlcmd);
            }
            else
            {
                double StepDownLen = 0.325;

                HexIII.RobotStepDown(&StepDownLen,StepDown_currentH,StepDown_nextH,*Gait_StepDown__Map);

                memcpy(StepDown_currentH,StepDown_nextH,sizeof(StepDown_nextH));

                delete[] map;

                for(int j=0;j<GAIT_STEPUPANDDOWN_LEN;j++)
                {
                    for(int i=0;i<GAIT_WIDTH;i++)
                    {
                        CGait::GaitStepDownMap[j][i]=-(int)(Gait_StepDown__Map[j][i]);
                    }
                }

                Aris::Core::MSG controldata;
                controldata.SetMsgID(STEPDOWN);
                cs.NRT_PostMsg(controldata);
            }
        }
    }
        break;
    case 38:
    {
        if(data.GetLength() == sizeof(int))
        {
            Vision_Msg visioncmd = Vision_StepOver;
            Aris::Core::MSG visionmsg;
            visionmsg.SetMsgID(VS_Capture);
            visionmsg.SetLength(sizeof(visioncmd));
            visionmsg.Copy(&visioncmd, sizeof(visioncmd));
            PostMsg(visionmsg);
        }
        else
        {
            cout<<"STEPOVER!"<<endl;
            CGait::IsStepOver = true;
            double *stepover_data = new double [data.GetLength()/sizeof(double)];
            memcpy(stepover_data, data.GetDataAddress(), data.GetLength());

            int count = int(stepover_data[0]);
            if(count == 5)
            {
                CGait::IsStepOver = false;
                cout<<"STEPOVER FINISHED!"<<endl;
                Vision_Msg visioncmd = Vision_UpperControl;
                Aris::Core::MSG visionmsg;
                visionmsg.SetMsgID(VS_Capture);
                visionmsg.SetLength(sizeof(visioncmd));
                visionmsg.Copy(&visioncmd, sizeof(visioncmd));
                PostMsg(visionmsg);
            }
            else
            {
                double move_data[3];
                move_data[0] = stepover_data[1];
                move_data[1] = stepover_data[2];
                move_data[2] = stepover_data[3];

                HexIII.RobotMove(move_data, *Gait_Move_Map);
                for(int j=0;j<GAIT_MOVE_MAP_LEN;j++)
                {
                    for(int i=0;i<GAIT_WIDTH;i++)
                    {
                        CGait::GaitMoveMap[j][i]=-(int)(Gait_Move_Map[j][i]);
                    }
                }
                Aris::Core::MSG controlcmd;
                controlcmd.SetMsgID(MOVE);
                cout<<"Send MOVE Message to CS"<<endl;
                cs.NRT_PostMsg(controlcmd);
            }
        }
    }
        break;
    case 39:
    {
        cout<<"BEGINDISCOVER!"<<endl;
        CGait::IsBeginDiscoverStart = true;
        Aris::Core::MSG controlcmd;
        controlcmd.SetMsgID(BEGINDISCOVER);
        cout<<"Send BEGINDISCOVER Message to CS"<<endl;
        cs.NRT_PostMsg(controlcmd);
    }
        break;
    default:
        break;
    }
    return 0;
}

int On_VS_ConnectionLost(Aris::Core::CONN *pConn)
{
    CGait::IsVisionWalk = false;
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
    Aris::Core::MSG visionmsg(msg);
    VisionSystem.SendData(visionmsg);
    return 0;
}

int On_VS_Lost(Aris::Core::MSG &msg)
{
    cout << "Visual system connection lost" << endl;
    VisionSystem.StartServer("5691");
    return 0;
}




