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
        Aris::Core::PostMsg(Aris::Core::MSG(VS_Capture));
        CGait::IsAutoCalibrateRegistered=true;
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
    cout<<"Calibration Singnal Recieved"<<endl;
    int num;
    memcpy(&num,data.GetDataAddress(),data.GetLength());
    //stop num >= 54
    if (num >= 62 )
    {
        cout<<"Calibration finished"<<endl;
        CGait::IsAutoCalibrateRegistered = false;
    }
    else
    {
        ifstream inf;
        stringstream filenum;
        string filename;
        filenum<<(num-1);
        filename = "./calibration_txt/"+filenum.str() + "F.txt";
        inf.open(filename);
        int Line_Num;
        double temp;
        for(int i=0;i<GAIT_AUTOCALIB_LEN/2;i++)
        {
            inf>>Line_Num;
            for(int j=0;j<GAIT_WIDTH;j++)
            {
                inf>>temp;
                CGait::GaitAutoCalib[i][j]=-(int)temp;
            }
        }

        inf.close();
        filenum.str("");
        filename.clear();

        filenum<<num;
        filename = "./calibration_txt/"+filenum.str()+".txt";
        inf.open(filename);
        for(int i=GAIT_AUTOCALIB_LEN/2;i<GAIT_AUTOCALIB_LEN;i++)
        {
            inf>>Line_Num;
            for(int j=0;j<GAIT_WIDTH;j++)
            {
                inf>>temp;
                CGait::GaitAutoCalib[i][j]=-(int)temp;
            }
        }
        inf.close();

        cout<<"Calibration "<<num<<" begin"<<endl;
        Aris::Core::MSG data;
        data.SetMsgID(AUTO_CALIBRATION);
        cout<<"Send Message to CS"<<endl;
        cs.NRT_PostMsg(data);
    }
    return 0;
}

int On_VS_ConnectionLost(Aris::Core::CONN *pConn)
{
    CGait::IsAutoCalibrateRegistered = false;
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

