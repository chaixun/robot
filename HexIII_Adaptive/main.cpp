#include "Control.h"
#include "Hexapod_Robot.h"

using namespace std;
using namespace Aris::Core;
using namespace Aris::RT_CONTROL;
using namespace Hexapod_Robot;

ACTUATION cs;
CSysInitParameters initParam;
Hexapod_Robot::ROBOT HexIII;

static int HEXBOT_HOME_OFFSETS_RESOLVER[18] =
{
    -15849882,	 -16354509,	 -16354509,
    -15849882,	 -16354509,	 -16354509,
    -15849882,	 -16354509,	 -16354509,
    -16354509,	 -15849882,	 -16354509,
    -15849882,	 -16354509,	 -16354509,
    -16354509,	 -16354509,  -15849882
};

int main()
{
    HexIII.LoadXML("HexapodIII.xml");

    Aris::Core::RegisterMsgCallback(CS_Connected,On_CS_Connected);
    Aris::Core::RegisterMsgCallback(CS_CMD_Received,On_CS_CMD_Received);
    Aris::Core::RegisterMsgCallback(CS_Lost,On_CS_Lost);
    Aris::Core::RegisterMsgCallback(GetControlCommand,OnGetControlCommand);

    Aris::Core::RegisterMsgCallback(VS_Connected,On_VS_Connected);
    Aris::Core::RegisterMsgCallback(VS_Capture,On_VS_Capture);
    Aris::Core::RegisterMsgCallback(VS_Lost,On_VS_Lost);


    ControlSystem.SetCallBackOnReceivedConnection(On_CS_ConnectionReceived);
    ControlSystem.SetCallBackOnReceivedData(On_CS_DataReceived);
    ControlSystem.SetCallBackOnLoseConnection(On_CS_ConnectionLost);
    ControlSystem.StartServer("5690");

    VisionSystem.SetCallBackOnReceivedConnection(On_VS_ConnectionReceived);
    VisionSystem.SetCallBackOnReceivedData(On_VS_DataReceived);
    VisionSystem.SetCallBackOnLoseConnection(On_VS_ConnectionLost);
    VisionSystem.StartServer("5691");

    Aris::Core::THREAD T1;
    T1.SetFunction(Thread1);
    T1.Start(0);

    initParam.motorNum=18;
    initParam.homeHighSpeed=300000;
    initParam.homeLowSpeed=40000;
    initParam.homeOffsets=HEXBOT_HOME_OFFSETS_RESOLVER;

    //cs.Load_XML_PrintMessage();
    cs.SetSysInitializer(initFun);
    cs.SetTrajectoryGenerator(tg);
    //cs.SetModeCycVel();
    cs.SysInit(initParam);
    cs.SysInitCommunication();
    cs.SysStart();

    cout<<"Will start"<<endl;

    while(!cs.IsSysStopped())
    {
        sleep(1);
    }

    return 0;
}


