#include "Control.h"

ACTUATION cs;
CSysInitParameters initParam;

using namespace Aris::Core;
using namespace Aris::RT_CONTROL;

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
    Aris::Core::RegisterMsgCallback(CS_Connected,On_CS_Connected);
    Aris::Core::RegisterMsgCallback(CS_CMD_Received,On_CS_CMD_Received);
    Aris::Core::RegisterMsgCallback(CS_Lost,On_CS_Lost);
    Aris::Core::RegisterMsgCallback(GetControlCommand,OnGetControlCommand);


    ControlSystem.SetCallBackOnReceivedConnection(On_CS_ConnectionReceived);
    ControlSystem.SetCallBackOnReceivedData(On_CS_DataReceived);
    ControlSystem.SetCallBackOnLoseConnection(On_CS_ConnectionLost);
    ControlSystem.StartServer("5690");

    //VisualSys.SetCallBackOnReceivedConnection(OnConnectReceived_VS);
    //VisualSys.SetCallBackOnReceivedData(OnSysDataReceived_VS);
    //VisualSys.SetCallBackOnLoseConnection(OnConnectionLost_VS);
    //VisualSys.StartServer("5691");

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

    while(!cs.IsSysStopped())
    {
        sleep(1);
    }

    return 0;
}


