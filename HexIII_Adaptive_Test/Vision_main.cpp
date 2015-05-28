#include <iostream>
#include <cstring>
#include <time.h>
#include <sys/stat.h>

#include "Platform.h"
#include "Vision_Client.h"
#include "Kinect_Test.h"

using namespace std;
using namespace Aris::Core;


#if ARIS_PLATFORM_ == _PLATFORM_LINUX_
#include <unistd.h>
#endif

CONN *pVisualSystem;
Kinect visionsensor;

int main()
{
    time_t t = time(NULL);
    tm* local = new tm;
    char buf[26] = {0};
    localtime_r(&t,local);
    strftime(buf, 64, "%Y-%m-%d %H-%M-%S", local);
    mkdir(buf,S_IRWXU | S_IRWXG);
    chdir(buf);

    visionsensor.viewcloud();
    visionsensor.start();

#if ARIS_PLATFORM_==_PLATFORM_LINUX_
    char RemoteIp[] = "127.0.0.1";
#endif


//    while(1)
//    {
//        int i = 0;
//        visionsensor.capture(&i);
//    }


        CONN VisualSystem;
        pVisualSystem = &VisualSystem;


        /*注册所有的消息函数*/
        Aris::Core::RegisterMsgCallback(VisualSystemDataNeeded, OnVisualSystemDataNeeded);
        Aris::Core::RegisterMsgCallback(VisualSystemLost, OnVisualSystemLost);

        /*设置所有CONN类型的回调函数*/
        VisualSystem.SetCallBackOnReceivedData(OnConnDataReceived);
        VisualSystem.SetCallBackOnLoseConnection(OnConnectionLost);

        /*连接服务器*/
        VisualSystem.Connect(RemoteIp, "5691");

        /*开始消息循环*/
        Aris::Core::RunMsgLoop();
}


