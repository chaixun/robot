#include <iostream>
#include <cstring>

#include "Vision_Client.h"

using namespace std;
using namespace Aris::Core;

int OnVisualSystemLost(Aris::Core::MSG &msg)
{
    cout << "Vision system connection lost" << endl;

    Aris::Core::StopMsgLoop();

    return 0;
}

int OnVisualSystemDataNeeded(Aris::Core::MSG &msg)
{
    static int i = 0;

    cout<<"Capture "<<i<<" begin!!!"<<endl;

    visionsensor.capture(&i);

    while(1)
    {
        if(i>40)
        {
            Kinect::ControlCommand = NoValidCommand;
            Aris::Core::MSG data;
            data.SetLength(1 * sizeof(int));
            memset(data.GetDataAddress(), 0, data.GetLength());
            data.Copy(&Kinect::ControlCommand, sizeof(i));
            pVisualSystem->SendData(data);
            break;
        }
        else
        {
            if(Kinect::IsCaptureEnd == true)
            {
                Kinect::IsCaptureEnd = false;
                Aris::Core::MSG data;
                data.SetLength(1 * sizeof(int));
                memset(data.GetDataAddress(), 0, data.GetLength());
                data.Copy(&Kinect::ControlCommand, sizeof(i));
                pVisualSystem->SendData(data);
                break;
            }
        }
    }
    return 0;
}

int OnConnDataReceived(Aris::Core::CONN *pConn, Aris::Core::MSG &data)
{
    Aris::Core::PostMsg(Aris::Core::MSG(VisualSystemDataNeeded));

    return 0;
}

int OnConnectionLost(Aris::Core::CONN *pConn)
{
    PostMsg(Aris::Core::MSG(VisualSystemLost));

    return 0;
}
