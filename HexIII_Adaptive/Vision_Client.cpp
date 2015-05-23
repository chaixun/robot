#include <iostream>
#include <cstring>

#include "Vision_Client.h"

const string GaitLog = "Gait.txt";

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
    fstream GaitFile;
    static double Foot_Height[5][6] = {0};
    static int i = -1;
    GaitFile.open(GaitLog, ios::out|ios::app);

    visionsensor.capture(&i);

    while(1)
    {
        if(i>9)
        {
            Aris::Core::MSG data;
            data.SetLength(6 * sizeof(double));
            memset(data.GetDataAddress(), 0, data.GetLength());
            pVisualSystem->SendData(data);
            break;
        }
        else
        {
            if(Kinect::IsCaptureEnd == true)
            {
                Foot_Height[(4+i)%5][0] = Kinect::CurrentHeight[1];
                Foot_Height[(2+i)%5][1] = Kinect::CurrentHeight[0];
                Foot_Height[(0+i)%5][2] = Kinect::CurrentHeight[1];
                Foot_Height[(4+i)%5][3] = Kinect::CurrentHeight[3];
                Foot_Height[(2+i)%5][4] = Kinect::CurrentHeight[2];
                Foot_Height[(0+i)%5][5] = Kinect::CurrentHeight[3];

                cout<<"CURRENT "<<endl<<Kinect::CurrentHeight[0]<<endl<<Kinect::CurrentHeight[0]<<endl
                        <<Kinect::CurrentHeight[0]<<endl<<Kinect::CurrentHeight[0]<<endl;

                cout<<"FOOT HEIGHT"<<endl<<Foot_Height[i%5][0]<<endl<<Foot_Height[i%5][1]<<endl<<Foot_Height[i%5][2]
                        <<endl<<Foot_Height[i%5][3]<<endl<<Foot_Height[i%5][4]<<endl<<Foot_Height[i%5][5]<<endl;

                Kinect::IsCaptureEnd = false;
                Aris::Core::MSG data;
                data.SetLength(6 * sizeof(double));
                data.Copy(Foot_Height[i%5], sizeof(Foot_Height[i%5]));
                pVisualSystem->SendData(data);

                GaitFile<<Foot_Height[i%5][0]<<" "<<Foot_Height[i%5][1]<<" "<<Foot_Height[i%5][2]<<
                                                                                                    " "<<Foot_Height[i%5][3]<<" "<<Foot_Height[i%5][4]<<" "<<Foot_Height[i%5][5]<<endl;
                GaitFile.close();
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
