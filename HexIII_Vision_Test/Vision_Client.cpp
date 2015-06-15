#include <iostream>
#include <cstring>

#include "Vision_Client.h"

const string GaitLog_StepUp = "Gait_StepUp.txt";
const string GaitLog_StepDown = "Gait_StepDown.txt";

using namespace std;
using namespace Aris::Core;

static int i = -1;

int OnVisualSystemLost(Aris::Core::MSG &msg)
{
    cout << "Vision system connection lost" << endl;

    Aris::Core::StopMsgLoop();

    return 0;
}

int OnVisualSystemDataNeeded(Aris::Core::MSG &msg)
{
    cout<<"Host Needs Vision Servo "<<endl;

    int vis_msg;
    msg.Paste(&vis_msg,sizeof(int));

    switch (vis_msg)
    {
    case 60:
    {
        cout<<"NeedUpperControl"<<endl;
        Aris::Core::MSG msgformcontrol;
        msgformcontrol.SetMsgID(NeedUpperControl);
        Aris::Core::PostMsg(msgformcontrol);
    }
        break;
    case 61:
    {
        cout<<"NeedStepUp"<<endl;
        Aris::Core::MSG msgformcontrol;
        msgformcontrol.SetMsgID(NeedStepUp);
        Aris::Core::PostMsg(msgformcontrol);
    }
        break;
    case 62:
    {
        cout<<"NeedStepDown"<<endl;
        Aris::Core::MSG msgformcontrol;
        msgformcontrol.SetMsgID(NeedStepDown);
        Aris::Core::PostMsg(msgformcontrol);
    }
        break;
    case 63:
    {
        cout<<"NeedStepOver"<<endl;
        Aris::Core::MSG msgformcontrol;
        msgformcontrol.SetMsgID(NeedStepOver);
        Aris::Core::PostMsg(msgformcontrol);
    }
        break;
    default:
        break;
    }

    return 0;
}


int OnConnDataReceived(Aris::Core::CONN *pConn, Aris::Core::MSG &data)
{
    Aris::Core::MSG msg(data);
    msg.SetMsgID(VisualSystemDataNeeded);
    Aris::Core::PostMsg(msg);

    return 0;
}

int OnConnectionLost(Aris::Core::CONN *pConn)
{
    PostMsg(Aris::Core::MSG(VisualSystemLost));

    return 0;
}

int OnUpperControl(Aris::Core::MSG &msg)
{
    cout<<"Upper Control Begin"<<endl;
    visionsensor.capture(&i);

    while(1)
    {
        if(Kinect::IsCaptureEnd == true)
        {
            Kinect::IsCaptureEnd = false;

            cout<<Kinect::Terrain<<endl;

            if(Kinect::Terrain != FlatTerrain)
            {
                /* Turn */
                if(abs(Kinect::leftedge_z - Kinect::rightedge_z) > 2)
                {
                    /*max turn*/
                    cout<<"TURN"<<endl;
                    cout<<"LEFT EDGE Z: "<<Kinect::leftedge_z<<endl;
                    cout<<"RIGHT EDGE Z: "<<Kinect::rightedge_z<<endl;
                    double turn_ang = atan2((Kinect::rightedge_z - Kinect::leftedge_z), 24);
                    cout<<turn_ang<<endl;
                    if(abs(turn_ang) > 35*M_PI/180)
                    {
                        turn_ang = turn_ang > 0? 35*M_PI/180 : -35*M_PI/180;
                    }
                    cout<<"NEEDS TURN: "<<turn_ang*180/M_PI<<endl;
                    Aris::Core::MSG turn_msg;
                    turn_msg.SetMsgID(Turn);
                    turn_msg.SetLength(sizeof(double));
                    turn_msg.Copy(&turn_ang,sizeof(turn_ang));
                    pVisualSystem->SendData(turn_msg);
                }
                else
                {
                    /*Move Closer*/
                    cout<<"MOVE FORWARD AND BACKWARD"<<endl;
                    cout<<"LEFT EDGE Z: "<<Kinect::leftedge_z<<endl;
                    cout<<"RIGHT EDGE Z: "<<Kinect::rightedge_z<<endl;
                    if((Kinect::leftedge_z < 30) || (Kinect::leftedge_z > 32))
                    {
                        /*max walk*/
                        double movez_data[3] = {0, 0, 0};
                        movez_data[2] = (Kinect::leftedge_z - 31)*0.025;
                        if(Kinect::leftedge_z < 30)
                        {
                            movez_data[2] = movez_data[2] < -0.325? -0.325 : movez_data[2];
                        }
                        else
                        {
                            movez_data[2] = movez_data[2] > 0.325? 0.325 : movez_data[2];
                        }
                        Aris::Core::MSG movez_msg;
                        movez_msg.SetMsgID(Move);
                        movez_msg.SetLength(3*sizeof(double));
                        movez_msg.Copy(movez_data, sizeof(movez_data));
                        pVisualSystem->SendData(movez_msg);
                        cout<<"MOVE ALONG Z "<<movez_data[2]<<endl;
                    }
                    else
                    {
                        /*Move Middle*/
                        cout<<"MOVE LFET AND RIGHT"<<endl;
                        cout<<"LEFT_X: "<<Kinect::leftedge_x<<endl;
                        cout<<"RIGHT_X: "<<Kinect::rightedge_x<<endl;
                        if((Kinect::rightedge_x !=0&&Kinect::rightedge_x > 39)
                                ||(Kinect::leftedge_x!=0&&Kinect::leftedge_x < 82))
                        {
                            /*max walk*/
                            if(Kinect::rightedge_x < 39)
                            {
                                double movexr_data[3] = {0, 0, 0};
                                movexr_data[0] = (Kinect::leftedge_x - 82)*0.025;
                                movexr_data[0] = movexr_data[0] < -0.07?-0.07 : movexr_data[0];
                                Aris::Core::MSG movexr_msg;
                                movexr_msg.SetMsgID(Move);
                                movexr_msg.SetLength(3*sizeof(double));
                                movexr_msg.Copy(movexr_data,sizeof(movexr_data));
                                pVisualSystem->SendData(movexr_msg);
                                cout<<"MOVE RIGHT: "<<movexr_data[0]<<endl;
                            }
                            else
                            {
                                double movexl_data[3] = {0, 0, 0};
                                movexl_data[0] = (Kinect::rightedge_x - 39)*0.025;
                                movexl_data[0] = movexl_data[0] > 0.07 ? 0.07 : movexl_data[0];
                                Aris::Core::MSG movexl_msg;
                                movexl_msg.SetMsgID(Move);
                                movexl_msg.SetLength(3*sizeof(double));
                                movexl_msg.Copy(movexl_data,sizeof(movexl_data));
                                pVisualSystem->SendData(movexl_msg);
                                cout<<"MOVE LEFT: "<<movexl_data[0]<<endl;
                            }
                        }
                        else
                        {
                            switch (Kinect::Terrain)
                            {
                            case StepUpTerrain:
                            {
                                cout<<"STEPUP TERRAIN"<<endl;
                                Aris::Core::MSG control_msg;
                                int i = 0;
                                control_msg.SetMsgID(BeginDiscover);
                                control_msg.SetLength(sizeof(i));
                                control_msg.Copy(&i, sizeof(i));
                                pVisualSystem->SendData(control_msg);
                            }
                                break;
                            case StepDownTerrain:
                            {
                                cout<<"STEPDOWN TERRAIN"<<endl;
                                Aris::Core::MSG control_msg;
                                int i = 0;
                                control_msg.SetMsgID(StepDown);
                                control_msg.SetLength(sizeof(i));
                                control_msg.Copy(&i, sizeof(i));
                                pVisualSystem->SendData(control_msg);
                            }
                                break;
                            case DitchTerrain:
                            {
                                cout<<"DITCH TERRAIN"<<endl;
                                Aris::Core::MSG control_msg;
                                int i = 0;
                                control_msg.SetMsgID(StepOver);
                                control_msg.SetLength(sizeof(i));
                                control_msg.Copy(&i, sizeof(i));
                                pVisualSystem->SendData(control_msg);
                            }
                                break;
                            default:
                                break;
                            }
                        }
                    }
                }
            }
            else
            {
                cout<<"FLAT TERRAIN MOVE"<<endl;
                double move_data[3] = {0, 0, 0};
                move_data[2] = 0.325;
                Aris::Core::MSG move_msg;
                move_msg.SetMsgID(Move);
                move_msg.SetLength(3*sizeof(double));
                move_msg.Copy(move_data, sizeof(move_data));
                pVisualSystem->SendData(move_msg);
            }
            break;
        }
    }
    return 0;
}

int OnStepUp(Aris::Core::MSG &msg)
{
    cout<<"Step Up!!!"<<endl;
    fstream GaitFile_Up;
    static double StepUp_Foot_Height[5][6] = {0};

    GaitFile_Up.open(GaitLog_StepUp, ios::out|ios::app);

    visionsensor.capture(&i);

    while(1)
    {
        if(Kinect::IsCaptureEnd == true)
        {
            for(int m = 0; m < 4; m++)
            {
                if(abs(Kinect::CurrentHeight[m] + 0.85) < 0.05 )
                {
                    Kinect::CurrentHeight[m] = -0.85;
                }

                if(Kinect::CurrentHeight[m] > -0.85 )
                {
                    Kinect::CurrentHeight[m] = -0.85;
                }

                if(Kinect::CurrentHeight[m] < -1.05 )
                {
                    Kinect::CurrentHeight[m] = -1.05;
                }
            }
            StepUp_Foot_Height[(4+i)%5][0] = Kinect::CurrentHeight[1];
            StepUp_Foot_Height[(2+i)%5][1] = Kinect::CurrentHeight[0];
            StepUp_Foot_Height[(0+i)%5][2] = Kinect::CurrentHeight[1];
            StepUp_Foot_Height[(4+i)%5][3] = Kinect::CurrentHeight[3];
            StepUp_Foot_Height[(2+i)%5][4] = Kinect::CurrentHeight[2];
            StepUp_Foot_Height[(0+i)%5][5] = Kinect::CurrentHeight[3];

            cout<<"FOOT HEIGHT"<<endl<<StepUp_Foot_Height[i%5][0]<<endl<<StepUp_Foot_Height[i%5][1]<<endl<<StepUp_Foot_Height[i%5][2]
                    <<endl<<StepUp_Foot_Height[i%5][3]<<endl<<StepUp_Foot_Height[i%5][4]<<endl<<StepUp_Foot_Height[i%5][5]<<endl;

            Kinect::IsCaptureEnd = false;
            Aris::Core::MSG stepup_msg;
            stepup_msg.SetMsgID(StepUp);
            stepup_msg.SetLength(6 * sizeof(double));
            stepup_msg.Copy(StepUp_Foot_Height[i%5], sizeof(StepUp_Foot_Height[i%5]));
            pVisualSystem->SendData(stepup_msg);

            GaitFile_Up<<StepUp_Foot_Height[i%5][0]<<" "<<StepUp_Foot_Height[i%5][1]<<" "<<StepUp_Foot_Height[i%5][2]
                    <<" "<<StepUp_Foot_Height[i%5][3]<<" "<<StepUp_Foot_Height[i%5][4]<<" "<<StepUp_Foot_Height[i%5][5]<<endl;
            GaitFile_Up.close();
            break;
        }
    }
    return 0;
}
int OnStepDown(Aris::Core::MSG &msg)
{
    cout<<"Step Down!!!"<<endl;

    fstream GaitFile_StepDown;
    static double StepDown_Foot_Height[5][6] = {0};

    GaitFile_StepDown.open(GaitLog_StepDown, ios::out|ios::app);

    visionsensor.capture(&i);

    while(1)
    {
        if(Kinect::IsCaptureEnd == true)
        {
            for(int m = 0; m < 4; m++)
            {
                if(abs(Kinect::CurrentHeight[m] + 1.05) < 0.05 )
                {
                    Kinect::CurrentHeight[m] = -1.05;
                }

                if(Kinect::CurrentHeight[m] < -1.05 )
                {
                    Kinect::CurrentHeight[m] = -1.05;
                }

                if(Kinect::CurrentHeight[m] > -0.85 )
                {
                    Kinect::CurrentHeight[m] = -0.85;
                }

            }

            StepDown_Foot_Height[(4+i)%5][0] = Kinect::CurrentHeight[1];
            StepDown_Foot_Height[(2+i)%5][1] = Kinect::CurrentHeight[0];
            StepDown_Foot_Height[(0+i)%5][2] = Kinect::CurrentHeight[1];
            StepDown_Foot_Height[(4+i)%5][3] = Kinect::CurrentHeight[3];
            StepDown_Foot_Height[(2+i)%5][4] = Kinect::CurrentHeight[2];
            StepDown_Foot_Height[(0+i)%5][5] = Kinect::CurrentHeight[3];

            cout<<"FOOT HEIGHT"<<endl<<StepDown_Foot_Height[i%5][0]<<endl<<StepDown_Foot_Height[i%5][1]<<endl<<StepDown_Foot_Height[i%5][2]
                    <<endl<<StepDown_Foot_Height[i%5][3]<<endl<<StepDown_Foot_Height[i%5][4]<<endl<<StepDown_Foot_Height[i%5][5]<<endl;

            Kinect::IsCaptureEnd = false;
            Aris::Core::MSG stepdown_msg;
            stepdown_msg.SetMsgID(StepDown);
            stepdown_msg.SetLength(6 * sizeof(double));
            stepdown_msg.Copy(StepDown_Foot_Height[i%5], sizeof(StepDown_Foot_Height[i%5]));
            pVisualSystem->SendData(stepdown_msg);

            GaitFile_StepDown<<StepDown_Foot_Height[i%5][0]<<" "<<StepDown_Foot_Height[i%5][1]<<" "<<StepDown_Foot_Height[i%5][2]
                    <<" "<<StepDown_Foot_Height[i%5][3]<<" "<<StepDown_Foot_Height[i%5][4]<<" "<<StepDown_Foot_Height[i%5][5]<<endl;
            GaitFile_StepDown.close();
            break;
        }
    }
    return 0;
}
int OnStepOver(Aris::Core::MSG &msg)
{
    cout<<"Step Over!!!"<<endl;

    static double counter = 0.0;

    visionsensor.capture(&i);

    while(1)
    {
        if(Kinect::IsCaptureEnd == true)
        {
            double stepover_data[4] = {0, 0, 0, 0};
            stepover_data[0] = counter;
            if(int(stepover_data[0]) % 2 == 0)
            {
                stepover_data[3] = 0.55;
            }
            else
            {
                stepover_data[3] = 0.10;
            }

            Aris::Core::MSG stepover_msg;
            stepover_msg.SetMsgID(Move);
            stepover_msg.SetLength(4*sizeof(double));
            stepover_msg.Copy(stepover_data, sizeof(stepover_data));
            pVisualSystem->SendData(stepover_msg);
            counter++;
            Kinect::IsCaptureEnd = false;
            break;
        }
    }
    return 0;
}
