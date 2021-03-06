#include"Gait.h"

bool CGait::IsKeepSafeRegistered;
bool CGait::IsKeepSafeStepRegistered;

bool CGait::isReadytoSetGait[AXIS_NUMBER];
EGAIT CGait::m_currentGait[AXIS_NUMBER];
long long int CGait::m_gaitStartTime[AXIS_NUMBER];
int CGait::m_gaitCurrentIndex[AXIS_NUMBER];
EGaitState CGait::m_gaitState[AXIS_NUMBER];
Aris::RT_CONTROL::CMotorData CGait::m_standStillData[AXIS_NUMBER];
Aris::RT_CONTROL::CMotorData CGait::m_commandDataMapped[AXIS_NUMBER];
Aris::RT_CONTROL::CMotorData CGait::m_feedbackDataMapped[AXIS_NUMBER];

///////////////////////////////////////////////
int GaitHome2Start[GAIT_HOME2START_LEN][GAIT_WIDTH];

int GaitMoveForward[GAIT_MOVE_LEN][GAIT_WIDTH];
int GaitMoveBackward[GAIT_MOVE_LEN][GAIT_WIDTH];

int GaitTurnLeft[GAIT_TURN_LEN][GAIT_WIDTH];
int GaitTurnRight[GAIT_TURN_LEN][GAIT_WIDTH];

int GaitLegUp[GAIT_LEGUP_LEN][GAIT_WIDTH];
///////////////////////////////////////////////

const int MapAbsToPhy[18]=
{
    10,	11,	9,
    12,	14,	13,
    17,	15,	16,
    6,	8,	7,
    3,	5,	4,
    0,	2,	1
};
const int MapPhyToAbs[18]=
{
    15,	17,	16,
    12,	14,	13,
    9,	11,	10,
    2,	0,	1,
    3,	5,	4,
    7,	8,	6
};

CGait::CGait()
{
    for(int i=1;i<AXIS_NUMBER;i++)
    {
        CGait::IsKeepSafeRegistered = false;
        CGait::IsKeepSafeStepRegistered = false;
        CGait::m_currentGait[i]=EGAIT::GAIT_NULL;
        CGait::m_gaitState[i]=EGaitState::NONE;
    }
}

CGait::~CGait()
{
}

void CGait::MapFeedbackDataIn(Aris::RT_CONTROL::CMachineData& p_data )
{
    for(int i=0;i<MOTOR_NUM;i++)
    {
        m_feedbackDataMapped[i].Position=p_data.feedbackData[MapAbsToPhy[i]].Position;
        m_feedbackDataMapped[i].Velocity=p_data.feedbackData[MapAbsToPhy[i]].Velocity;
        m_feedbackDataMapped[i].Torque=p_data.feedbackData[MapAbsToPhy[i]].Torque;
    }
}
void CGait::MapCommandDataOut(Aris::RT_CONTROL::CMachineData& p_data )
{
    for(int i=0;i<MOTOR_NUM;i++)
    {
        p_data.commandData[i].Position=m_commandDataMapped[MapPhyToAbs[i]].Position;
        p_data.commandData[i].Velocity=m_commandDataMapped[MapPhyToAbs[i]].Velocity;
        p_data.commandData[i].Torque=m_commandDataMapped[MapPhyToAbs[i]].Torque;
    }
}

bool CGait::IsGaitFinished()
{
    for(int i=0;i<AXIS_NUMBER;i++)
    {
        if(m_gaitState[i]!=EGaitState::GAIT_STOP)
            return false;
    }
    return true;
}

static std::ifstream fin;

int CGait::InitGait(Aris::RT_CONTROL::CSysInitParameters& param)
{
    int Line_Num;
    double temp;

    std::cout << "Start Reading File" << std::endl;
    std::cout<<"reading file data..."<<std::endl;

    fin.open("./gait/TL.txt");
    for(int i=0;i<GAIT_TURN_LEN;i++)
    {
        fin>>Line_Num;
        for(int j=0;j<GAIT_WIDTH;j++)
        {
            fin>>temp;
            GaitTurnLeft[i][j]=-(int)temp;
        }
    }
    fin.close();

    fin.open("./gait/TR.txt");
    for(int i=0;i<GAIT_TURN_LEN;i++)
    {
        fin>>Line_Num;

        for(int j=0;j<GAIT_WIDTH;j++)
        {
            fin>>temp;
            GaitTurnRight[i][j]=-(int)temp;
        }
    }
    fin.close();

    fin.open("./gait/leg_up.txt");
    for(int i=0;i<GAIT_LEGUP_LEN;i++)
    {
        fin>>Line_Num;

        for(int j=0;j<GAIT_WIDTH;j++)
        {
            fin>>temp;
            GaitLegUp[i][j]=-(int)temp;
        }
    }
    fin.close();

    fin.open("./gait/start.txt");
    for(int i=0;i<GAIT_HOME2START_LEN;i++)
    {
        fin>>Line_Num;

        for(int j=0;j<GAIT_WIDTH;j++)
        {
            fin>>temp;
            GaitHome2Start[i][j]=-(int)temp;
        }
    }
    fin.close();

    //FAST FOWARD

    fin.open("./gait/fast_forward.txt");
    for(int i=0; i<GAIT_MOVE_LEN;i++)
    {
        fin>>Line_Num;
        for(int j=0;j<GAIT_WIDTH;j++)
        {
            fin>>temp;
            GaitMoveForward[i][j]=-(int)temp;
        }
    }
    fin.close();

    //FAST BACKWARD

    fin.open("./gait/fast_backward.txt");
    for(int i=0; i<GAIT_MOVE_LEN;i++)
    {
        fin>>Line_Num;
        for(int j=0;j<GAIT_WIDTH;j++)
        {
            fin>>temp;
            GaitMoveBackward[i][j]=-(int)temp;
        }
    }
    fin.close();

    return 0;
}

void CGait::IfReadytoSetGait(bool b,int driverID)
{
    CGait::isReadytoSetGait[driverID]=b;
}

int CGait::RunGait(EGAIT* p_gait,Aris::RT_CONTROL::CMachineData& p_data)
{
    //rt_printf("operation mode %d\n",p_data.motorsModes[0]);

    MapFeedbackDataIn(p_data);

    for(int i=0;i<AXIS_NUMBER;i++)
    {
        if(isReadytoSetGait[i]==true)
        {
            int motorID =MapPhyToAbs[i];
            switch(p_gait[i])
            {
            case GAIT_NULL:
                p_gait[i]=GAIT_STANDSTILL;
                CGait::m_gaitState[i]=EGaitState::GAIT_STOP;

                m_standStillData[motorID].Position=m_feedbackDataMapped[motorID].Position;
                m_standStillData[motorID].Velocity=m_feedbackDataMapped[motorID].Velocity;
                m_standStillData[motorID].Torque=m_feedbackDataMapped[motorID].Torque;

                m_commandDataMapped[motorID].Position=m_standStillData[motorID].Position;
                m_commandDataMapped[motorID].Velocity=m_standStillData[motorID].Velocity;
                m_commandDataMapped[motorID].Torque=m_standStillData[motorID].Torque;

                rt_printf("driver %d: GAIT_NONE will transfer to GAIT_STANDSTILL...\n",i);
                break;

            case GAIT_STANDSTILL:
                if(p_gait[i]!=m_currentGait[i])
                {
                    rt_printf("driver %d:   GAIT_STANDSTILL begins\n",i);
                    m_currentGait[i]=p_gait[i];
                    m_gaitStartTime[i]=p_data.time;

                    m_standStillData[motorID].Position=m_feedbackDataMapped[motorID].Position;
                    m_standStillData[motorID].Velocity=m_feedbackDataMapped[motorID].Velocity;
                    m_standStillData[motorID].Torque=m_feedbackDataMapped[motorID].Torque;

                    m_commandDataMapped[motorID].Position=m_standStillData[motorID].Position;
                    m_commandDataMapped[motorID].Velocity=m_standStillData[motorID].Velocity;
                    m_commandDataMapped[motorID].Torque=m_standStillData[motorID].Torque;
                }
                else
                {
                    m_commandDataMapped[motorID].Position=m_standStillData[motorID].Position;
                    m_commandDataMapped[motorID].Velocity=m_standStillData[motorID].Velocity;
                    m_commandDataMapped[motorID].Torque=m_standStillData[motorID].Torque;
                }
                break;

            case GAIT_HOME:
                m_commandDataMapped[motorID].Position=m_feedbackDataMapped[motorID].Position;
                m_commandDataMapped[motorID].Velocity=m_feedbackDataMapped[motorID].Velocity;
                m_commandDataMapped[motorID].Torque=m_feedbackDataMapped[motorID].Torque;

                if(p_data.isMotorHomed[i]==true)
                {
                    m_standStillData[motorID].Position=m_feedbackDataMapped[motorID].Position;
                    m_standStillData[motorID].Velocity=m_feedbackDataMapped[motorID].Velocity;
                    m_standStillData[motorID].Torque=m_feedbackDataMapped[motorID].Torque;

                    p_gait[i]=GAIT_STANDSTILL;
                    rt_printf("driver %d: HOMED\n",i);

                }
                break;

            case GAIT_HOME2START:

                if(p_gait[i]!=m_currentGait[i])
                {
                    rt_printf("driver %d: GAIT_HOME2START begin\n",i);
                    m_gaitState[i]=EGaitState::GAIT_RUN;
                    m_currentGait[i]=p_gait[i];
                    m_gaitStartTime[i]=p_data.time;
                    m_commandDataMapped[motorID].Position=GaitHome2Start[0][motorID];
                }
                else
                {
                    m_gaitCurrentIndex[i]=(int)(p_data.time-m_gaitStartTime[i]);
                    m_commandDataMapped[motorID].Position=GaitHome2Start[m_gaitCurrentIndex[i]][motorID];


                    if(m_gaitCurrentIndex[i]==GAIT_HOME2START_LEN-1)
                    {
                        rt_printf("driver %d:GAIT_HOME2START will transfer to GAIT_STANDSTILL...\n",i);
                        p_gait[i]=GAIT_STANDSTILL;

                        m_standStillData[motorID].Position=m_feedbackDataMapped[motorID].Position;
                        m_standStillData[motorID].Velocity=m_feedbackDataMapped[motorID].Velocity;
                        m_standStillData[motorID].Torque=m_feedbackDataMapped[motorID].Torque;
                        m_gaitState[i]=EGaitState::GAIT_STOP;
                    }
                }
                break;

            case GAIT_LEGUP:
                if(p_gait[i]!=m_currentGait[i])
                {
                    rt_printf("driver %d: GAIT_LEGUP begin\n",i);
                    m_gaitState[i]=EGaitState::GAIT_RUN;
                    m_currentGait[i]=p_gait[i];
                    m_gaitStartTime[i]=p_data.time;
                    m_commandDataMapped[motorID].Position=GaitLegUp[0][motorID];

                }
                else
                {
                    m_gaitCurrentIndex[i]=(int)(p_data.time-m_gaitStartTime[i]);
                    m_commandDataMapped[motorID].Position=GaitLegUp[m_gaitCurrentIndex[i]][motorID];

                    if(m_gaitCurrentIndex[i]==GAIT_LEGUP_LEN-1)
                    {
                        rt_printf("driver %d:GAIT_LEGUP will transfer to GAIT_STANDSTILL...\n",i);
                        p_gait[i]=GAIT_STANDSTILL;

                        m_standStillData[motorID].Position=m_feedbackDataMapped[motorID].Position;
                        m_standStillData[motorID].Velocity=m_feedbackDataMapped[motorID].Velocity;
                        m_standStillData[motorID].Torque=m_feedbackDataMapped[motorID].Torque;
                        m_gaitState[i]=EGaitState::GAIT_STOP;
                    }
                }
                break;

            case GAIT_TURN_LEFT:
                if(p_gait[i]!=m_currentGait[i])
                {
                    rt_printf("driver %d: GAIT_TURNLEFT begin\n",i);
                    m_gaitState[i]=EGaitState::GAIT_RUN;
                    m_currentGait[i]=p_gait[i];
                    m_gaitStartTime[i]=p_data.time;
                    m_commandDataMapped[motorID].Position=GaitTurnLeft[0][motorID];
                }
                else
                {
                    m_gaitCurrentIndex[i]=(int)(p_data.time-m_gaitStartTime[i]);
                    m_commandDataMapped[motorID].Position=GaitTurnLeft[m_gaitCurrentIndex[i]][motorID];

                    if(m_gaitCurrentIndex[i]==GAIT_TURN_LEN-1)
                    {
                        rt_printf("driver %d:GAIT_TURNLEFT will transfer to GAIT_STANDSTILL...\n",i);
                        p_gait[i]=GAIT_STANDSTILL;

                        m_standStillData[motorID].Position=m_feedbackDataMapped[motorID].Position;
                        m_standStillData[motorID].Velocity=m_feedbackDataMapped[motorID].Velocity;
                        m_standStillData[motorID].Torque=m_feedbackDataMapped[motorID].Torque;
                        m_gaitState[i]=EGaitState::GAIT_STOP;

                        if(IsKeepSafeRegistered == true)
                        {
                            IsKeepSafeStepRegistered = true;
                        }
                    }
                }
                break;

            case GAIT_TURN_RIGHT:

                if(p_gait[i]!=m_currentGait[i])
                {
                    rt_printf("driver %d: GAIT_TURNRIGHT begin\n",i);
                    m_gaitState[i]=EGaitState::GAIT_RUN;
                    m_currentGait[i]=p_gait[i];
                    m_gaitStartTime[i]=p_data.time;
                    m_commandDataMapped[motorID].Position=GaitTurnRight[0][motorID];
                }
                else
                {
                    m_gaitCurrentIndex[i]=(int)(p_data.time-m_gaitStartTime[i]);
                    m_commandDataMapped[motorID].Position=GaitTurnRight[m_gaitCurrentIndex[i]][motorID];


                    if(m_gaitCurrentIndex[i]==GAIT_TURN_LEN-1)
                    {
                        rt_printf("driver %d:GAIT_TURNRIGHT will transfer to GAIT_STANDSTILL...\n",i);
                        p_gait[i]=GAIT_STANDSTILL;

                        m_standStillData[motorID].Position=m_feedbackDataMapped[motorID].Position;
                        m_standStillData[motorID].Velocity=m_feedbackDataMapped[motorID].Velocity;
                        m_standStillData[motorID].Torque=m_feedbackDataMapped[motorID].Torque;
                        m_gaitState[i]=EGaitState::GAIT_STOP;

                        if(IsKeepSafeRegistered == true)
                        {
                            IsKeepSafeStepRegistered = true;
                        }
                    }
                }
                break;

            case GAIT_MOVE:

                if(p_gait[i]!=m_currentGait[i])
                {
                    rt_printf("driver %d: GAIT_MOVE begin\n",i);
                    m_gaitState[i]=EGaitState::GAIT_RUN;
                    m_currentGait[i]=p_gait[i];
                    m_gaitStartTime[i]=p_data.time;
                    m_commandDataMapped[motorID].Position=GaitMoveForward[0][motorID];

                    rt_printf("driver %d:Begin Accelerating foward...\n",i);
                }
                else
                {
                    m_gaitCurrentIndex[i]=(int)(p_data.time-m_gaitStartTime[i]);
                    m_commandDataMapped[motorID].Position=GaitMoveForward[m_gaitCurrentIndex[i]][motorID];

                    if(m_gaitCurrentIndex[i]==GAIT_MOVE_LEN-1)
                    {
                        rt_printf("driver %d: GAIT_MOVE will transfer to GAIT_STANDSTILL...\n",i);
                        p_gait[i]=GAIT_STANDSTILL;

                        m_standStillData[motorID].Position=m_feedbackDataMapped[motorID].Position;
                        m_standStillData[motorID].Velocity=m_feedbackDataMapped[motorID].Velocity;
                        m_standStillData[motorID].Torque=m_feedbackDataMapped[motorID].Torque;
                        m_gaitState[i]=EGaitState::GAIT_STOP;

                        if(IsKeepSafeRegistered == true)
                        {
                            IsKeepSafeStepRegistered = true;
                        }
                    }
                }
                break;

            case GAIT_MOVE_BACK:
                if(p_gait[i]!=m_currentGait[i])
                {
                    rt_printf("driver %d: GAIT_MOVE_BACK begin\n",i);
                    m_gaitState[i]=EGaitState::GAIT_RUN;
                    m_currentGait[i]=p_gait[i];
                    m_gaitStartTime[i]=p_data.time;
                    m_commandDataMapped[motorID].Position=GaitMoveBackward[0][motorID];

                    rt_printf("driver %d:Begin Accelerating backward...\n",i);
                }
                else
                {
                    m_gaitCurrentIndex[i]=(int)(p_data.time-m_gaitStartTime[i]);
                    m_commandDataMapped[motorID].Position=GaitMoveBackward[m_gaitCurrentIndex[i]][motorID];

                    if(m_gaitCurrentIndex[i]==GAIT_MOVE_LEN-1)
                    {
                        rt_printf("driver %d: GAIT_MOVE_BACK will transfer to GAIT_STANDSTILL...\n",i);

                        p_gait[i]=GAIT_STANDSTILL;


                        m_standStillData[motorID].Position=m_feedbackDataMapped[motorID].Position;
                        m_standStillData[motorID].Velocity=m_feedbackDataMapped[motorID].Velocity;
                        m_standStillData[motorID].Torque=m_feedbackDataMapped[motorID].Torque;

                        //only in this cycle, out side get true from IsGaitFinished()
                        m_gaitState[i]=EGaitState::GAIT_STOP;

                        if(IsKeepSafeRegistered == true)
                        {
                            IsKeepSafeStepRegistered = true;
                        }
                    }
                }
                break;

            default:
                p_gait[i]=GAIT_STANDSTILL;
                rt_printf("enter the default\n");

                break;
            }
        }
    }
    MapCommandDataOut(p_data);

    return 0;
}

