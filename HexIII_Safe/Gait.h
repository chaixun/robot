#ifndef GAIT_H_
#define GAIT_H_

#include <fstream>
#include <iostream>
#include <Aris_Control.h>
#include <Aris_ControlData.h>

using namespace std;

#define GAIT_WIDTH 18

#define GAIT_MOVE_LEN 3700
#define GAIT_HOME2START_LEN 4001
#define GAIT_LEGUP_LEN 12201
#define GAIT_TURN_LEN 6001

#define MOTOR_NUM 18

enum EGaitState
{
    NONE,
    GAIT_START,
    GAIT_RUN,
    GAIT_STOP,
};

enum EGAIT
{
    GAIT_NULL = 0,
    GAIT_STANDSTILL = 1,
    GAIT_HOME2START = 2,
    GAIT_MOVE = 3,
    GAIT_MOVE_BACK = 4,
    GAIT_TURN_LEFT=5,
    GAIT_TURN_RIGHT=6,
    GAIT_LEGUP = 7,
    GAIT_HOME,
};

class CGait
{
public:
    CGait();
    ~CGait();
    //read txt to array
    static int InitGait(Aris::RT_CONTROL::CSysInitParameters& param);
    static int RunGait(EGAIT* p_gait,Aris::RT_CONTROL::CMachineData& p_data);
    static bool IsGaitFinished();
    //new
    static void  IfReadytoSetGait(bool b, int driverID);
    static bool IsKeepSafeRegistered;
    static bool IsKeepSafeStepRegistered;
    static EGaitState m_gaitState[AXIS_NUMBER];


private:
    //new
    static bool isReadytoSetGait[AXIS_NUMBER];
    static EGAIT m_currentGait[AXIS_NUMBER];
    static long long int m_gaitStartTime[AXIS_NUMBER];
    static int m_gaitCurrentIndex[AXIS_NUMBER];
    static Aris::RT_CONTROL::CMotorData m_standStillData[AXIS_NUMBER];
    static Aris::RT_CONTROL::CMotorData m_commandDataMapped[AXIS_NUMBER];
    static Aris::RT_CONTROL::CMotorData m_feedbackDataMapped[AXIS_NUMBER];
    static void MapFeedbackDataIn(Aris::RT_CONTROL::CMachineData& p_data );
    static void MapCommandDataOut(Aris::RT_CONTROL::CMachineData& p_data );
};

#endif // GAIT_H_
