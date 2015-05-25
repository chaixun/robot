#include "Control.h"

void* Thread1(void *)
{
    cout<<"Running MsgLoop"<<endl;
    Aris::Core::RunMsgLoop();
    return NULL;
}

int initFun(CSysInitParameters& param)
{
    gait.InitGait(param);
    return 0;
}

int tg(CMachineData& machineData,RT_MSG& msg)
{
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

    int CommandID;
    CommandID=msg.GetMsgID();

    switch(CommandID)
    {
    case NOCMD:
        for(int i=0;i<18;i++)
        {
            machineData.motorsCommands[i]=EMCMD_NONE;
        }
        rt_printf("NONE Command Get in NRT\n" );
        break;
    case ENABLE:
        for(int i=0;i<18;i++)
        {
            machineData.motorsCommands[i]=EMCMD_ENABLE;
        }
        rt_printf("ENABLE Command Get in NRT\n" );
        break;
    case POWEROFF:
        for(int i=0;i<18;i++)
        {
            machineData.motorsCommands[i]=EMCMD_POWEROFF;
            gait.IfReadytoSetGait(false,i);
        }
        rt_printf("POWEROFF Command Get in NRT\n" );
        break;
    case STOP:
        for(int i=0;i<18;i++)
        {
            machineData.motorsCommands[i]=EMCMD_STOP;
        }
        rt_printf("STOP Command Get in NRT\n" );
        break;
    case RUNNING:
        for(int i=0;i<18;i++)
        {
            machineData.motorsCommands[i]=EMCMD_RUNNING;
            gait.IfReadytoSetGait(true,i);
        }
        rt_printf("RUNNING Command Get in NRT\n" );
        break;
    case GOHOME_1:
        machineData.motorsCommands[MapAbsToPhy[0]]=EMCMD_GOHOME;
        machineData.motorsCommands[MapAbsToPhy[1]]=EMCMD_GOHOME;
        machineData.motorsCommands[MapAbsToPhy[2]]=EMCMD_GOHOME;
        machineData.motorsCommands[MapAbsToPhy[6]]=EMCMD_GOHOME;
        machineData.motorsCommands[MapAbsToPhy[7]]=EMCMD_GOHOME;
        machineData.motorsCommands[MapAbsToPhy[8]]=EMCMD_GOHOME;
        machineData.motorsCommands[MapAbsToPhy[12]]=EMCMD_GOHOME;
        machineData.motorsCommands[MapAbsToPhy[13]]=EMCMD_GOHOME;
        machineData.motorsCommands[MapAbsToPhy[14]]=EMCMD_GOHOME;

        gaitcmd[MapAbsToPhy[0]]=EGAIT::GAIT_HOME;
        gaitcmd[MapAbsToPhy[1]]=EGAIT::GAIT_HOME;
        gaitcmd[MapAbsToPhy[2]]=EGAIT::GAIT_HOME;
        gaitcmd[MapAbsToPhy[6]]=EGAIT::GAIT_HOME;
        gaitcmd[MapAbsToPhy[7]]=EGAIT::GAIT_HOME;
        gaitcmd[MapAbsToPhy[8]]=EGAIT::GAIT_HOME;
        gaitcmd[MapAbsToPhy[12]]=EGAIT::GAIT_HOME;
        gaitcmd[MapAbsToPhy[13]]=EGAIT::GAIT_HOME;
        gaitcmd[MapAbsToPhy[14]]=EGAIT::GAIT_HOME;

        rt_printf("GOHOME_1 Command Get in NRT\n" );
        break;
    case GOHOME_2:
        machineData.motorsCommands[MapAbsToPhy[3]]=EMCMD_GOHOME;
        machineData.motorsCommands[MapAbsToPhy[4]]=EMCMD_GOHOME;
        machineData.motorsCommands[MapAbsToPhy[5]]=EMCMD_GOHOME;
        machineData.motorsCommands[MapAbsToPhy[9]]=EMCMD_GOHOME;
        machineData.motorsCommands[MapAbsToPhy[10]]=EMCMD_GOHOME;
        machineData.motorsCommands[MapAbsToPhy[11]]=EMCMD_GOHOME;
        machineData.motorsCommands[MapAbsToPhy[15]]=EMCMD_GOHOME;
        machineData.motorsCommands[MapAbsToPhy[16]]=EMCMD_GOHOME;
        machineData.motorsCommands[MapAbsToPhy[17]]=EMCMD_GOHOME;

        gaitcmd[MapAbsToPhy[3]]=EGAIT::GAIT_HOME;
        gaitcmd[MapAbsToPhy[4]]=EGAIT::GAIT_HOME;
        gaitcmd[MapAbsToPhy[5]]=EGAIT::GAIT_HOME;
        gaitcmd[MapAbsToPhy[9]]=EGAIT::GAIT_HOME;
        gaitcmd[MapAbsToPhy[10]]=EGAIT::GAIT_HOME;
        gaitcmd[MapAbsToPhy[11]]=EGAIT::GAIT_HOME;
        gaitcmd[MapAbsToPhy[15]]=EGAIT::GAIT_HOME;
        gaitcmd[MapAbsToPhy[16]]=EGAIT::GAIT_HOME;
        gaitcmd[MapAbsToPhy[17]]=EGAIT::GAIT_HOME;

        rt_printf("GOHOME_2 Command Get in NRT\n" );
        break;
    case HOME2START_1:
        if(gait.m_gaitState[MapAbsToPhy[0]]==GAIT_STOP)
        {
            for(int i=0;i<18;i++)
            {
                machineData.motorsModes[i]=EOperationMode::OM_CYCLICVEL;
            }
            gaitcmd[MapAbsToPhy[0]]=EGAIT::GAIT_HOME2START;
            gaitcmd[MapAbsToPhy[1]]=EGAIT::GAIT_HOME2START;
            gaitcmd[MapAbsToPhy[2]]=EGAIT::GAIT_HOME2START;
            gaitcmd[MapAbsToPhy[6]]=EGAIT::GAIT_HOME2START;
            gaitcmd[MapAbsToPhy[7]]=EGAIT::GAIT_HOME2START;
            gaitcmd[MapAbsToPhy[8]]=EGAIT::GAIT_HOME2START;
            gaitcmd[MapAbsToPhy[12]]=EGAIT::GAIT_HOME2START;
            gaitcmd[MapAbsToPhy[13]]=EGAIT::GAIT_HOME2START;
            gaitcmd[MapAbsToPhy[14]]=EGAIT::GAIT_HOME2START;

            rt_printf("HOME2START_1 Command Get in NRT\n" );
        }
        break;
    case HOME2START_2:
        if(gait.m_gaitState[MapAbsToPhy[3]]==GAIT_STOP)
        {
            for(int i=0;i<18;i++)
            {
                machineData.motorsModes[i]=EOperationMode::OM_CYCLICVEL;
            }
            gaitcmd[MapAbsToPhy[3]]=EGAIT::GAIT_HOME2START;
            gaitcmd[MapAbsToPhy[4]]=EGAIT::GAIT_HOME2START;
            gaitcmd[MapAbsToPhy[5]]=EGAIT::GAIT_HOME2START;
            gaitcmd[MapAbsToPhy[9]]=EGAIT::GAIT_HOME2START;
            gaitcmd[MapAbsToPhy[10]]=EGAIT::GAIT_HOME2START;
            gaitcmd[MapAbsToPhy[11]]=EGAIT::GAIT_HOME2START;
            gaitcmd[MapAbsToPhy[15]]=EGAIT::GAIT_HOME2START;
            gaitcmd[MapAbsToPhy[16]]=EGAIT::GAIT_HOME2START;
            gaitcmd[MapAbsToPhy[17]]=EGAIT::GAIT_HOME2START;

            rt_printf("HOME2START_2 Command Get in NRT\n" );
        }
        break;
    case FORWARD:
        if(gait.m_gaitState[MapAbsToPhy[0]]==GAIT_STOP)
        {
            for(int i=0;i<18;i++)
            {
                machineData.motorsModes[i]=EOperationMode::OM_CYCLICVEL;
                gaitcmd[MapAbsToPhy[i]]=EGAIT::GAIT_MOVE;
            }
        }
        break;
    case BACKWARD:
        if(gait.m_gaitState[MapAbsToPhy[0]]==GAIT_STOP)
        {
            for(int i=0;i<18;i++)
            {
                machineData.motorsModes[i]=EOperationMode::OM_CYCLICVEL;
                gaitcmd[MapAbsToPhy[i]]=EGAIT::GAIT_MOVE_BACK;
            }
        }
        break;
    case TURNLEFT:
        if(gait.m_gaitState[MapAbsToPhy[0]]==GAIT_STOP)
        {
            for(int i=0;i<18;i++)
            {
                machineData.motorsModes[i]=EOperationMode::OM_CYCLICVEL;
                gaitcmd[MapAbsToPhy[i]]=EGAIT::GAIT_TURN_LEFT;
            }
        }
        break;
    case TURNRIGHT:
        if(gait.m_gaitState[MapAbsToPhy[0]]==GAIT_STOP)
        {
            for(int i=0;i<18;i++)
            {
                machineData.motorsModes[i]=EOperationMode::OM_CYCLICVEL;
                gaitcmd[MapAbsToPhy[i]]=EGAIT::GAIT_TURN_RIGHT;
            }
        }
        break;
    case LEGUP:
        if(gait.m_gaitState[MapAbsToPhy[0]]==GAIT_STOP)
        {
            for(int i=0;i<18;i++)
            {
                machineData.motorsModes[i]=EOperationMode::OM_CYCLICVEL;
                gaitcmd[MapAbsToPhy[i]]=EGAIT::GAIT_LEGUP;
            }
        }
        break;
    case BEGINDISCOVER:
        if(gait.m_gaitState[MapAbsToPhy[0]]==GAIT_STOP)
        {
            for(int i=0;i<18;i++)
            {
                machineData.motorsModes[i]=EOperationMode::OM_CYCLICVEL;
                gaitcmd[MapAbsToPhy[i]]=EGAIT::GAIT_BEGIN_DISCOVER;
            }
        }
        break;
    case ENDDISCOVER:
        if(gait.m_gaitState[MapAbsToPhy[0]]== GAIT_STOP)
        {
            for(int i=0;i<18;i++)
            {
                machineData.motorsModes[i]=EOperationMode::OM_CYCLICVEL;
                gaitcmd[MapAbsToPhy[i]]=EGAIT::GAIT_END_DISCOVER;
            }
        }
        break;
    case WALKADAPTIVE:
        if(gait.m_gaitState[MapAbsToPhy[0]]==GAIT_STOP)
        {
            for(int i=0;i<18;i++)
            {
                machineData.motorsModes[i]=EOperationMode::OM_CYCLICVEL;
                gaitcmd[MapAbsToPhy[i]]=EGAIT::GAIT_WALK_ADAPTIVE;
            }
        }
        break;
    default:
        //DO NOTHING, CMD AND TRAJ WILL KEEP STILL
        break;
    }
    // gait.IfReadytoSetGait(machineData.isMotorHomed[0]);
    // rt_printf("driver 0 gaitcmd:%d\n",gaitcmd[0]);

    gait.RunGait(gaitcmd,machineData);

    if(gait.IsGaitFinished())
    {
        if (CGait::IsWalkAdaptiveRegistered == true && CGait::IsWalkAdaptiveStepRegistered == true)
        {
            rt_printf("WALK AVOID STEP FINISHED!!!!!\n");
            CGait::IsWalkAdaptiveStepRegistered = false;
            Aris::Core::PostMsg(Aris::Core::MSG(VS_Capture));
        }

        if (CGait::IsWalkAvoidRegistered == true && CGait::IsWalkAvoidStepRegistered == true)
        {
            rt_printf("WALK AVOID STEP FINISHED!!!!!\n");
            CGait::IsWalkAvoidStepRegistered = false;
            Aris::Core::PostMsg(Aris::Core::MSG(VS_Capture));
        }
    }

    return 0;
}

