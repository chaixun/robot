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
                rt_printf("FORWARD Command Get in NRT\n" );
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
                rt_printf("BACKWARD Command Get in NRT\n" );
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
                rt_printf("TURNLEFT Command Get in NRT\n" );
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
                rt_printf("TURNRIGHT Command Get in NRT\n" );
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
                rt_printf("LEGUP Command Get in NRT\n" );

            }
        }
        break;
    case MOVE:
        if(gait.m_gaitState[MapAbsToPhy[0]]==GAIT_STOP)
        {
            for(int i=0;i<18;i++)
            {
                machineData.motorsModes[i]=EOperationMode::OM_CYCLICVEL;
                gaitcmd[MapAbsToPhy[i]]=EGAIT::GAIT_MOVE_MAP;
                rt_printf("MOVE_MAP Command Get in NRT\n" );

            }
        }
        break;
    case TURN:
        if(gait.m_gaitState[MapAbsToPhy[0]]==GAIT_STOP)
        {
            for(int i=0;i<18;i++)
            {
                machineData.motorsModes[i]=EOperationMode::OM_CYCLICVEL;
                gaitcmd[MapAbsToPhy[i]]=EGAIT::GAIT_TURN_MAP;
                rt_printf("TURN_MAP Command Get in NRT\n" );

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
                rt_printf("BEGINDISCOVER Command Get in NRT\n" );
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
                rt_printf("ENDDISCOVER Command Get in NRT\n" );
            }
        }
        break;
    case STEPUP:
        if(gait.m_gaitState[MapAbsToPhy[0]]== GAIT_STOP)
        {
            for(int i=0;i<18;i++)
            {
                machineData.motorsModes[i]=EOperationMode::OM_CYCLICVEL;
                gaitcmd[MapAbsToPhy[i]]=EGAIT::GAIT_STEPUP_MAP;
                rt_printf("STEPUP_MAP Command Get in NRT\n" );
            }
        }
        break;
    case STEPDOWN:
        if(gait.m_gaitState[MapAbsToPhy[0]]== GAIT_STOP)
        {
            for(int i=0;i<18;i++)
            {
                machineData.motorsModes[i]=EOperationMode::OM_CYCLICVEL;
                gaitcmd[MapAbsToPhy[i]]=EGAIT::GAIT_STEPDOWN_MAP;
                rt_printf("STEPDOWN_MAP Command Get in NRT\n" );
            }
        }
        break;
    default:
        break;
    }

    gait.RunGait(gaitcmd,machineData);

    if(gait.IsGaitFinished())
    {
        if(CGait::IsMove == true&&CGait::IsMoveEnd == true)
        {
            CGait::IsMove = false;
            CGait::IsMoveEnd = false;
            cout<<"MOVE FINISHED!"<<endl;
            Vision_Msg visioncmd = Vision_UpperControl;
            Aris::Core::MSG visionmsg;
            visionmsg.SetMsgID(VS_Capture);
            visionmsg.SetLength(sizeof(visioncmd));
            visionmsg.Copy(&visioncmd, sizeof(visioncmd));
            PostMsg(visionmsg);
        }

        if(CGait::IsTurn == true&&CGait::IsTurnEnd == true)
        {
            CGait::IsTurn = false;
            CGait::IsTurnEnd = false;
            cout<<"TURN FINISHED!"<<endl;
            Vision_Msg visioncmd = Vision_UpperControl;
            Aris::Core::MSG visionmsg;
            visionmsg.SetMsgID(VS_Capture);
            visionmsg.SetLength(sizeof(visioncmd));
            visionmsg.Copy(&visioncmd, sizeof(visioncmd));
            PostMsg(visionmsg);
        }

        if(CGait::IsBeginDiscoverStart == true && CGait::IsBeginDiscoverEnd == true)
        {
            cout<<"BEGINDISCOVER FINISHED!"<<endl;
            Vision_Msg visioncmd = Vision_StepUp;
            Aris::Core::MSG visionmsg;
            visionmsg.SetMsgID(VS_Capture);
            visionmsg.SetLength(sizeof(visioncmd));
            visionmsg.Copy(&visioncmd, sizeof(visioncmd));
            PostMsg(visionmsg);
            CGait::IsBeginDiscoverStart = false;
            CGait::IsBeginDiscoverStart = false;
        }

        if(CGait::IsEndDiscoverStart == true && CGait::IsEndDiscoverEnd == true)
        {
            cout<<"ENDDISCOVER FINISHED!"<<endl;
            Vision_Msg visioncmd = Vision_UpperControl;
            Aris::Core::MSG visionmsg;
            visionmsg.SetMsgID(VS_Capture);
            visionmsg.SetLength(sizeof(visioncmd));
            visionmsg.Copy(&visioncmd, sizeof(visioncmd));
            PostMsg(visionmsg);
            CGait::IsEndDiscoverStart = false;
            CGait::IsEndDiscoverStart = false;
        }

        if(CGait::IsStepUp == true&&CGait::IsStepUpstep == true)
        {
            CGait::IsStepUpstep = false;
            Vision_Msg visioncmd = Vision_StepUp;
            Aris::Core::MSG visionmsg;
            visionmsg.SetMsgID(VS_Capture);
            visionmsg.SetLength(sizeof(visioncmd));
            visionmsg.Copy(&visioncmd, sizeof(visioncmd));
            PostMsg(visionmsg);
        }

        if(CGait::IsStepDown == true&&CGait::IsStepDownstep == true)
        {
            CGait::IsStepDownstep = false;
            Vision_Msg visioncmd = Vision_StepDown;
            Aris::Core::MSG visionmsg;
            visionmsg.SetMsgID(VS_Capture);
            visionmsg.SetLength(sizeof(visioncmd));
            visionmsg.Copy(&visioncmd, sizeof(visioncmd));
            PostMsg(visionmsg);
        }
        if(CGait::IsStepOver == true&&CGait::IsStepOverstep == true)
        {
            CGait::IsStepOverstep = false;
            Vision_Msg visioncmd = Vision_StepOver;
            Aris::Core::MSG visionmsg;
            visionmsg.SetMsgID(VS_Capture);
            visionmsg.SetLength(sizeof(visioncmd));
            visionmsg.Copy(&visioncmd, sizeof(visioncmd));
            PostMsg(visionmsg);
        }

        if (CGait::IsWalkAvoidRegistered == true && CGait::IsWalkAvoidStepRegistered == true)
        {
            rt_printf("WALK AVOID STEP FINISHED!!!!!\n");
            CGait::IsWalkAvoidStepRegistered = false;
            Vision_Msg visioncmd = Walk_Avoid;
            Aris::Core::MSG visionmsg;
            visionmsg.SetMsgID(VS_Capture);
            visionmsg.SetLength(sizeof(visioncmd));
            visionmsg.Copy(&visioncmd, sizeof(visioncmd));
            PostMsg(visionmsg);
        }
    }

    return 0;
}

