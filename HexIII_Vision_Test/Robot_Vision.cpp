#include "Robot_Vision.h"

Vision_Robot::Vision_Robot()
{

}

Vision_Robot::~Vision_Robot()
{

}

int Vision_Robot::RobotStepUp(double *StepUpCurrentPos, double *StepUpNextPos, double *StepUpData)
{
    double locStepUpCurrentPos[6],locStepUpNextPos[6];

    locStepUpCurrentPos[0] = StepUpCurrentPos[0];
    locStepUpCurrentPos[1] = StepUpCurrentPos[1];
    locStepUpCurrentPos[2] = StepUpCurrentPos[2];
    locStepUpCurrentPos[3] = StepUpCurrentPos[3];
    locStepUpCurrentPos[4] = StepUpCurrentPos[4];
    locStepUpCurrentPos[5] = StepUpCurrentPos[5];

    locStepUpNextPos[0] = StepUpNextPos[0];
    locStepUpNextPos[1] = StepUpNextPos[1];
    locStepUpNextPos[2] = StepUpNextPos[2];
    locStepUpNextPos[3] = StepUpNextPos[3];
    locStepUpNextPos[4] = StepUpNextPos[4];
    locStepUpNextPos[5] = StepUpNextPos[5];

#define RATIO -5600000/16000*65536

    memset(StepUpData, 0, sizeof(double)* 12000 * 18);

    double pEE[18],pIn[18];

    double pEEo[] =
    {
        -0.3, -0.85-0.2, -0.65,
        -0.45, -0.85-0.2,     0,
        -0.3, -0.85-0.2,  0.65,
        0.3, -0.85-0.2, -0.65,
        0.45, -0.85-0.2,     0,
        0.3, -0.85-0.2,  0.65
    };

    double bodyEp[6], bodyEpo[6];
    memset(bodyEpo, 0, sizeof(bodyEpo));

    double stepH = 0.25;

    //STEP LENGTH
    double stepD = 0.65;
    int index = 0;

    for (int i = 0; i < 6; i++)
    {
        pEEo[i * 3 + 1] += locStepUpCurrentPos[i];
    }

#pragma region 第一步
    /*第一步*/

    /*抬起加速*/
    memcpy(pEE, pEEo, sizeof(pEE));
    memcpy(bodyEp, bodyEpo, sizeof(bodyEpo));
    for (int i = 0; i < 1000; i++)
    {
        pEE[1] = pEEo[1] + (stepH - locStepUpCurrentPos[0]) / 2 * acc_even(1000, i + 1);
        pEE[7] = pEEo[7] + (stepH - locStepUpCurrentPos[2]) / 2 * acc_even(1000, i + 1);
        pEE[13] = pEEo[13] + (stepH - locStepUpCurrentPos[4]) / 2 * acc_even(1000, i + 1);

        this->SetPee(pEE, bodyEp, "G");
        this->GetPin(pIn);
        s_daxpy(18, RATIO, pIn, 1, &StepUpData[(i + index) * 18], 1);
    }
    index += 1000;
    memcpy(pEEo, pEE, sizeof(pEE));
    memcpy(bodyEpo, bodyEp, sizeof(bodyEp));

    /*抬起减速*/
    memcpy(pEE, pEEo, sizeof(pEE));
    memcpy(bodyEp, bodyEpo, sizeof(bodyEpo));
    for (int i = 0; i < 1000; i++)
    {
        pEE[1] = pEEo[1] + (stepH - locStepUpCurrentPos[0]) / 2 * dec_even(1000, i + 1);
        pEE[7] = pEEo[7] + (stepH - locStepUpCurrentPos[2]) / 2 * dec_even(1000, i + 1);
        pEE[13] = pEEo[13] + (stepH - locStepUpCurrentPos[4]) / 2 * dec_even(1000, i + 1);

        this->SetPee(pEE, bodyEp, "G");
        this->GetPin(pIn);
        s_daxpy(18, RATIO, pIn, 1, &StepUpData[(i + index) * 18], 1);
    }
    index += 1000;
    memcpy(pEEo, pEE, sizeof(pEE));
    memcpy(bodyEpo, bodyEp, sizeof(bodyEp));

    /*往前加速*/
    memcpy(pEE, pEEo, sizeof(pEE));
    memcpy(bodyEp, bodyEpo, sizeof(bodyEpo));
    for (int i = 0; i < 1000; i++)
    {
        pEE[2] = pEEo[2] + stepD / 4 * acc_even(1000, i + 1);
        pEE[8] = pEEo[8] + stepD / 4 * acc_even(1000, i + 1);
        pEE[14] = pEEo[14] + stepD / 4 * acc_even(1000, i + 1);
        bodyEp[5] = bodyEpo[5] + stepD / 8 * acc_even(1000, i + 1);

        this->SetPee(pEE, bodyEp, "G");
        this->GetPin(pIn);
        s_daxpy(18, RATIO, pIn, 1, &StepUpData[(i + index) * 18], 1);
    }
    index += 1000;
    memcpy(pEEo, pEE, sizeof(pEE));
    memcpy(bodyEpo, bodyEp, sizeof(bodyEp));

    /*往前减速*/
    memcpy(pEE, pEEo, sizeof(pEE));
    memcpy(bodyEp, bodyEpo, sizeof(bodyEpo));
    for (int i = 0; i < 1000; i++)
    {
        pEE[2] = pEEo[2] + stepD / 4 * dec_even(1000, i + 1);
        pEE[8] = pEEo[8] + stepD / 4 * dec_even(1000, i + 1);
        pEE[14] = pEEo[14] + stepD / 4 * dec_even(1000, i + 1);
        bodyEp[5] = bodyEpo[5] + stepD / 8 * dec_even(1000, i + 1);

        this->SetPee(pEE, bodyEp, "G");
        this->GetPin(pIn);
        s_daxpy(18, RATIO, pIn, 1, &StepUpData[(i + index) * 18], 1);
    }
    index += 1000;
    memcpy(pEEo, pEE, sizeof(pEE));
    memcpy(bodyEpo, bodyEp, sizeof(bodyEp));

    /*落下加速*/
    memcpy(pEE, pEEo, sizeof(pEE));
    memcpy(bodyEp, bodyEpo, sizeof(bodyEpo));
    for (int i = 0; i < 1000; i++)
    {
        pEE[1] = pEEo[1] - (stepH - locStepUpNextPos[0]) / 2 * acc_even(1000, i + 1);
        pEE[7] = pEEo[7] - (stepH - locStepUpNextPos[2]) / 2 * acc_even(1000, i + 1);
        pEE[13] = pEEo[13] - (stepH - locStepUpNextPos[4]) / 2 * acc_even(1000, i + 1);

        this->SetPee(pEE, bodyEp, "G");
        this->GetPin(pIn);
        s_daxpy(18, RATIO, pIn, 1, &StepUpData[(i + index) * 18], 1);
    }
    index += 1000;
    memcpy(pEEo, pEE, sizeof(pEE));
    memcpy(bodyEpo, bodyEp, sizeof(bodyEp));

    /*落下减速*/
    memcpy(pEE, pEEo, sizeof(pEE));
    memcpy(bodyEp, bodyEpo, sizeof(bodyEpo));
    for (int i = 0; i < 1000; i++)
    {
        pEE[1] = pEEo[1] - (stepH - locStepUpNextPos[0]) / 2 * dec_even(1000, i + 1);
        pEE[7] = pEEo[7] - (stepH - locStepUpNextPos[2]) / 2 * dec_even(1000, i + 1);
        pEE[13] = pEEo[13] - (stepH - locStepUpNextPos[4]) / 2 * dec_even(1000, i + 1);

        this->SetPee(pEE, bodyEp, "G");
        this->GetPin(pIn);
        s_daxpy(18, RATIO, pIn, 1, &StepUpData[(i + index) * 18], 1);
    }
    index += 1000;
    memcpy(pEEo, pEE, sizeof(pEE));
    memcpy(bodyEpo, bodyEp, sizeof(bodyEp));
#pragma endregion

#pragma region 第二步
    /*第一步*/

    /*抬起加速*/
    memcpy(pEE, pEEo, sizeof(pEE));
    memcpy(bodyEp, bodyEpo, sizeof(bodyEpo));
    for (int i = 0; i < 1000; i++)
    {
        pEE[4] = pEEo[4] + (stepH - locStepUpCurrentPos[1]) / 2 * acc_even(1000, i + 1);
        pEE[10] = pEEo[10] + (stepH - locStepUpCurrentPos[3]) / 2 * acc_even(1000, i + 1);
        pEE[16] = pEEo[16] + (stepH - locStepUpCurrentPos[5]) / 2 * acc_even(1000, i + 1);

        this->SetPee(pEE, bodyEp, "G");
        this->GetPin(pIn);
        s_daxpy(18, RATIO, pIn, 1, &StepUpData[(i + index) * 18], 1);
    }
    index += 1000;
    memcpy(pEEo, pEE, sizeof(pEE));
    memcpy(bodyEpo, bodyEp, sizeof(bodyEp));

    /*抬起减速*/
    memcpy(pEE, pEEo, sizeof(pEE));
    memcpy(bodyEp, bodyEpo, sizeof(bodyEpo));
    for (int i = 0; i < 1000; i++)
    {
        pEE[4] = pEEo[4] + (stepH - locStepUpCurrentPos[1]) / 2 * dec_even(1000, i + 1);
        pEE[10] = pEEo[10] + (stepH - locStepUpCurrentPos[3]) / 2 * dec_even(1000, i + 1);
        pEE[16] = pEEo[16] + (stepH - locStepUpCurrentPos[5]) / 2 * dec_even(1000, i + 1);

        this->SetPee(pEE, bodyEp, "G");
        this->GetPin(pIn);
        s_daxpy(18, RATIO, pIn, 1, &StepUpData[(i + index) * 18], 1);
    }
    index += 1000;
    memcpy(pEEo, pEE, sizeof(pEE));
    memcpy(bodyEpo, bodyEp, sizeof(bodyEp));

    /*往前加速*/
    memcpy(pEE, pEEo, sizeof(pEE));
    memcpy(bodyEp, bodyEpo, sizeof(bodyEpo));
    for (int i = 0; i < 1000; i++)
    {
        pEE[5] = pEEo[5] + stepD / 4 * acc_even(1000, i + 1);
        pEE[11] = pEEo[11] + stepD / 4 * acc_even(1000, i + 1);
        pEE[17] = pEEo[17] + stepD / 4 * acc_even(1000, i + 1);
        bodyEp[5] = bodyEpo[5] + stepD / 8 * acc_even(1000, i + 1);

        this->SetPee(pEE, bodyEp, "G");
        this->GetPin(pIn);
        s_daxpy(18, RATIO, pIn, 1, &StepUpData[(i + index) * 18], 1);
    }
    index += 1000;
    memcpy(pEEo, pEE, sizeof(pEE));
    memcpy(bodyEpo, bodyEp, sizeof(bodyEp));

    /*往前减速*/
    memcpy(pEE, pEEo, sizeof(pEE));
    memcpy(bodyEp, bodyEpo, sizeof(bodyEpo));
    for (int i = 0; i < 1000; i++)
    {
        pEE[5] = pEEo[5] + stepD / 4 * dec_even(1000, i + 1);
        pEE[11] = pEEo[11] + stepD / 4 * dec_even(1000, i + 1);
        pEE[17] = pEEo[17] + stepD / 4 * dec_even(1000, i + 1);
        bodyEp[5] = bodyEpo[5] + stepD / 8 * dec_even(1000, i + 1);

        this->SetPee(pEE, bodyEp, "G");
        this->GetPin(pIn);
        s_daxpy(18, RATIO, pIn, 1, &StepUpData[(i + index) * 18], 1);
    }
    index += 1000;
    memcpy(pEEo, pEE, sizeof(pEE));
    memcpy(bodyEpo, bodyEp, sizeof(bodyEp));


    /*落下加速*/
    memcpy(pEE, pEEo, sizeof(pEE));
    memcpy(bodyEp, bodyEpo, sizeof(bodyEpo));

    for (int i = 0; i < 1000; i++)
    {
        pEE[4] = pEEo[4] - (stepH - locStepUpNextPos[1]) / 2 * acc_even(1000, i + 1);
        pEE[10] = pEEo[10] - (stepH - locStepUpNextPos[3]) / 2 * acc_even(1000, i + 1);
        pEE[16] = pEEo[16] - (stepH - locStepUpNextPos[5]) / 2 * acc_even(1000, i + 1);

        this->SetPee(pEE, bodyEp, "G");
        this->GetPin(pIn);
        s_daxpy(18, RATIO, pIn, 1, &StepUpData[(i + index) * 18], 1);
    }

    index += 1000;
    memcpy(pEEo, pEE, sizeof(pEE));
    memcpy(bodyEpo, bodyEp, sizeof(bodyEp));

    /*落下减速*/
    memcpy(pEE, pEEo, sizeof(pEE));
    memcpy(bodyEp, bodyEpo, sizeof(bodyEpo));
    for (int i = 0; i < 1000; i++)
    {
        pEE[4] = pEEo[4] - (stepH - locStepUpNextPos[1]) / 2 * dec_even(1000, i + 1);
        pEE[10] = pEEo[10] - (stepH - locStepUpNextPos[3]) / 2 * dec_even(1000, i + 1);
        pEE[16] = pEEo[16] - (stepH - locStepUpNextPos[5]) / 2 * dec_even(1000, i + 1);

        this->SetPee(pEE, bodyEp, "G");
        this->GetPin(pIn);
        s_daxpy(18, RATIO, pIn, 1, &StepUpData[(i + index) * 18], 1);
    }
    index += 1000;
    memcpy(pEEo, pEE, sizeof(pEE));
    memcpy(bodyEpo, bodyEp, sizeof(bodyEp));
#pragma endregion

    return 0;

}

