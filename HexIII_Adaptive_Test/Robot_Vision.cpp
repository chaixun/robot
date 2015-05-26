#include "Robot_Vision.h"

Vision_Robot::Vision_Robot()
{

}

Vision_Robot::~Vision_Robot()
{

}


int Vision_Robot::MotionPlanWithKinect(double* currentH, double *nextH, double *data)
{
    double locCurrentH[6],locNextH[6];

    locCurrentH[0] = currentH[0];
    locCurrentH[1] = currentH[1];
    locCurrentH[2] = currentH[2];
    locCurrentH[3] = currentH[3];
    locCurrentH[4] = currentH[4];
    locCurrentH[5] = currentH[5];

    locNextH[0] = nextH[0];
    locNextH[1] = nextH[1];
    locNextH[2] = nextH[2];
    locNextH[3] = nextH[3];
    locNextH[4] = nextH[4];
    locNextH[5] = nextH[5];

#define RATIO -5600000/16000*65536

    memset(data, 0, sizeof(double)* 12000 * 18);

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
        pEEo[i * 3 + 1] += locCurrentH[i];
    }

#pragma region 第一步
    /*第一步*/

    /*抬起加速*/
    memcpy(pEE, pEEo, sizeof(pEE));
    memcpy(bodyEp, bodyEpo, sizeof(bodyEpo));
    for (int i = 0; i < 1000; i++)
    {
        pEE[1] = pEEo[1] + (stepH - locCurrentH[0]) / 2 * acc_even(1000, i + 1);
        pEE[7] = pEEo[7] + (stepH - locCurrentH[2]) / 2 * acc_even(1000, i + 1);
        pEE[13] = pEEo[13] + (stepH - locCurrentH[4]) / 2 * acc_even(1000, i + 1);

        this->SetPee(pEE, bodyEp, "G");
        this->GetPin(pIn);
        s_daxpy(18, RATIO, pIn, 1, &data[(i + index) * 18], 1);
    }
    index += 1000;
    memcpy(pEEo, pEE, sizeof(pEE));
    memcpy(bodyEpo, bodyEp, sizeof(bodyEp));

    /*抬起减速*/
    memcpy(pEE, pEEo, sizeof(pEE));
    memcpy(bodyEp, bodyEpo, sizeof(bodyEpo));
    for (int i = 0; i < 1000; i++)
    {
        pEE[1] = pEEo[1] + (stepH - locCurrentH[0]) / 2 * dec_even(1000, i + 1);
        pEE[7] = pEEo[7] + (stepH - locCurrentH[2]) / 2 * dec_even(1000, i + 1);
        pEE[13] = pEEo[13] + (stepH - locCurrentH[4]) / 2 * dec_even(1000, i + 1);

        this->SetPee(pEE, bodyEp, "G");
        this->GetPin(pIn);
        s_daxpy(18, RATIO, pIn, 1, &data[(i + index) * 18], 1);
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
        s_daxpy(18, RATIO, pIn, 1, &data[(i + index) * 18], 1);
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
        s_daxpy(18, RATIO, pIn, 1, &data[(i + index) * 18], 1);
    }
    index += 1000;
    memcpy(pEEo, pEE, sizeof(pEE));
    memcpy(bodyEpo, bodyEp, sizeof(bodyEp));

    /*落下加速*/
    memcpy(pEE, pEEo, sizeof(pEE));
    memcpy(bodyEp, bodyEpo, sizeof(bodyEpo));
    for (int i = 0; i < 1000; i++)
    {
        pEE[1] = pEEo[1] - (stepH - locNextH[0]) / 2 * acc_even(1000, i + 1);
        pEE[7] = pEEo[7] - (stepH - locNextH[2]) / 2 * acc_even(1000, i + 1);
        pEE[13] = pEEo[13] - (stepH - locNextH[4]) / 2 * acc_even(1000, i + 1);

        this->SetPee(pEE, bodyEp, "G");
        this->GetPin(pIn);
        s_daxpy(18, RATIO, pIn, 1, &data[(i + index) * 18], 1);
    }
    index += 1000;
    memcpy(pEEo, pEE, sizeof(pEE));
    memcpy(bodyEpo, bodyEp, sizeof(bodyEp));

    /*落下减速*/
    memcpy(pEE, pEEo, sizeof(pEE));
    memcpy(bodyEp, bodyEpo, sizeof(bodyEpo));
    for (int i = 0; i < 1000; i++)
    {
        pEE[1] = pEEo[1] - (stepH - locNextH[0]) / 2 * dec_even(1000, i + 1);
        pEE[7] = pEEo[7] - (stepH - locNextH[2]) / 2 * dec_even(1000, i + 1);
        pEE[13] = pEEo[13] - (stepH - locNextH[4]) / 2 * dec_even(1000, i + 1);

        this->SetPee(pEE, bodyEp, "G");
        this->GetPin(pIn);
        s_daxpy(18, RATIO, pIn, 1, &data[(i + index) * 18], 1);
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
        pEE[4] = pEEo[4] + (stepH - locCurrentH[1]) / 2 * acc_even(1000, i + 1);
        pEE[10] = pEEo[10] + (stepH - locCurrentH[3]) / 2 * acc_even(1000, i + 1);
        pEE[16] = pEEo[16] + (stepH - locCurrentH[5]) / 2 * acc_even(1000, i + 1);

        this->SetPee(pEE, bodyEp, "G");
        this->GetPin(pIn);
        s_daxpy(18, RATIO, pIn, 1, &data[(i + index) * 18], 1);
    }
    index += 1000;
    memcpy(pEEo, pEE, sizeof(pEE));
    memcpy(bodyEpo, bodyEp, sizeof(bodyEp));

    /*抬起减速*/
    memcpy(pEE, pEEo, sizeof(pEE));
    memcpy(bodyEp, bodyEpo, sizeof(bodyEpo));
    for (int i = 0; i < 1000; i++)
    {
        pEE[4] = pEEo[4] + (stepH - locCurrentH[1]) / 2 * dec_even(1000, i + 1);
        pEE[10] = pEEo[10] + (stepH - locCurrentH[3]) / 2 * dec_even(1000, i + 1);
        pEE[16] = pEEo[16] + (stepH - locCurrentH[5]) / 2 * dec_even(1000, i + 1);

        this->SetPee(pEE, bodyEp, "G");
        this->GetPin(pIn);
        s_daxpy(18, RATIO, pIn, 1, &data[(i + index) * 18], 1);
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
        s_daxpy(18, RATIO, pIn, 1, &data[(i + index) * 18], 1);
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
        s_daxpy(18, RATIO, pIn, 1, &data[(i + index) * 18], 1);
    }
    index += 1000;
    memcpy(pEEo, pEE, sizeof(pEE));
    memcpy(bodyEpo, bodyEp, sizeof(bodyEp));


    /*落下加速*/
    memcpy(pEE, pEEo, sizeof(pEE));
    memcpy(bodyEp, bodyEpo, sizeof(bodyEpo));

    for (int i = 0; i < 1000; i++)
    {
        pEE[4] = pEEo[4] - (stepH - locNextH[1]) / 2 * acc_even(1000, i + 1);
        pEE[10] = pEEo[10] - (stepH - locNextH[3]) / 2 * acc_even(1000, i + 1);
        pEE[16] = pEEo[16] - (stepH - locNextH[5]) / 2 * acc_even(1000, i + 1);

        this->SetPee(pEE, bodyEp, "G");
        this->GetPin(pIn);
        s_daxpy(18, RATIO, pIn, 1, &data[(i + index) * 18], 1);
    }

    index += 1000;
    memcpy(pEEo, pEE, sizeof(pEE));
    memcpy(bodyEpo, bodyEp, sizeof(bodyEp));

    /*落下减速*/
    memcpy(pEE, pEEo, sizeof(pEE));
    memcpy(bodyEp, bodyEpo, sizeof(bodyEpo));
    for (int i = 0; i < 1000; i++)
    {
        pEE[4] = pEEo[4] - (stepH - locNextH[1]) / 2 * dec_even(1000, i + 1);
        pEE[10] = pEEo[10] - (stepH - locNextH[3]) / 2 * dec_even(1000, i + 1);
        pEE[16] = pEEo[16] - (stepH - locNextH[5]) / 2 * dec_even(1000, i + 1);

        this->SetPee(pEE, bodyEp, "G");
        this->GetPin(pIn);
        s_daxpy(18, RATIO, pIn, 1, &data[(i + index) * 18], 1);
    }
    index += 1000;
    memcpy(pEEo, pEE, sizeof(pEE));
    memcpy(bodyEpo, bodyEp, sizeof(bodyEp));
#pragma endregion

    return 0;

}

