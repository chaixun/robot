#include "Robot_Vision.h"

#define pi 3.14159265358979323846

Vision_Robot::Vision_Robot()
{

}

Vision_Robot::~Vision_Robot()
{

}

void CalPee(int n, double theta, double ymax, double foot_pos[18], double eur_pos[6])
{
    /***** cal of geometric parameters *****/
    /*cal of initial pos*/
    double t = 6;
    double alpha[6];
    alpha[5] = atan(0.3 / 0.65);
    alpha[3] = pi - alpha[5];
    alpha[0] = pi + alpha[5];
    alpha[2] = 2 * pi - alpha[5];
    alpha[1] = 3 * pi / 2;
    alpha[4] = pi / 2;

    double delta0 = theta * pi / 180;

    /*coordinate of foot in CG*/
    double pos_initial[6][3] =
    { -0.3, -0.85, -0.65,
      -0.45, -0.85, 0,
      -0.3, -0.85, 0.65,
      0.3, -0.85, -0.65,
      0.45, -0.85, 0,
      0.3, -0.85, 0.65 };
    double a1 = 0.3, a2 = 0.65;
    int i, j, k = 2;
    double b1 = pow(a1, k), b2 = pow(a2, k);
    double R1346 = sqrt(b1 + b2);
    double R25 = 0.45;
    double R[6] = { R1346, R25, R1346, R1346, R25, R1346 };

    double pos_end[6][3];
    double pos_ymax[6];


    for (i = 0; i < 6; i++)
    {
        pos_end[i][0] = R[i] * sin(alpha[i] + delta0);
        pos_end[i][1] = pos_initial[i][1];
        pos_end[i][2] = R[i] * cos(alpha[i] + delta0);

        pos_ymax[i] = pos_initial[i][1] + ymax;
    }

    /***** cal of pos of foot *****/
    /*pos design：x = Acos(wt) + K*/
    double Ax[6], Kx[6], Ay[6], Ky[6], Az[6], Kz[6];
    for (i = 0; i < 6; i++)
    {
        Ax[i] = (pos_initial[i][0] - pos_end[i][0]) / 2;
        Kx[i] = (pos_initial[i][0] + pos_end[i][0]) / 2;
        Ay[i] = (pos_initial[i][1] - pos_ymax[i]) / 2;
        Ky[i] = (pos_initial[i][1] + pos_ymax[i]) / 2;
        Az[i] = (pos_initial[i][2] - pos_end[i][2]) / 2;
        Kz[i] = (pos_initial[i][2] + pos_end[i][2]) / 2;
    }

    int length_t_half = t * 500;
    int length_t = 2 * length_t_half;

    /*cal trajectory in half time of each leg in x,y,z*/
    double x[6], y[6], z[6];
    for (j = 0; j < 6; j++)
    {
        if (n < length_t_half)
        {
            x[j] = Ax[j] * cos(2 * pi / t * n*0.001) + Kx[j];
            y[j] = Ay[j] * cos(4 * pi / t * n*0.001) + Ky[j];
            z[j] = Az[j] * cos(2 * pi / t * n*0.001) + Kz[j];
        }
        else
        {
            x[j] = Ax[j] * cos(2 * pi / t * (n-length_t_half)*0.001) + Kx[j];
            y[j] = Ay[j] * cos(4 * pi / t * (n - length_t_half)*0.001) + Ky[j];
            z[j] = Az[j] * cos(2 * pi / t * (n - length_t_half)*0.001) + Kz[j];
        }

    }

    /*Get the output 6*3 foot_pos matrix for trajectory at given time*/
    if (n<length_t_half)
    {
        /*leg 1,3,5 move, leg 2,4,6 stay pos_initial*/
        foot_pos[0] = x[0];
        foot_pos[1] = y[0];
        foot_pos[2] = z[0];

        foot_pos[3] = pos_initial[1][0];
        foot_pos[4] = pos_initial[1][1];
        foot_pos[5] = pos_initial[1][2];

        foot_pos[6] = x[2];
        foot_pos[7] = y[2];
        foot_pos[8] = z[2];

        foot_pos[9] = pos_initial[3][0];
        foot_pos[10] = pos_initial[3][1];
        foot_pos[11] = pos_initial[3][2];

        foot_pos[12] = x[4];
        foot_pos[13] = y[4];
        foot_pos[14] = z[4];

        foot_pos[15] = pos_initial[5][0];
        foot_pos[16] = pos_initial[5][1];
        foot_pos[17] = pos_initial[5][2];
    }
    else
    {
        /*leg 1,3,5 stay pos_end, leg 2,4,6 move*/
        foot_pos[0] = pos_end[0][0];
        foot_pos[1] = pos_end[0][1];
        foot_pos[2] = pos_end[0][2];

        foot_pos[3] = x[1];
        foot_pos[4] = y[1];
        foot_pos[5] = z[1];

        foot_pos[6] = pos_end[2][0];
        foot_pos[7] = pos_end[2][1];
        foot_pos[8] = pos_end[2][2];

        foot_pos[9] = x[3];
        foot_pos[10] = y[3];
        foot_pos[11] = z[3];

        foot_pos[12] = pos_end[4][0];
        foot_pos[13] = pos_end[4][1];
        foot_pos[14] = pos_end[4][2];

        foot_pos[15] = x[5];
        foot_pos[16] = y[5];
        foot_pos[17] = z[5];
    }

    /*Get the output 6*1 eur_pos matrix for trajectory at given time*/
    double Ab = -delta0 / 2;
    double Kb = delta0 / 2;
    double delta = Ab*cos(pi/t*n*0.001) + Kb;
    eur_pos[0] = pi / 2;
    eur_pos[1] = delta;
    eur_pos[2] = -pi / 2;
    eur_pos[3] = 0;
    eur_pos[4] = 0;
    eur_pos[5] = 0;
}

int Vision_Robot::Robotbody(double *BodyTrans, double *BodyTransData)
{
#define RATIO -5600000/16000*65536

    memset(BodyTransData, 0, sizeof(double)* 2500 * 18);

    double BodyTransX = BodyTrans[0];
    double BodyTransY = BodyTrans[1];
    double BodyTransZ = BodyTrans[2];

    double pEE[18],pIn[18];

    double pEEo[] =
    {
        -0.3, -0.85, -0.65,
        -0.45, -0.85,     0,
        -0.3, -0.85,  0.65,
        0.3, -0.85, -0.65,
        0.45, -0.85,     0,
        0.3, -0.85,  0.65
    };

    double bodyEp[6], bodyEpo[6];
    memset(bodyEpo, 0, sizeof(bodyEpo));

    int index = 0;

    /* 加速 */
    memcpy(pEE, pEEo, sizeof(pEE));
    memcpy(bodyEp, bodyEpo, sizeof(bodyEpo));
    for (int i = 0; i < 1250; i++)
    {
        bodyEp[3] = bodyEpo[3] + BodyTransX/2 * acc_even(1250, i + 1);
        bodyEp[4] = bodyEpo[4] + BodyTransY/2 * acc_even(1250, i + 1);
        bodyEp[5] = bodyEpo[5] + BodyTransZ/2 * acc_even(1250, i + 1);

        this->SetPee(pEE, bodyEp, "G");
        this->GetPin(pIn);
        s_daxpy(18, RATIO, pIn, 1, &BodyTransData[(i + index) * 18], 1);
    }
    index += 1250;

    /* 减速P */
    memcpy(pEE, pEEo, sizeof(pEE));
    memcpy(bodyEp, bodyEpo, sizeof(bodyEpo));
    for (int i = 0; i < 1250; i++)
    {
        bodyEp[3] = bodyEpo[3] + BodyTransX/2 + BodyTransX/2 * dec_even(1250, i + 1);
        bodyEp[4] = bodyEpo[4] + BodyTransY/2 + BodyTransY/2 * dec_even(1250, i + 1);
        bodyEp[5] = bodyEpo[5] + BodyTransZ/2 + BodyTransZ/2 * dec_even(1250, i + 1);

        this->SetPee(pEE, bodyEp, "G");
        this->GetPin(pIn);
        s_daxpy(18, RATIO, pIn, 1, &BodyTransData[(i + index) * 18], 1);
    }
    index += 1250;

    return 0;
}

int Vision_Robot::RobotTurn(double *TurnAng, double *TurnData)
{
#define RATIO -5600000/16000*65536

    memset(TurnData, 0, sizeof(double)* 6001 * 18);

    double foot_pos[18], eur_pos[6], pIn[18];
    for (int i = 0; i < 6001; i++)
    {
        CalPee(i, *TurnAng, 0.05, foot_pos, eur_pos);
        this->SetPee(foot_pos, eur_pos, "G");
        this->GetPin(pIn);
        s_daxpy(18, RATIO, pIn, 1, &TurnData[i* 18], 1);
    }

    return 0;
}

int Vision_Robot::RobotMove(double *Move, double *MoveData)
{

#define RATIO -5600000/16000*65536

    memset(MoveData, 0, sizeof(double)* 5000 * 18);

    double pEE[18],pIn[18];

    double pEEo[] =
    {
        -0.3, -0.85, -0.65,
        -0.45, -0.85,     0,
        -0.3, -0.85,  0.65,
        0.3, -0.85, -0.65,
        0.45, -0.85,     0,
        0.3, -0.85,  0.65
    };

    double bodyEp[6], bodyEpo[6];
    memset(bodyEpo, 0, sizeof(bodyEpo));

    /*Step Parameters*/
    /*StepH = b, StepDZ = 2a, StepDX = 2a */
    double StepH = 0.05;
    double StepDZ = Move[2];
    double StepDX = Move[0];

    double Ellipse_ax = StepDX/2;
    double Ellipse_az = StepDZ/2;
    double Ellipse_b = StepH;

    int index = 0;

#pragma region 第一步
    /* 第一步 */

    /* 加速PI/2 */
    memcpy(pEE, pEEo, sizeof(pEE));
    memcpy(bodyEp, bodyEpo, sizeof(bodyEpo));
    for (int i = 0; i < 1250; i++)
    {
        pEE[0] = pEEo[0] + Ellipse_ax - Ellipse_ax * cos(PI/2 * acc_even(1250, i + 1));
        pEE[1] = pEEo[1] + Ellipse_b * sin(PI/2 * acc_even(1250, i + 1));
        pEE[2] = pEEo[2] + Ellipse_az - Ellipse_az * cos(PI/2 * acc_even(1250, i + 1));

        pEE[6] = pEEo[6] + Ellipse_ax - Ellipse_ax * cos(PI/2 * acc_even(1250, i + 1));
        pEE[7] = pEEo[7] + Ellipse_b * sin(PI/2*acc_even(1250, i + 1));
        pEE[8] = pEEo[8] + Ellipse_az - Ellipse_az * cos(PI/2 * acc_even(1250, i + 1));

        pEE[12] = pEEo[12] + Ellipse_ax - Ellipse_ax * cos(PI/2 * acc_even(1250, i + 1));
        pEE[13] = pEEo[13] + Ellipse_b * sin(PI/2 * acc_even(1250, i + 1));
        pEE[14] = pEEo[14] + Ellipse_az - Ellipse_az * cos(PI/2 * acc_even(1250, i + 1));


        bodyEp[3] = bodyEpo[3] + Ellipse_ax/2 * acc_even(1250, i + 1);
        bodyEp[5] = bodyEpo[5] + Ellipse_az/2 * acc_even(1250, i + 1);

        this->SetPee(pEE, bodyEp, "G");
        this->GetPin(pIn);
        s_daxpy(18, RATIO, pIn, 1, &MoveData[(i + index) * 18], 1);
    }
    index += 1250;

    /* 减速PI/2 */
    memcpy(pEE, pEEo, sizeof(pEE));
    memcpy(bodyEp, bodyEpo, sizeof(bodyEpo));
    for (int i = 0; i < 1250; i++)
    {
        pEE[0] = pEEo[0] + Ellipse_ax - Ellipse_ax * cos(PI/2 + PI/2 * dec_even(1250, i + 1));
        pEE[1] = pEEo[1] + Ellipse_b * sin(PI/2 + PI/2 * dec_even(1250, i + 1));
        pEE[2] = pEEo[2] + Ellipse_az - Ellipse_az * cos(PI/2 + PI/2 * dec_even(1250, i + 1));

        pEE[6] = pEEo[6] + Ellipse_ax - Ellipse_ax * cos(PI/2 + PI/2 * dec_even(1250, i + 1));
        pEE[7] = pEEo[7] + Ellipse_b * sin(PI/2 + PI/2*dec_even(1250, i + 1));
        pEE[8] = pEEo[8] + Ellipse_az - Ellipse_az * cos(PI/2 + PI/2 * dec_even(1250, i + 1));

        pEE[12] = pEEo[12] + Ellipse_ax - Ellipse_ax * cos(PI/2 + PI/2 * dec_even(1250, i + 1));
        pEE[13] = pEEo[13] + Ellipse_b * sin(PI/2 + PI/2 * dec_even(1250, i + 1));
        pEE[14] = pEEo[14] + Ellipse_az - Ellipse_az * cos(PI/2 + PI/2 * dec_even(1250, i + 1));


        bodyEp[3] = bodyEpo[3] + Ellipse_ax/2 + Ellipse_ax/2 * dec_even(1250, i + 1);
        bodyEp[5] = bodyEpo[5] + Ellipse_az/2 + Ellipse_az/2 * acc_even(1250, i + 1);

        this->SetPee(pEE, bodyEp, "G");
        this->GetPin(pIn);
        s_daxpy(18, RATIO, pIn, 1, &MoveData[(i + index) * 18], 1);
    }
    index += 1250;
#pragma endregion

#pragma region 第二步
    /* 第二步 */

    /* 加速PI/2 */

    memcpy(pEE, pEEo, sizeof(pEE));
    memcpy(bodyEp, bodyEpo, sizeof(bodyEpo));
    for (int i = 0; i < 1250; i++)
    {
        pEE[0] = pEEo[0] + 2*Ellipse_ax;
        pEE[1] = pEEo[1];
        pEE[2] = pEEo[2] + 2*Ellipse_az;

        pEE[3] = pEEo[3] + Ellipse_ax - Ellipse_ax * cos(PI/2 * acc_even(1250, i + 1));
        pEE[4] = pEEo[4] + Ellipse_b * sin(PI/2 * acc_even(1250, i + 1));
        pEE[5] = pEEo[5] + Ellipse_az - Ellipse_az * cos(PI/2 * acc_even(1250, i + 1));

        pEE[6] = pEEo[6] + 2*Ellipse_ax;
        pEE[7] = pEEo[7];
        pEE[8] = pEEo[8] + 2*Ellipse_az;

        pEE[9] = pEEo[9] + Ellipse_ax - Ellipse_ax * cos(PI/2 * acc_even(1250, i + 1));
        pEE[10] = pEEo[10] + Ellipse_b * sin(PI/2*acc_even(1250, i + 1));
        pEE[11] = pEEo[11] + Ellipse_az - Ellipse_az * cos(PI/2 * acc_even(1250, i + 1));

        pEE[12] = pEEo[12] + 2*Ellipse_ax;
        pEE[13] = pEEo[13];
        pEE[14] = pEEo[14] + 2*Ellipse_az;

        pEE[15] = pEEo[15] + Ellipse_ax - Ellipse_ax * cos(PI/2 * acc_even(1250, i + 1));
        pEE[16] = pEEo[16] + Ellipse_b * sin(PI/2 * acc_even(1250, i + 1));
        pEE[17] = pEEo[17] + Ellipse_az - Ellipse_az * cos(PI/2 * acc_even(1250, i + 1));


        bodyEp[3] = bodyEpo[3] + Ellipse_ax + Ellipse_ax/2 * acc_even(1250, i + 1);
        bodyEp[5] = bodyEpo[5] + Ellipse_az + Ellipse_az/2 * acc_even(1250, i + 1);

        this->SetPee(pEE, bodyEp, "G");
        this->GetPin(pIn);
        s_daxpy(18, RATIO, pIn, 1, &MoveData[(i + index) * 18], 1);
    }
    index += 1250;

    /* 减速PI/2 */

    memcpy(pEE, pEEo, sizeof(pEE));
    memcpy(bodyEp, bodyEpo, sizeof(bodyEpo));
    for (int i = 0; i < 1250; i++)
    {
        pEE[0] = pEEo[0] + 2*Ellipse_ax;
        pEE[1] = pEEo[1];
        pEE[2] = pEEo[2] + 2*Ellipse_az;

        pEE[3] = pEEo[3] + Ellipse_ax - Ellipse_ax * cos(PI/2 + PI/2 * dec_even(1250, i + 1));
        pEE[4] = pEEo[4] + Ellipse_b * sin(PI/2 + PI/2 * dec_even(1250, i + 1));
        pEE[5] = pEEo[5] + Ellipse_az - Ellipse_az * cos(PI/2 + PI/2 * dec_even(1250, i + 1));

        pEE[6] = pEEo[6] + 2*Ellipse_ax;
        pEE[7] = pEEo[7];
        pEE[8] = pEEo[8] + 2*Ellipse_az;

        pEE[9] = pEEo[9] + Ellipse_ax - Ellipse_ax * cos(PI/2 + PI/2 * dec_even(1250, i + 1));
        pEE[10] = pEEo[10] + Ellipse_b * sin(PI/2 + PI/2*dec_even(1250, i + 1));
        pEE[11] = pEEo[11] + Ellipse_az - Ellipse_az * cos(PI/2 + PI/2 * dec_even(1250, i + 1));

        pEE[12] = pEEo[12] + 2*Ellipse_ax;
        pEE[13] = pEEo[13];
        pEE[14] = pEEo[14] + 2*Ellipse_az;

        pEE[15] = pEEo[15] + Ellipse_ax - Ellipse_ax * cos(PI/2 + PI/2 * dec_even(1250, i + 1));
        pEE[16] = pEEo[16] + Ellipse_b * sin(PI/2 + PI/2 * dec_even(1250, i + 1));
        pEE[17] = pEEo[17] + Ellipse_az - Ellipse_az * cos(PI/2 + PI/2 * dec_even(1250, i + 1));


        bodyEp[3] = bodyEpo[3] + 3*Ellipse_ax/2 + Ellipse_ax/2 * dec_even(1250, i + 1);
        bodyEp[5] = bodyEpo[5] + 3*Ellipse_az/2 + Ellipse_az/2 * dec_even(1250, i + 1);

        this->SetPee(pEE, bodyEp, "G");
        this->GetPin(pIn);
        s_daxpy(18, RATIO, pIn, 1, &MoveData[(i + index) * 18], 1);
    }
    index += 1250;

#pragma endregion

    return 0;
}

int Vision_Robot::RobotStepUp(double *StepUpLen, double *StepUpCurrentPos, double *StepUpNextPos, double *StepUpData)
{

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

    double stepUpH = 0.25;

    //STEP LENGTH
    double stepUpD = *StepUpLen;
    int index = 0;

    for (int i = 0; i < 6; i++)
    {
        pEEo[i * 3 + 1] =  StepUpCurrentPos[i];
    }

#pragma region 第一步
    /*第一步*/

    /*抬起加速*/
    memcpy(pEE, pEEo, sizeof(pEE));
    memcpy(bodyEp, bodyEpo, sizeof(bodyEpo));
    for (int i = 0; i < 1000; i++)
    {
        pEE[1] = pEEo[1] + (stepUpH - (StepUpCurrentPos[0] + 1.05)) / 2 * acc_even(1000, i + 1);
        pEE[7] = pEEo[7] + (stepUpH - (StepUpCurrentPos[2] + 1.05)) / 2 * acc_even(1000, i + 1);
        pEE[13] = pEEo[13] + (stepUpH - (StepUpCurrentPos[4] + 1.05)) / 2 * acc_even(1000, i + 1);

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
        pEE[1] = pEEo[1] + (stepUpH - (StepUpCurrentPos[0] + 1.05)) / 2 * dec_even(1000, i + 1);
        pEE[7] = pEEo[7] + (stepUpH - (StepUpCurrentPos[2] + 1.05)) / 2 * dec_even(1000, i + 1);
        pEE[13] = pEEo[13] + (stepUpH - (StepUpCurrentPos[4] + 1.05)) / 2 * dec_even(1000, i + 1);

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
        pEE[2] = pEEo[2] + stepUpD / 2 * acc_even(1000, i + 1);
        pEE[8] = pEEo[8] + stepUpD / 2 * acc_even(1000, i + 1);
        pEE[14] = pEEo[14] + stepUpD / 2 * acc_even(1000, i + 1);
        bodyEp[5] = bodyEpo[5] + stepUpD / 4 * acc_even(1000, i + 1);

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
        pEE[2] = pEEo[2] + stepUpD / 2 * dec_even(1000, i + 1);
        pEE[8] = pEEo[8] + stepUpD / 2 * dec_even(1000, i + 1);
        pEE[14] = pEEo[14] + stepUpD / 2 * dec_even(1000, i + 1);
        bodyEp[5] = bodyEpo[5] + stepUpD / 4 * dec_even(1000, i + 1);

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
        pEE[1] = pEEo[1] - (stepUpH - (StepUpNextPos[0] + 1.05)) / 2 * acc_even(1000, i + 1);
        pEE[7] = pEEo[7] - (stepUpH - (StepUpNextPos[2] + 1.05)) / 2 * acc_even(1000, i + 1);
        pEE[13] = pEEo[13] - (stepUpH - (StepUpNextPos[4] + 1.05)) / 2 * acc_even(1000, i + 1);

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
        pEE[1] = pEEo[1] - (stepUpH - (StepUpNextPos[0] + 1.05)) / 2 * dec_even(1000, i + 1);
        pEE[7] = pEEo[7] - (stepUpH - (StepUpNextPos[2] + 1.05)) / 2 * dec_even(1000, i + 1);
        pEE[13] = pEEo[13] - (stepUpH - (StepUpNextPos[4] + 1.05)) / 2 * dec_even(1000, i + 1);

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
        pEE[4] = pEEo[4] + (stepUpH - (StepUpCurrentPos[1] + 1.05)) / 2 * acc_even(1000, i + 1);
        pEE[10] = pEEo[10] + (stepUpH - (StepUpCurrentPos[3] + 1.05)) / 2 * acc_even(1000, i + 1);
        pEE[16] = pEEo[16] + (stepUpH - (StepUpCurrentPos[5] + 1.05)) / 2 * acc_even(1000, i + 1);

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
        pEE[4] = pEEo[4] + (stepUpH - (StepUpCurrentPos[1] + 1.05)) / 2 * dec_even(1000, i + 1);
        pEE[10] = pEEo[10] + (stepUpH - (StepUpCurrentPos[3] + 1.05)) / 2 * dec_even(1000, i + 1);
        pEE[16] = pEEo[16] + (stepUpH - (StepUpCurrentPos[5] + 1.05)) / 2 * dec_even(1000, i + 1);

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
        pEE[5] = pEEo[5] + stepUpD / 2 * acc_even(1000, i + 1);
        pEE[11] = pEEo[11] + stepUpD / 2 * acc_even(1000, i + 1);
        pEE[17] = pEEo[17] + stepUpD / 2 * acc_even(1000, i + 1);
        bodyEp[5] = bodyEpo[5] + stepUpD / 4 * acc_even(1000, i + 1);

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
        pEE[5] = pEEo[5] + stepUpD / 2 * dec_even(1000, i + 1);
        pEE[11] = pEEo[11] + stepUpD / 2 * dec_even(1000, i + 1);
        pEE[17] = pEEo[17] + stepUpD / 2 * dec_even(1000, i + 1);
        bodyEp[5] = bodyEpo[5] + stepUpD / 4 * dec_even(1000, i + 1);

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
        pEE[4] = pEEo[4] - (stepUpH - (StepUpNextPos[1] + 1.05)) / 2 * acc_even(1000, i + 1);
        pEE[10] = pEEo[10] - (stepUpH - (StepUpNextPos[3] + 1.05)) / 2 * acc_even(1000, i + 1);
        pEE[16] = pEEo[16] - (stepUpH - (StepUpNextPos[5] + 1.05)) / 2 * acc_even(1000, i + 1);

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
        pEE[4] = pEEo[4] - (stepUpH - (StepUpNextPos[1] + 1.05)) / 2 * dec_even(1000, i + 1);
        pEE[10] = pEEo[10] - (stepUpH - (StepUpNextPos[3] + 1.05)) / 2 * dec_even(1000, i + 1);
        pEE[16] = pEEo[16] - (stepUpH - (StepUpNextPos[5] + 1.05)) / 2 * dec_even(1000, i + 1);

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

int Vision_Robot::RobotStepDown(double *StepDownLen, double *StepDownCurrentPos, double *StepDownNextPos, double *StepDownData)
{

#define RATIO -5600000/16000*65536

    memset(StepDownData, 0, sizeof(double)* 12000 * 18);

    double pEE[18],pIn[18];

    double pEEo[] =
    {
        -0.3, -0.85, -0.65,
        -0.45, -0.85,     0,
        -0.3, -0.85,  0.65,
        0.3, -0.85, -0.65,
        0.45, -0.85,     0,
        0.3, -0.85,  0.65
    };

    double bodyEp[6], bodyEpo[6];
    memset(bodyEpo, 0, sizeof(bodyEpo));

    double StepDownH = 0.05;

    //STEP LENGTH
    double StepDownD = *StepDownLen;
    int index = 0;

    for (int i = 0; i < 6; i++)
    {
        pEEo[i * 3 + 1] =  StepDownCurrentPos[i];
    }

#pragma region 第一步
    /*第一步*/

    /*抬起加速*/
    memcpy(pEE, pEEo, sizeof(pEE));
    memcpy(bodyEp, bodyEpo, sizeof(bodyEpo));
    for (int i = 0; i < 1000; i++)
    {
        pEE[1] = pEEo[1] + (StepDownH + (-StepDownCurrentPos[0] - 0.85)) / 2 * acc_even(1000, i + 1);
        pEE[7] = pEEo[7] + (StepDownH + (-StepDownCurrentPos[2] - 0.85)) / 2 * acc_even(1000, i + 1);
        pEE[13] = pEEo[13] + (StepDownH + (-StepDownCurrentPos[4] - 0.85)) / 2 * acc_even(1000, i + 1);

        this->SetPee(pEE, bodyEp, "G");
        this->GetPin(pIn);
        s_daxpy(18, RATIO, pIn, 1, &StepDownData[(i + index) * 18], 1);
    }
    index += 1000;
    memcpy(pEEo, pEE, sizeof(pEE));
    memcpy(bodyEpo, bodyEp, sizeof(bodyEp));

    /*抬起减速*/
    memcpy(pEE, pEEo, sizeof(pEE));
    memcpy(bodyEp, bodyEpo, sizeof(bodyEpo));
    for (int i = 0; i < 1000; i++)
    {
        pEE[1] = pEEo[1] + (StepDownH + (-StepDownCurrentPos[0] - 0.85)) / 2 * dec_even(1000, i + 1);
        pEE[7] = pEEo[7] + (StepDownH + (-StepDownCurrentPos[2] - 0.85)) / 2 * dec_even(1000, i + 1);
        pEE[13] = pEEo[13] + (StepDownH + (-StepDownCurrentPos[4] - 0.85)) / 2 * dec_even(1000, i + 1);

        this->SetPee(pEE, bodyEp, "G");
        this->GetPin(pIn);
        s_daxpy(18, RATIO, pIn, 1, &StepDownData[(i + index) * 18], 1);
    }
    index += 1000;
    memcpy(pEEo, pEE, sizeof(pEE));
    memcpy(bodyEpo, bodyEp, sizeof(bodyEp));

    /*往前加速*/
    memcpy(pEE, pEEo, sizeof(pEE));
    memcpy(bodyEp, bodyEpo, sizeof(bodyEpo));
    for (int i = 0; i < 1000; i++)
    {
        pEE[2] = pEEo[2] + StepDownD / 2 * acc_even(1000, i + 1);
        pEE[8] = pEEo[8] + StepDownD / 2 * acc_even(1000, i + 1);
        pEE[14] = pEEo[14] + StepDownD / 2 * acc_even(1000, i + 1);
        bodyEp[5] = bodyEpo[5] + StepDownD / 4 * acc_even(1000, i + 1);

        this->SetPee(pEE, bodyEp, "G");
        this->GetPin(pIn);
        s_daxpy(18, RATIO, pIn, 1, &StepDownData[(i + index) * 18], 1);
    }
    index += 1000;
    memcpy(pEEo, pEE, sizeof(pEE));
    memcpy(bodyEpo, bodyEp, sizeof(bodyEp));

    /*往前减速*/
    memcpy(pEE, pEEo, sizeof(pEE));
    memcpy(bodyEp, bodyEpo, sizeof(bodyEpo));
    for (int i = 0; i < 1000; i++)
    {
        pEE[2] = pEEo[2] + StepDownD / 2 * dec_even(1000, i + 1);
        pEE[8] = pEEo[8] + StepDownD / 2 * dec_even(1000, i + 1);
        pEE[14] = pEEo[14] + StepDownD / 2 * dec_even(1000, i + 1);
        bodyEp[5] = bodyEpo[5] + StepDownD / 4 * dec_even(1000, i + 1);

        this->SetPee(pEE, bodyEp, "G");
        this->GetPin(pIn);
        s_daxpy(18, RATIO, pIn, 1, &StepDownData[(i + index) * 18], 1);
    }
    index += 1000;
    memcpy(pEEo, pEE, sizeof(pEE));
    memcpy(bodyEpo, bodyEp, sizeof(bodyEp));

    /*落下加速*/
    memcpy(pEE, pEEo, sizeof(pEE));
    memcpy(bodyEp, bodyEpo, sizeof(bodyEpo));
    for (int i = 0; i < 1000; i++)
    {
        pEE[1] = pEEo[1] - (StepDownH + (-StepDownNextPos[0] - 0.85)) / 2 * acc_even(1000, i + 1);
        pEE[7] = pEEo[7] - (StepDownH + (-StepDownNextPos[2] - 0.85)) / 2 * acc_even(1000, i + 1);
        pEE[13] = pEEo[13] - (StepDownH + (-StepDownNextPos[4] - 0.85)) / 2 * acc_even(1000, i + 1);

        this->SetPee(pEE, bodyEp, "G");
        this->GetPin(pIn);
        s_daxpy(18, RATIO, pIn, 1, &StepDownData[(i + index) * 18], 1);
    }
    index += 1000;
    memcpy(pEEo, pEE, sizeof(pEE));
    memcpy(bodyEpo, bodyEp, sizeof(bodyEp));

    /*落下减速*/
    memcpy(pEE, pEEo, sizeof(pEE));
    memcpy(bodyEp, bodyEpo, sizeof(bodyEpo));
    for (int i = 0; i < 1000; i++)
    {
        pEE[1] = pEEo[1] - (StepDownH + (-StepDownNextPos[0] - 0.85)) / 2 * dec_even(1000, i + 1);
        pEE[7] = pEEo[7] - (StepDownH + (-StepDownNextPos[2] - 0.85)) / 2 * dec_even(1000, i + 1);
        pEE[13] = pEEo[13] - (StepDownH + (-StepDownNextPos[4] - 0.85)) / 2 * dec_even(1000, i + 1);

        this->SetPee(pEE, bodyEp, "G");
        this->GetPin(pIn);
        s_daxpy(18, RATIO, pIn, 1, &StepDownData[(i + index) * 18], 1);
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
        pEE[4] = pEEo[4] + (StepDownH + (-StepDownCurrentPos[1] - 0.85)) / 2 * acc_even(1000, i + 1);
        pEE[10] = pEEo[10] + (StepDownH + (-StepDownCurrentPos[3] - 0.85)) / 2 * acc_even(1000, i + 1);
        pEE[16] = pEEo[16] + (StepDownH + (-StepDownCurrentPos[5] - 0.85)) / 2 * acc_even(1000, i + 1);

        this->SetPee(pEE, bodyEp, "G");
        this->GetPin(pIn);
        s_daxpy(18, RATIO, pIn, 1, &StepDownData[(i + index) * 18], 1);
    }
    index += 1000;
    memcpy(pEEo, pEE, sizeof(pEE));
    memcpy(bodyEpo, bodyEp, sizeof(bodyEp));

    /*抬起减速*/
    memcpy(pEE, pEEo, sizeof(pEE));
    memcpy(bodyEp, bodyEpo, sizeof(bodyEpo));
    for (int i = 0; i < 1000; i++)
    {
        pEE[4] = pEEo[4] + (StepDownH + (-StepDownCurrentPos[1] - 0.85)) / 2 * dec_even(1000, i + 1);
        pEE[10] = pEEo[10] + (StepDownH + (-StepDownCurrentPos[3] - 0.85)) / 2 * dec_even(1000, i + 1);
        pEE[16] = pEEo[16] + (StepDownH + (-StepDownCurrentPos[5] - 0.85)) / 2 * dec_even(1000, i + 1);

        this->SetPee(pEE, bodyEp, "G");
        this->GetPin(pIn);
        s_daxpy(18, RATIO, pIn, 1, &StepDownData[(i + index) * 18], 1);
    }
    index += 1000;
    memcpy(pEEo, pEE, sizeof(pEE));
    memcpy(bodyEpo, bodyEp, sizeof(bodyEp));

    /*往前加速*/
    memcpy(pEE, pEEo, sizeof(pEE));
    memcpy(bodyEp, bodyEpo, sizeof(bodyEpo));
    for (int i = 0; i < 1000; i++)
    {
        pEE[5] = pEEo[5] + StepDownD / 2 * acc_even(1000, i + 1);
        pEE[11] = pEEo[11] + StepDownD / 2 * acc_even(1000, i + 1);
        pEE[17] = pEEo[17] + StepDownD / 2 * acc_even(1000, i + 1);
        bodyEp[5] = bodyEpo[5] + StepDownD / 4 * acc_even(1000, i + 1);

        this->SetPee(pEE, bodyEp, "G");
        this->GetPin(pIn);
        s_daxpy(18, RATIO, pIn, 1, &StepDownData[(i + index) * 18], 1);
    }
    index += 1000;
    memcpy(pEEo, pEE, sizeof(pEE));
    memcpy(bodyEpo, bodyEp, sizeof(bodyEp));

    /*往前减速*/
    memcpy(pEE, pEEo, sizeof(pEE));
    memcpy(bodyEp, bodyEpo, sizeof(bodyEpo));
    for (int i = 0; i < 1000; i++)
    {
        pEE[5] = pEEo[5] + StepDownD / 2 * dec_even(1000, i + 1);
        pEE[11] = pEEo[11] + StepDownD / 2 * dec_even(1000, i + 1);
        pEE[17] = pEEo[17] + StepDownD / 2 * dec_even(1000, i + 1);
        bodyEp[5] = bodyEpo[5] + StepDownD / 4 * dec_even(1000, i + 1);

        this->SetPee(pEE, bodyEp, "G");
        this->GetPin(pIn);
        s_daxpy(18, RATIO, pIn, 1, &StepDownData[(i + index) * 18], 1);
    }
    index += 1000;
    memcpy(pEEo, pEE, sizeof(pEE));
    memcpy(bodyEpo, bodyEp, sizeof(bodyEp));


    /*落下加速*/
    memcpy(pEE, pEEo, sizeof(pEE));
    memcpy(bodyEp, bodyEpo, sizeof(bodyEpo));

    for (int i = 0; i < 1000; i++)
    {
        pEE[4] = pEEo[4] - (StepDownH + (-StepDownNextPos[1] - 0.85)) / 2 * acc_even(1000, i + 1);
        pEE[10] = pEEo[10] - (StepDownH + (-StepDownNextPos[3] - 0.85)) / 2 * acc_even(1000, i + 1);
        pEE[16] = pEEo[16] - (StepDownH + (-StepDownNextPos[5] - 0.85)) / 2 * acc_even(1000, i + 1);

        this->SetPee(pEE, bodyEp, "G");
        this->GetPin(pIn);
        s_daxpy(18, RATIO, pIn, 1, &StepDownData[(i + index) * 18], 1);
    }

    index += 1000;
    memcpy(pEEo, pEE, sizeof(pEE));
    memcpy(bodyEpo, bodyEp, sizeof(bodyEp));

    /*落下减速*/
    memcpy(pEE, pEEo, sizeof(pEE));
    memcpy(bodyEp, bodyEpo, sizeof(bodyEpo));
    for (int i = 0; i < 1000; i++)
    {
        pEE[4] = pEEo[4] - (StepDownH + (-StepDownNextPos[1] - 0.85)) / 2 * dec_even(1000, i + 1);
        pEE[10] = pEEo[10] - (StepDownH + (-StepDownNextPos[3] - 0.85)) / 2 * dec_even(1000, i + 1);
        pEE[16] = pEEo[16] - (StepDownH + (-StepDownNextPos[5] - 0.85)) / 2 * dec_even(1000, i + 1);

        this->SetPee(pEE, bodyEp, "G");
        this->GetPin(pIn);
        s_daxpy(18, RATIO, pIn, 1, &StepDownData[(i + index) * 18], 1);
    }
    index += 1000;
    memcpy(pEEo, pEE, sizeof(pEE));
    memcpy(bodyEpo, bodyEp, sizeof(bodyEp));
#pragma endregion

    return 0;
}

