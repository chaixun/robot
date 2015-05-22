#ifndef ROBOT_EXPORTS
#define ROBOT_EXPORTS
#endif

#include "Hexapod_Robot_Prmt.h"
#include <complex>
#include <cstring>

using namespace Aris::DynKer;
using namespace std;
namespace Hexapod_Robot
{

	PRMT::PRMT(void)
	{
	}
	PRMT::~PRMT(void)
	{
	}
	LEGP::LEGP(void)
	{
		P1aMass=10;
		ThighMass=10;
		P2aMass=10;
		P2bMass=10;
		P3aMass=10;
		P3bMass=10;

		
		double loc_U1iep[6] = { PI / 2, PI / 2, 0, 0, 0, 0 };
		double loc_U1jep[6] = { PI / 2, PI / 2, 0, 0, 0, 0 };
		double loc_U2iep[6] = { PI / 2, PI / 2, 0, 0, 0.228, 0.132 };
		//double loc_U2iep[6] = { PI / 2, PI / 2, 0, 0.1, 0.235, 0.128 };
		double loc_U2jep[6] = { PI / 2, PI / 2, 0, 0, 0, 0 };
		double loc_U3iep[6] = { PI / 2, PI / 2, 0, 0, 0.228, -0.132 };
		double loc_U3jep[6] = { PI / 2, PI / 2, 0, 0, 0, 0 };
		double loc_P1iep[6] = { PI / 2, PI / 2, -PI / 2, 0, 0, 0 };
		double loc_P1jep[6] = { PI / 2, PI / 2, -PI / 2, 0, 0, 0 };
		double loc_P2iep[6] = { PI / 2, PI / 2, -PI / 2, 0, 0, 0 };
		double loc_P2jep[6] = { PI / 2, PI / 2, -PI / 2, 0, 0, 0 };
		double loc_P3iep[6] = { PI / 2, PI / 2, -PI / 2, 0, 0, 0 };
		double loc_P3jep[6] = { PI / 2, PI / 2, -PI / 2, 0, 0, 0 };
		double loc_S2iep[6] = { 0, -PI / 2, 0, 0, 0.059, 0.034 };
		double loc_S2jep[6] = { 0, -PI / 2, 0, 0, 0, 0 };
		double loc_S3iep[6] = { 0, -PI / 2, 0, 0, 0.059, -0.034 };
		//double loc_S3iep[6] = { 0, -PI / 2, 0, 0.03, 0.068, -0.036 };
		double loc_S3jep[6] = { 0, -PI / 2, 0, 0, 0, 0 };
		double loc_Sfiep[6] = { 0, 0, 0, 0.042, 0, 0 };
		double loc_Sfjep[6] = { 0, 0, 0,     0, 0, 0 };

		std::memcpy(U1iep, loc_U1iep, sizeof(double)* 6);
		std::memcpy(U1jep, loc_U1jep, sizeof(double)* 6);
		std::memcpy(U2iep, loc_U2iep, sizeof(double)* 6);
		std::memcpy(U2jep, loc_U2jep, sizeof(double)* 6);
		std::memcpy(U3iep, loc_U3iep, sizeof(double)* 6);
		std::memcpy(U3jep, loc_U3jep, sizeof(double)* 6);
		std::memcpy(P1iep, loc_P1iep, sizeof(double)* 6);
		std::memcpy(P1jep, loc_P1jep, sizeof(double)* 6);
		std::memcpy(P2iep, loc_P2iep, sizeof(double)* 6);
		std::memcpy(P2jep, loc_P2jep, sizeof(double)* 6);
		std::memcpy(P3iep, loc_P3iep, sizeof(double)* 6);
		std::memcpy(P3jep, loc_P3jep, sizeof(double)* 6);
		std::memcpy(S2iep, loc_S2iep, sizeof(double)* 6);
		std::memcpy(S2jep, loc_S2jep, sizeof(double)* 6);
		std::memcpy(S3iep, loc_S3iep, sizeof(double)* 6);
		std::memcpy(S3jep, loc_S3jep, sizeof(double)* 6);
		std::memcpy(Sfiep, loc_Sfiep, sizeof(double)* 6);
		std::memcpy(Sfjep, loc_Sfjep, sizeof(double)* 6);

		double temp_P1aInertia[3][3] = { { 3, 0, 0 }, { 0, 3, 0 }, { 0, 0, 3 } };
		double temp_P1aCM[6] = { 0, 0, 0, 0, 0, 0 };
		double temp_P2aInertia[3][3] = { { 3, 0, 0 }, { 0, 3, 0 }, { 0, 0, 3 } };
		double temp_P2aCM[6] = { 0, 0, 0, 0, 0, 0 };
		double temp_P3aInertia[3][3] = { { 3, 0, 0 }, { 0, 3, 0 }, { 0, 0, 3 } };
		double temp_P3aCM[6] = { 0, 0, 0, 0, 0, 0 };
		double temp_ThighInertia[3][3] = { { 3, 0, 0 }, { 0, 3, 0 }, { 0, 0, 3 } };
		double temp_ThighCM[6] = { 0, 0, 0, 0, 0, 0 };
		double temp_P2bInertia[3][3] = { { 3, 0, 0 }, { 0, 3, 0 }, { 0, 0, 3 } };
		double temp_P2bCM[6] = { 0, 0, 0, 0, 0, 0 };
		double temp_P3bInertia[3][3] = { { 3, 0, 0 }, { 0, 3, 0 }, { 0, 0, 3 } };
		double temp_P3bCM[6] = { 0, 0, 0, 0, 0, 0 };

		/*double temp_P1aInertia[3][3] = { { 0, 0, 0 }, { 0, 0, 0 }, { 0, 0, 0 } };
		double temp_P1aCM[6] = { 0, 0, 0, 0, 0, 0 };
		double temp_P2aInertia[3][3] = { { 0, 0, 0 }, { 0, 0, 0 }, { 0, 0, 0 } };
		double temp_P2aCM[6] = { 0, 0, 0, 0, 0, 0 };
		double temp_P3aInertia[3][3] = { { 0, 0, 0 }, { 0, 0, 0 }, { 0, 0, 0 } };
		double temp_P3aCM[6] = { 0, 0, 0, 0, 0, 0 };
		double temp_ThighInertia[3][3] = { { 0, 0, 0 }, { 0, 0, 0 }, { 0, 0, 0 } };
		double temp_ThighCM[6] = { 0, 0, 0, 0, 0, 0 };
		double temp_P2bInertia[3][3] = { { 0, 0, 0 }, { 0, 0, 0 }, { 0, 0, 0 } };
		double temp_P2bCM[6] = { 0, 0, 0, 0, 0, 0 };
		double temp_P3bInertia[3][3] = { { 0, 0, 0 }, { 0, 0, 0 }, { 0, 0, 0 } };
		double temp_P3bCM[6] = { 0, 0, 0, 0, 0, 0 };*/

		/*double temp_P1aInertia[3][3] = { { 0, 0, 0 }, { 0, 0, 0 }, { 0, 0, 0 } };
		double temp_P1aCM[6] = { 0, 0, 0, 0, 0, 0 };
		double temp_P2aInertia[3][3] = { { 0, 0, 0 }, { 0, 0, 0 }, { 0, 0, 0 } };
		double temp_P2aCM[6] = { 0, 0, 0, 0, 0, 0 };
		double temp_P3aInertia[3][3] = { { 0, 0, 0 }, { 0, 0, 0 }, { 0, 0, 0 } };
		double temp_P3aCM[6] = { 0, 0, 0, 0, 0, 0 };
		double temp_ThighInertia[3][3] = { { 1, 0, 0 }, { 0, 2, 0 }, { 0, 0, 3 } };
		double temp_ThighCM[6] = { 0, 0, 0, 0, 0, 0 };
		double temp_P2bInertia[3][3] = { { 0, 0, 0 }, { 0, 0, 0 }, { 0, 0, 0 } };
		double temp_P2bCM[6] = { 0, 0, 0, 0, 0, 0 };
		double temp_P3bInertia[3][3] = { { 0, 0, 0 }, { 0, 0, 0 }, { 0, 0, 0 } };
		double temp_P3bCM[6] = { 0, 0, 0, 0, 0, 0 };*/

		std::memcpy(P1aInertia, temp_P1aInertia, sizeof(double)* 9);
		std::memcpy(P1aCM, temp_P1aCM, sizeof(double)* 6);
		std::memcpy(P2aInertia, temp_P2aInertia, sizeof(double)* 9);
		std::memcpy(P2aCM, temp_P2aCM, sizeof(double)* 6);
		std::memcpy(P3aInertia, temp_P3aInertia, sizeof(double)* 9);
		std::memcpy(P3aCM, temp_P3aCM, sizeof(double)* 6);
		std::memcpy(ThighInertia, temp_ThighInertia, sizeof(double)* 9);
		std::memcpy(ThighCM, temp_ThighCM, sizeof(double)* 6);
		std::memcpy(P2bInertia, temp_P2bInertia, sizeof(double)* 9);
		std::memcpy(P2bCM, temp_P2bCM, sizeof(double)* 6);
		std::memcpy(P3bInertia, temp_P3bInertia, sizeof(double)* 9);
		std::memcpy(P3bCM, temp_P3bCM, sizeof(double)* 6);
	}
	LEGP::~LEGP(void)
	{
	}
	MBDP::MBDP(void)
	{
		this->MbdMass=10;

		double a=-PI*9/36;
		double b=0.416;

		double loc_LFep[6] = { PI / 2, (PI * 2 / 3), -PI / 2 + a, -b / 4, 0, -b / 4 * sqrt((double)3) };
		double loc_LMep[6] = { PI / 2, (PI * 3 / 3), -PI / 2 + a, -b / 2, 0, 0 };
		double loc_LRep[6] = { PI / 2, (PI * 4 / 3), -PI / 2 + a, -b / 4, 0, b / 4 * sqrt((double)3) };
		double loc_RFep[6] = { PI / 2, (PI * 1 / 3), -PI / 2 + a, b / 4, 0, -b / 4 * sqrt((double)3) };
		double loc_RMep[6] = { PI / 2, (PI * 0 / 3), -PI / 2 + a, b / 2, 0, 0 };
		double loc_RRep[6] = { PI / 2, (PI * 5 / 3), -PI / 2 + a, b / 4, 0, b / 4 * sqrt((double)3) };

		std::memcpy(LFep, loc_LFep, sizeof(double)* 6);
		std::memcpy(LMep, loc_LMep, sizeof(double)* 6);
		std::memcpy(LRep, loc_LRep, sizeof(double)* 6);
		std::memcpy(RFep, loc_RFep, sizeof(double)* 6);
		std::memcpy(RMep, loc_RMep, sizeof(double)* 6);
		std::memcpy(RRep, loc_RRep, sizeof(double)* 6);

		double temp_MbdInertia[3][3] = { { 3, 0, 0 }, { 0, 3, 0 }, { 0, 0, 3 } };
		double temp_MbdCM[6] = { 0, 0, 0, 0, 0, 0 };
	
		std::memcpy(MbdInertia, temp_MbdInertia, sizeof(double)* 9);
		std::memcpy(MbdCM, temp_MbdCM, sizeof(double)* 6);
	}
	MBDP::~MBDP(void)
	{
	}

}
