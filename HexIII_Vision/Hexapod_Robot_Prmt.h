#ifndef PRMT_H
#define PRMT_H

#include "Aris_DynKer.h"

using namespace Aris::DynKer;

namespace Hexapod_Robot
{	
	/** \brief 机器人单腿参数
	*
	* 用于保存机器人中每条单腿的参数。
	*
	*/
	class LEGP
	{
	public:
		double U1iep[6];/*!< \brief 储存坐标系U1i相对于单腿坐标系Base的欧拉角与位置 */
		double U1jep[6];/*!< \brief 储存坐标系U1j相对于部件P1a的欧拉角与位置 */
		double U2iep[6];/*!< \brief 储存坐标系U2i相对于单腿坐标系Base的欧拉角与位置 */
		double U2jep[6];/*!< \brief 储存坐标系U2j相对于部件P2a的欧拉角与位置 */
		double U3iep[6];/*!< \brief 储存坐标系U3i相对于单腿坐标系Base的欧拉角与位置 */
		double U3jep[6];/*!< \brief 储存坐标系U3j相对于部件P3a的欧拉角与位置 */
		double P1iep[6];/*!< \brief 储存坐标系P1i相对于部件P1a的欧拉角与位置 */
		double P1jep[6];/*!< \brief 储存坐标系P1j相对于部件Thigh的欧拉角与位置 */
		double P2iep[6];/*!< \brief 储存坐标系P2i相对于部件P2a的欧拉角与位置 */
		double P2jep[6];/*!< \brief 储存坐标系P2j相对于部件P2b的欧拉角与位置 */
		double P3iep[6];/*!< \brief 储存坐标系P3i相对于部件P3a的欧拉角与位置 */
		double P3jep[6];/*!< \brief 储存坐标系P2j相对于部件P3b的欧拉角与位置 */
		double S2iep[6];/*!< \brief 储存坐标系S2i相对于部件P2b的欧拉角与位置 */
		double S2jep[6];/*!< \brief 储存坐标系S2j相对于部件Thigh的欧拉角与位置 */
		double S3iep[6];/*!< \brief 储存坐标系S3i相对于部件P3b的欧拉角与位置 */
		double S3jep[6];/*!< \brief 储存坐标系S2j相对于部件Thigh的欧拉角与位置 */
		double Sfiep[6];/*!< \brief 储存坐标系Sfi相对于部件Thigh的欧拉角与位置 */
		double Sfjep[6];/*!< \brief 储存坐标系Sfj相对于部件Ground的欧拉角与位置 */

		double P1aMass;/*!< \brief 部件P1a的质量 */
		double P1aInertia[3][3];/*!< \brief 部件P1a的转动惯量 */
		double P1aCM[6];/*!< \brief 部件P1a的质心坐标系的欧拉角与位置 */
		double P2aMass;/*!< \brief 部件P2a的质量 */
		double P2aInertia[3][3];/*!< \brief 部件P2a的转动惯量 */
		double P2aCM[6];/*!< \brief 部件P2a的质心坐标系的欧拉角与位置 */
		double P3aMass;/*!< \brief 部件P3a的质量 */
		double P3aInertia[3][3];/*!< \brief 部件P3a的转动惯量 */
		double P3aCM[6];/*!< \brief 部件P3a的质心坐标系的欧拉角与位置 */
		double ThighMass;/*!< \brief 部件Thigh的质量 */
		double ThighInertia[3][3];/*!< \brief 部件Thigh的转动惯量 */
		double ThighCM[6];/*!< \brief 部件Thigh的质心坐标系的欧拉角与位置 */
		double P2bMass;/*!< \brief 部件P2b的质量 */
		double P2bInertia[3][3];/*!< \brief 部件P2b的转动惯量 */
		double P2bCM[6];/*!< \brief 部件P2b的质心坐标系的欧拉角与位置 */
		double P3bMass;/*!< \brief 部件P3b的质量 */
		double P3bInertia[3][3];/*!< \brief 部件P3b的转动惯量 */
		double P3bCM[6];/*!< \brief 部件P3b的质心坐标系的欧拉角与位置 */
	public:
		LEGP();/*!< \brief 构造函数 */
		~LEGP();/*!< \brief 析构函数 */
	};
	/** \brief 机器人机身参数
	*
	* 用于保存机器人中机身的参数。
	*
	*/
	class MBDP
	{
	public:
		double LFep[6];/*!< \brief 储存左前腿基准坐标系Base的欧拉角与位置 */
		double LMep[6];/*!< \brief 储存左中腿基准坐标系Base的欧拉角与位置 */
		double LRep[6];/*!< \brief 储存左后腿基准坐标系Base的欧拉角与位置 */
		double RFep[6];/*!< \brief 储存右前腿基准坐标系Base的欧拉角与位置 */
		double RMep[6];/*!< \brief 储存右中腿基准坐标系Base的欧拉角与位置 */
		double RRep[6];/*!< \brief 储存右后腿基准坐标系Base的欧拉角与位置 */

		double MbdMass;/*!< \brief 机身部件的质量 */
		double MbdInertia[3][3];/*!< \brief 机身部件的转动惯量 */
		double MbdCM[6];/*!< \brief 机身部件质心坐标系的欧拉角与位置 */

	public:
		MBDP(void);/*!< \brief 构造函数 */
		~MBDP(void);/*!< \brief 析构函数 */
	};
	/** \brief 机器人中的参数
	*
	* 用于保存机器人中的所有参数。
	*
	*/
	class PRMT
	{
	public:
		LEGP LegP;/*!< \brief 机器人单腿参数 */
		MBDP MbdP;/*!< \brief 机器人机身参数 */

	public:
		PRMT(void);/*!< \brief 构造函数 */
		~PRMT(void);/*!< \brief 析构函数 */
	};
}


#endif
