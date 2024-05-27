#ifndef __POSEGENERATE_H__
#define __POSEGENERATE_H__

#include <iostream>
#include <vector>
#include <Eigen/Dense>

using namespace std;

/* 生成控制点位 */
class poseGenerate
{
public:
	struct dataCollect
	{
		Eigen::Vector3d circCenter;
		std::array<double, 6> endPos;
		Eigen::VectorXd anglesX;
		Eigen::VectorXd anglesY;
		vector<std::array<double, 6>> dataCollectPose;
	}m_dC;
	

	poseGenerate();
	~poseGenerate();

	/* 生成数据采集位姿 */
	vector<std::array<double, 6>> generateDataCollectPose(Eigen::Vector3d circCenter, std::array<double, 6> endPos, Eigen::VectorXd& anglesX, Eigen::VectorXd& anglesY);

private:

};

#endif // !__POSEGENERATE_H__