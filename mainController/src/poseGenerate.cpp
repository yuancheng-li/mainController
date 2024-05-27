#include "poseGenerate.h"
#include "rokae/utility.h"

poseGenerate::poseGenerate()
{
}
poseGenerate::~poseGenerate()
{
}

vector<std::array<double, 6>> poseGenerate::generateDataCollectPose(Eigen::Vector3d circCenter, std::array<double, 6> endPos, Eigen::VectorXd& anglesX, Eigen::VectorXd& anglesY) {
	std::error_code ec;
	cout << "jinru" << endl;
	// 生成数据采集位姿
	vector<std::array<double, 6>> dataCollectPose;

	// 使用circCenter和单位旋转阵组成基座到圆心的变换矩阵
	Eigen::Matrix4d T_BO = Eigen::Matrix4d::Identity();
	T_BO.block<3, 1>(0, 3) = circCenter;
	cout << "T_BO: " << endl << T_BO << endl;

	// 获取当前机器人末端位姿
	//endPos = m_robot->robotPtr->posture(rokae::CoordinateType::flangeInBase, ec);
	std::array<double, 16> endArray;
	rokae::Utils::postureToTransArray(endPos, endArray);
	// 转化为旋转矩阵和平移向量
	Eigen::Matrix3d rotMatrix;
	Eigen::Vector3d transVector;
	rokae::Utils::arrayToTransMatrix(endArray, rotMatrix, transVector);
	cout << "rotMatrix(基座到末端法兰): " << endl << rotMatrix << endl;
	cout << "transVector(基座到末端法兰): " << endl << transVector << endl;
	// 组成齐次变换矩阵
	Eigen::Matrix4d T_BT = Eigen::Matrix4d::Identity();
	T_BT.block<3, 3>(0, 0) = rotMatrix;
	T_BT.block<3, 1>(0, 3) = transVector;
	cout << "T_BT: " << endl << T_BT << endl;

	// 计算圆心到末端的变换矩阵
	Eigen::Matrix4d T_OT = T_BO.inverse() * T_BT;
	cout << "T_OT: " << endl << T_OT << endl;

	// 提取x轴方向向量
	Eigen::Vector3d xVector = rotMatrix.col(0);
	cout << "xVector(末端法兰X轴): " << endl << xVector << endl;

	for (int i = 0; i < anglesX.size(); i++) {
		// 绕x轴旋转作用子
		Eigen::AngleAxisd rotationX(anglesX[i] / 180 * M_PI, xVector.normalized());
		Eigen::Matrix4d T_rotationX = Eigen::Matrix4d::Identity();
		T_rotationX.block<3, 3>(0, 0) = rotationX.matrix();
		cout << "T_rotationX: " << endl << T_rotationX << endl;

		// X轴平面法向量 圆周位置点
		Eigen::Matrix4d T_OT_rotatedX = T_rotationX * T_OT;
		cout << "T_OT_rotated: " << endl << T_OT_rotatedX << endl;

		for (int j = 0; j < anglesY.size(); j++) {
			// 当前姿态R_OT_rotatedX
			Eigen::Matrix3d rotMatrixX;
			rotMatrixX = T_OT_rotatedX.block<3, 3>(0, 0);

			// y轴 Y_O
			Eigen::Vector3d yVector_rotatedX = rotMatrixX.col(1);
			// 绕y轴旋转作用子
			Eigen::AngleAxisd rotationY(anglesY[j] / 180 * M_PI, yVector_rotatedX.normalized());
			Eigen::Matrix4d T_rotationY = Eigen::Matrix4d::Identity();
			T_rotationY.block<3, 3>(0, 0) = rotationY.matrix();
			cout << "T_rotationY: " << endl << T_rotationY << endl;

			// Y轴平面法向量 圆周位置点
			Eigen::Matrix4d T_OT_rotatedY = T_rotationY * T_OT_rotatedX;
			Eigen::Matrix4d T_BT_rotatedY = T_BO * T_OT_rotatedY;
			std::array<double, 16> endMat_rotatedY;
			rokae::Utils::transMatrixToArray(T_BT_rotatedY.block<3, 3>(0, 0), T_BT_rotatedY.block<3, 1>(0, 3), endMat_rotatedY);
			std::array<double, 6> endPos_rotatedY;
			rokae::Utils::transArrayToPosture(endMat_rotatedY, endPos_rotatedY);

			dataCollectPose.push_back(endPos_rotatedY);
		}
	}

	return dataCollectPose;
}