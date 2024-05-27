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
	// �������ݲɼ�λ��
	vector<std::array<double, 6>> dataCollectPose;

	// ʹ��circCenter�͵�λ��ת����ɻ�����Բ�ĵı任����
	Eigen::Matrix4d T_BO = Eigen::Matrix4d::Identity();
	T_BO.block<3, 1>(0, 3) = circCenter;
	cout << "T_BO: " << endl << T_BO << endl;

	// ��ȡ��ǰ������ĩ��λ��
	//endPos = m_robot->robotPtr->posture(rokae::CoordinateType::flangeInBase, ec);
	std::array<double, 16> endArray;
	rokae::Utils::postureToTransArray(endPos, endArray);
	// ת��Ϊ��ת�����ƽ������
	Eigen::Matrix3d rotMatrix;
	Eigen::Vector3d transVector;
	rokae::Utils::arrayToTransMatrix(endArray, rotMatrix, transVector);
	cout << "rotMatrix(������ĩ�˷���): " << endl << rotMatrix << endl;
	cout << "transVector(������ĩ�˷���): " << endl << transVector << endl;
	// �����α任����
	Eigen::Matrix4d T_BT = Eigen::Matrix4d::Identity();
	T_BT.block<3, 3>(0, 0) = rotMatrix;
	T_BT.block<3, 1>(0, 3) = transVector;
	cout << "T_BT: " << endl << T_BT << endl;

	// ����Բ�ĵ�ĩ�˵ı任����
	Eigen::Matrix4d T_OT = T_BO.inverse() * T_BT;
	cout << "T_OT: " << endl << T_OT << endl;

	// ��ȡx�᷽������
	Eigen::Vector3d xVector = rotMatrix.col(0);
	cout << "xVector(ĩ�˷���X��): " << endl << xVector << endl;

	for (int i = 0; i < anglesX.size(); i++) {
		// ��x����ת������
		Eigen::AngleAxisd rotationX(anglesX[i] / 180 * M_PI, xVector.normalized());
		Eigen::Matrix4d T_rotationX = Eigen::Matrix4d::Identity();
		T_rotationX.block<3, 3>(0, 0) = rotationX.matrix();
		cout << "T_rotationX: " << endl << T_rotationX << endl;

		// X��ƽ�淨���� Բ��λ�õ�
		Eigen::Matrix4d T_OT_rotatedX = T_rotationX * T_OT;
		cout << "T_OT_rotated: " << endl << T_OT_rotatedX << endl;

		for (int j = 0; j < anglesY.size(); j++) {
			// ��ǰ��̬R_OT_rotatedX
			Eigen::Matrix3d rotMatrixX;
			rotMatrixX = T_OT_rotatedX.block<3, 3>(0, 0);

			// y�� Y_O
			Eigen::Vector3d yVector_rotatedX = rotMatrixX.col(1);
			// ��y����ת������
			Eigen::AngleAxisd rotationY(anglesY[j] / 180 * M_PI, yVector_rotatedX.normalized());
			Eigen::Matrix4d T_rotationY = Eigen::Matrix4d::Identity();
			T_rotationY.block<3, 3>(0, 0) = rotationY.matrix();
			cout << "T_rotationY: " << endl << T_rotationY << endl;

			// Y��ƽ�淨���� Բ��λ�õ�
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