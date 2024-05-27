#include "manipulability.h"

manipulability::manipulability()
{

}

manipulability::~manipulability()
{

}

Eigen::MatrixXd toEigenMatrix(std::vector<std::vector<double> > vec2d)
{
    Eigen::MatrixXd outMat(vec2d.size(), vec2d[0].size());
    for (int i = 0; i < vec2d.size(); i++)
    {
        outMat.row(i) = Eigen::VectorXd::Map(&vec2d[i][0], vec2d[0].size());
    }
    return outMat;
}

std::vector<std::vector<double>> toVector(Eigen::MatrixXd matrix)
{
    std::vector<std::vector<double>> output(matrix.rows(), std::vector<double>(matrix.cols()));
    for (int i = 0; i < matrix.rows(); i++)
    {
        Eigen::VectorXd p = matrix.row(i);
        Eigen::VectorXd::Map(&output[i][0], p.size()) = p;
    }
    return output;
}

// 计算操纵度指标
double manipulability::calcMani(const Eigen::MatrixXd& JacMat) {
    //Eigen::MatrixXd JacMat = toEigenMatrix(Jacobian);
    Eigen::MatrixXd JacMat_T = JacMat.transpose();
    std::cout << JacMat.determinant() << std::endl;
    std::cout << JacMat_T.transpose().determinant() << std::endl;
    // 计算 JJ^T 的行列式
    double det_JJ_T = JacMat.determinant() * JacMat_T.transpose().determinant();

    // 计算操纵度指标
    return std::sqrt(det_JJ_T);
}
