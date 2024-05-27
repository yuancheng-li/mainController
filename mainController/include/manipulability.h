#ifndef __ROBOTMANIPULABILITY_H__
#define __ROBOTMANIPULABILITY_H__

#include <Eigen/Dense>
#include <iostream>
#include <vector>
#include <cmath>


class manipulability
{
public:
	manipulability();
	~manipulability();
	double calcMani(const Eigen::MatrixXd& JacMat);

};

Eigen::MatrixXd toEigenMatrix(std::vector<std::vector<double> > vec2d);
std::vector<std::vector<double>> toVector(Eigen::MatrixXd matrix);


#endif // !__ROBOTMANIPULABILITY_H__

