#ifndef SHARED_DATA
#define SHARED_DATA

#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/Sparse"

struct sharedData
{
	int ind[4] = {1};
	int cnt = 4;
	Eigen::Matrix<double, 18, 1> q = Eigen::Matrix<double, 18, 1>::Zero();
	Eigen::Matrix<double, 18, 1> dq = Eigen::Matrix<double, 18, 1>::Zero();
	Eigen::Matrix<double, 3, 4> toePos = Eigen::Matrix<double, 3, 4>::Zero();
	Eigen::Matrix<double, 12, 1> comDes = Eigen::Matrix<double, 12, 1>::Zero();
	Eigen::Matrix<double, 12, 1> fDes = Eigen::Matrix<double, 12, 1>::Zero();
	Eigen::Matrix<double, 12, 1> QP_force = Eigen::Matrix<double, 12, 1>::Zero();
};


#endif