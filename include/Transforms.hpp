#ifndef TRANSFORM_FUNCS
#define TRANSFORM_FUNCS

#include "math.h"
#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Core"
#include "stdlib.h"
#include "vector"

inline void quat_to_XYZ(double qw, double qx, double qy, double qz, double &roll, double &pitch, double &yaw){
	double aSinInput = 2*(qx*qz + qy*qw);
    roll = atan2( -2*(qy*qz-qx*qw), pow(qw,2)-pow(qx,2)-pow(qy,2)+pow(qz,2) );
    pitch = asin( aSinInput );
    yaw = atan2( -2*(qx*qy - qz*qw), pow(qw,2) + pow(qx,2) - pow(qy,2) - pow(qz,2) );
}

template <typename inputmat, typename outputmat>
inline void quat_to_XYZ(const Eigen::DenseBase<inputmat> &quat, Eigen::DenseBase<outputmat> &eul){
	double roll = 0.0;
	double pitch = 0.0;
	double yaw = 0.0;
	quat_to_XYZ(quat(0),quat(1),quat(2),quat(3),roll,pitch,yaw);
	eul(0) = roll;
	eul(1) = pitch;
	eul(2) = yaw;
}

template <typename inputmat, typename outputmat>
inline void quat_to_R(const Eigen::DenseBase<inputmat> &quat, Eigen::DenseBase<outputmat> &Rot){
	double q0, q1, q2, q3;
	q0=quat(0); q1=quat(1); q2=quat(2); q3=quat(3);
	Rot(0,0) = 2*(q0*q0+q1*q1)-1;
	Rot(1,0) = 2*(q1*q2+q0*q3);
	Rot(2,0) = 2*(q1*q3-q0*q2);

	Rot(0,1) = 2*(q1*q2-q0*q3);
	Rot(1,1) = 2*(q0*q0+q2*q2)-1;
	Rot(2,1) = 2*(q2*q3+q0*q1);

	Rot(0,2) = 2*(q1*q3+q0*q2);
	Rot(1,2) = 2*(q2*q3-q0*q1);
	Rot(2,2) = 2*(q0*q0+q3*q3)-1;
}

template <typename inputarr, typename outputmat>
inline void quat_to_R(const inputarr q[4], Eigen::DenseBase<outputmat> &Rot){
	Rot(0,0) = 2*(q[0]*q[0]+q[1]*q[1])-1;
    Rot(0,1) = 2*(q[1]*q[2]-q[0]*q[3]);
    Rot(0,2) = 2*(q[1]*q[3]+q[0]*q[2]);
    Rot(1,0) = 2*(q[1]*q[2]+q[0]*q[3]);
    Rot(1,1) = 2*(q[0]*q[0]+q[2]*q[2])-1;
    Rot(1,2) = 2*(q[2]*q[3]-q[0]*q[1]);
    Rot(2,0) = 2*(q[1]*q[3]-q[0]*q[2]);
    Rot(2,1) = 2*(q[2]*q[3]+q[0]*q[1]);
    Rot(2,2) = 2*(q[0]*q[0]+q[3]*q[3])-1;
}

inline void R_XYZ(double roll, double pitch, double yaw, Eigen::Matrix<double,3,3> &R){
	double sroll = sin(roll);
	double croll = cos(roll);
	double spitch = sin(pitch);
	double cpitch = cos(pitch);
	double syaw = sin(yaw);
	double cyaw = cos(yaw);

	R(0,0) = cpitch*cyaw;
	R(1,0) = croll*syaw + cyaw*spitch*sroll;
	R(2,0) = sroll*syaw - croll*cyaw*spitch;
	R(0,1) = -cpitch*syaw;
	R(1,1) = croll*cyaw - spitch*sroll*syaw;
	R(2,1) = cyaw*sroll + croll*spitch*syaw;
	R(0,2) = spitch;
	R(1,2) = -cpitch*sroll;
	R(2,2) = cpitch*croll;
}

inline void R_XYZ(Eigen::Matrix<double,3,1> &orient, Eigen::Matrix<double,3,3> &R){
	double roll = orient(0);
	double pitch = orient(1);
	double yaw = orient(2);
	R_XYZ(roll, pitch, yaw, R);
}

template <typename inputmat>
inline void XYZ_to_quat(double roll, double pitch, double yaw, Eigen::DenseBase<inputmat> &quat){
	double  sroll = sin(roll/2); double  croll = cos(roll/2);
	double spitch = sin(pitch/2); double cpitch = cos(pitch/2);
	double   syaw = sin(yaw/2); double   cyaw = cos(yaw/2);

	quat(0) =  croll*cpitch*cyaw-sroll*spitch*syaw;
	quat(1) =  sroll*cpitch*cyaw+croll*spitch*syaw;
	quat(2) = -sroll*cpitch*syaw+croll*spitch*cyaw;
	quat(3) =  croll*cpitch*syaw+sroll*spitch*cyaw;
}

template <typename inputmat, typename outputmat>
inline void XYZ_to_quat(const Eigen::DenseBase<inputmat> &eul, Eigen::DenseBase<outputmat> &quat){
	double  sroll = sin(eul(0)/2); double  croll = cos(eul(0)/2);
	double spitch = sin(eul(1)/2); double cpitch = cos(eul(1)/2);
	double   syaw = sin(eul(2)/2); double   cyaw = cos(eul(2)/2);

	quat(0) =  croll*cpitch*cyaw-sroll*spitch*syaw;
	quat(1) =  sroll*cpitch*cyaw+croll*spitch*syaw;
	quat(2) = -sroll*cpitch*syaw+croll*spitch*cyaw;
	quat(3) =  croll*cpitch*syaw+sroll*spitch*cyaw;
}

inline Eigen::Matrix<double, 3, 1> toWorld(const Eigen::Matrix<double, 3, 1> &data, const Eigen::Matrix<double, 3, 3> &rot){
    return rot * data;
}

inline Eigen::Matrix<double, 3, 1> toBody(const Eigen::Matrix<double, 3, 1> &data, const Eigen::Matrix<double, 3, 3> &rot){
    return rot.transpose() * data;
}

inline void toBody(double *outVec, Eigen::Matrix<double, 3, 1> &data, const Eigen::Matrix<double, 3, 3> &rot){
    data = rot.transpose()*data;
    outVec[0] = data(0); outVec[1] = data(1); outVec[2] = data(2);
}

inline void toWorld(double *outVec, Eigen::Matrix<double, 3, 1> &data, const Eigen::Matrix<double, 3, 3> &rot){
    data = rot*data;
    outVec[0] = data(0); outVec[1] = data(1); outVec[2] = data(2);
}

#endif