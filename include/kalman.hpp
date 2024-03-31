#ifndef COM_KALMAN
#define COM_KALMAN

#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Sparse"
#include "Transforms.hpp"

class KF{
    public:
        KF(float dt_in);
        virtual ~KF(){};
        void updateKalman(const int contactIndex[4], float IMU[3], const double eul[3], const double relVec[12]);
        void updateKalman(const int contactIndex[4], float IMU[3], const float quat[4], const double relVec[12]);
        void quat2rot(const float q[4], Eigen::Matrix3d &R);
        double* getComPosVel(){ return xk_.block(0,0,6,1).data(); };
        double* getRotMat(){ return Rot_.data();}
        double* getFootPos(){ return xk_.block(6,0,12,1).data(); };

    private:
        Eigen::Matrix<double, 18, 18> A_;
        Eigen::Matrix<double, 18,  3> B_;
        Eigen::Matrix<double, 16, 18> C_;
        Eigen::Matrix<double, 18, 18> Q_;
        Eigen::Matrix<double, 16, 16> R_;

        Eigen::Matrix<double, 18,  1> xk_;
        Eigen::Matrix<double, 18, 18> Pk_;
        
        float dt;
        double sax;
        double say;
        double saz;
        
        Eigen::Matrix3d Rot_;
};

#endif