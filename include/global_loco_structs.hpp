#ifndef GLOBAL_LOCO_STRUCTS
#define GLOBAL_LOCO_STRUCTS

#include "math.h"
#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/Sparse"
#include "global_loco_opts.h"
#include "vector"
#include "iostream" // REMOVE LATER!!! ++++++++++++++++++++++++++=

struct StateInfo
{
    Eigen::Matrix<double, TOTAL_DOF, 1> q;
    Eigen::Matrix<double, TOTAL_DOF, 1> dq;
    Eigen::Matrix<double, 6, 1> bodyVel;
    Eigen::Matrix<double, 6, 1> bodyPos;
    Eigen::Matrix<double, 3, 3> R;
    Eigen::Matrix<double, TOTAL_DOF, 1> q_prev;
    Eigen::Matrix<double, TOTAL_DOF, 1> dq_prev;
    Eigen::Matrix<double, 3, 1> comFiltered;
};

struct ContactInfo
{
    int ind[4] = {1, 1, 1, 1};
    int ind_prev[4] = {1, 1, 1, 1};
    int des[4] = {1, 1, 1, 1};
    int act[4] = {1, 1, 1, 1};
    int cnt = 4;
    int changeDomain = 0;
};

struct DynamicsInfo
{
    Eigen::Matrix<double, TOTAL_DOF, TOTAL_DOF> D;
    Eigen::Matrix<double, TOTAL_DOF, TOTAL_DOF> Dinv;
    Eigen::Matrix<double, TOTAL_DOF, TOTAL_IN> B;
    Eigen::Matrix<double, TOTAL_DOF, 1> H;
};

struct KinematicsInfo
{
    Eigen::Matrix<double, 3, 4> toePos;
    Eigen::Matrix<double, TOTAL_IN, TOTAL_DOF> Jtoe;
    Eigen::Matrix<double, TOTAL_IN, 1> dJtoe;
    Eigen::Matrix<double, 3, 4> hipPos;
    Eigen::Matrix<double, TOTAL_IN, TOTAL_DOF> Jhip;
    Eigen::Matrix<double, TOTAL_IN, 1> dJhip;
    Eigen::MatrixXd Jc, dJc, Js, dJs;
};

struct VCInfo
{
    Eigen::MatrixXd y, dy;
    Eigen::MatrixXd H0, dH0;
    Eigen::MatrixXd hd, dhd, ddhd;
    Eigen::MatrixXd hd_ST, dhd_ST; // desired stance toe, and stance toe dot 
    Eigen::MatrixXd y_ST, dy_ST;
    Eigen::Matrix<double, 12, 1> fDes;
};

struct TrajInfo
{
    Eigen::MatrixXd redDes;
    Eigen::Matrix<double, 12, 1> comDes;
    double toeOffset[3] = {0.0,0.0,0.0};
    double stepLen[3] = {0.0,0.0,0.0};
    Eigen::Matrix<double, 3, 4> toeInit;
    Eigen::Matrix<double, 3, 4> toeFinal;
    double domLen = 0;
};

struct LLInfo{
    double tau[18] = {0};
    Eigen::Matrix<double, 12, 1> QP_force;
    Eigen::Matrix<double, 18, 1> q,dq,ddq;
    double V = 0;
    double dV = 0;
};

namespace Settings{

struct MPC_params{
    double qpx, qpy, qpz; // Position gains
    double qvx, qvy, qvz; // Velocity gains
    double qrr, qrp, qry; // Rotation gains
    double qwr, qwp, qwy; // Angular Vel gains
    double  rx,  ry,  rz; // Input gains
    double  pp, pv, pr, pw; 
    double mu_MPC;        // Coeff of Fric for MPC
    double updateEveryItter;
    double useMPCTraj;
};

struct LL_params{
    // General
    double mu;      // Coeff of Fric
    double kp,kd;   // IO gains
    int useCLF;     // 0 or 1, use CLF or not  

    // Cost
    double tauPen; // input weight (cost)
    double dfPen;  // ||F-F_d|| weight (cost)
    double auxPen; // auxiliary var weight (cost)
    double clfPen; // clf defect var weight (cost)
    
    // Constraints
    double auxMax; // max aux value (constraint)
    double clfEps; // CLF convergence tuning variable
};

struct Motion_params{
    double standHeight;
    double swingHeight;
    double fwdSpeed;
    double latSpeed;
    double yawSpeed;
    double neverStopTrot;
    double extCmd;
};

};

inline void mark(int a){
    std::cout<<a<<std::endl;
};
inline void mark(int a, int b){
    std::cout<<a<<"."<<b<<std::endl;
};
inline void mark(int a, int b, int c){
    std::cout<<a<<"."<<b<<"."<<c<<std::endl;
};
inline void mark(int a, int b, int c, int d){
    std::cout<<a<<"."<<b<<"."<<c<<"."<<d<<std::endl;
};

#endif