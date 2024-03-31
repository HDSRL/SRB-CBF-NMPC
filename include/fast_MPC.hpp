//
// Authror: Randy Fawcett on 03/2021.
//
// Copyright (c) Hybrid Dynamic Systems and Robot Locomotion Lab, Virginia Tech
//

#ifndef FAST_MPC_H
#define FAST_MPC_H

#include "utils.h"
#include "A1_Dynamics.h"
#include "iswift_qp.h"
#include "stdlib.h"
#include "iostream"
#include "fstream"
#include "string"
#include "parameters.hpp"
#include "robot_model.hpp"

#define simfreq_raisim 0.001        // maybe constant
#define simfreq 0.001               // maybe changable
#define MPC_Hz 200
#define ctrlHz 1000                 // depends on simfreq
#define LL_Hz 1000                  // Low level control loop rate

#define TOTAL_DOF 18 // Total degrees of freedom for the system
#define TOTAL_IN  12 // Total system inputs

#define CTRL_HORIZ     10 // Length of the prediction horizon
#define NUM_RED_STATE  12 // Number of reduced order states
#define NUM_RED_INPUT  12 // Number of input forces to the MPC (4 legs *(x,y,z))
#define NUM_CTRL       12 // Number of actuators

#define FR_LEG 0
#define FL_LEG 1
#define RR_LEG 2
#define RL_LEG 3

#define STAND   0
#define WALK    1
#define AMBLE   2
#define TROT    3
#define BOUND   4
#define INPLACE 5
#define POSE    6
#define TAP     7
#define INPLACE_WALK 8

#define POSE_X     0
#define POSE_Y     1
#define POSE_Z     2
#define POSE_ROLL  3
#define POSE_PITCH 4
#define POSE_YAW   5
#define POSE_COMB  6
#define POSE_DEMO  7

const size_t FILE_CNT = 1;
const std::string FILE_NAMES[FILE_CNT] = {
//     "/home/kaveh/ThesisData/InplaceTrot.csv"
    //  "/home/kaveh/A1_exp_data/swing_.csv"
   "/media/kavehakbarihamed/Data/A1_RaiSim_Outputs/Test.txt"
};

class FastMPC{
    public:
        std::fstream file[FILE_CNT];
        FastMPC(int argc, char *argv[]);
        virtual ~FastMPC(){
            if(FILE_RW == true){
                for(size_t i=0; i<FILE_CNT; i++){
                    if(file[i].is_open()){
                        file[i].close();
                    }
                }
            }
            delete params;
        }

        void timeSetup(size_t stand_start, size_t loco_start);
        void posSetup(Eigen::Matrix<double,3,1> posCOM);
        void setPoseType(size_t type);

        // Robot functions
        void updateRobotState(const double q[18], const double  dq[18], const double R[9]);
        void updateState(const double q[18], const double dq[18], const double R[9]);
        void updateDynamics();
        void updateJacobian();
        void updateJacobianDot();
        void updateFwdKinematics();
        void updateSwingState();

        // Trajectory Planning
        void planTraj(size_t gait, size_t ctrlTick);
        void updateDesiredForce(Eigen::Matrix<double, 6,1> &desAcc, Eigen::MatrixXd &desForce);

        // MPC functions
        void updateMPC();
        void updateErrorState();
        void runMPC();
        void MPC_Cost(Eigen::MatrixXd &P_QP, Eigen::MatrixXd &c_QP);
        void MPC_Constraints(Eigen::MatrixXd &A_QP, Eigen::MatrixXd &b_QP, Eigen::MatrixXd &G_QP, Eigen::MatrixXd &h_QP);
        void getLinearDynamics(Eigen::MatrixXd &Ad, Eigen::MatrixXd &Bd, Eigen::MatrixXd &Dd);

        // Low-level functions
        void compute(const double q[18], const double  dq[18], const double R[9], size_t gait, size_t locoTick, double speed[3], int ctrlCon[4]);
        void impactDetection(size_t gait, size_t ctrlTick, int ctrlCon[4]);
        void computeVirtualConstraints(size_t gait, size_t ctrlTick);
        void computeTorque();
        void LL_Cost(Eigen::MatrixXd &P_QP, Eigen::MatrixXd &c_QP, size_t outDim, size_t conDim, size_t numDec, size_t useCLF);
        void LL_Constraints(Eigen::MatrixXd &A_QP, Eigen::MatrixXd &b_QP, Eigen::MatrixXd &G_QP, Eigen::MatrixXd &h_QP, size_t outDim, size_t conDim, size_t numDec, size_t useCLF);
        void torqueSat();
        void dataLog(size_t ctrlTick);

        const Eigen::Matrix<double, 18,  1>& getJointTorqueCommand(){ return jointTorque_FF_; };
        const Eigen::Matrix<double, 18,  1>& getJointPositionCommand(){ return q_; };
        const Eigen::Matrix<double, 18,  1>& getJointVelocityCommand(){ return dq_; };
        const Eigen::Matrix<double,  6,  1>& getHZDOutputs(){ return y_out_; };
        const Eigen::Matrix<double,  6,  1>& getSwingOutputs(){ return y_swing_; };
        const Eigen::Matrix<double,  6,  1>& getSwingDes(){ return hd_swing_; };
        const Eigen::Matrix<double, 18, 18>& getInertia(){ return D_; };
        const Eigen::Matrix<double, 18,  1>& getNonlinearVec(){ return H_; };
        const Eigen::Matrix<double,  3,  4>& getToePos(){ return toePos_; };
        const Eigen::Matrix<double, 12, 18>& getToeJaco(){ return toeJaco_; };
        const Eigen::Matrix<double, 12,  1>& getToeJacoDot(){ return toeJacoDot_; };
        const Eigen::Matrix<int   ,  4,  1>& getContactMatrix(){ return contactIndex_; };
        const Eigen::Matrix<int   ,  4,  1>& getDomainDes(){ return contactDes_; };
        const double & getPhaseVar(){ return phaseVar_; };
        const int & impactDetected(){ return newDom_; };
        const void setTauEst(double tauEst[12]){ for(int i=0; i<12; ++i){tauEst_(i)=tauEst[i];} return; };
        const void setTauPrev(double tauPrev[12]){ for(int i=0; i<12; ++i){tau_prev_(i)=tauPrev[i];} return; };
        

    private:
        // Parameters
        double mu_;
        double mu_MPC_;
        double Ts_opt_;
        double mass_;
        double standHeight_;
        double startHeight_;
        double startx_;
        double finalX_;
        double finalY_;
        double h_sw_;
        double pitch_;
        Eigen::Matrix<double, 3, 1> stepLength_;
        Eigen::Matrix<double, 3, 3> inertia_;

        Parameters *params;

        // MPC variables
        Eigen::Matrix<double, NUM_RED_STATE, 1> redDes_;
        Eigen::Matrix<double, NUM_RED_STATE, 1> desLock_;
        Eigen::Matrix<double, NUM_RED_STATE, 1> redCurrent_;
        Eigen::Matrix<double, NUM_RED_STATE, 1> trajMPC_;
        Eigen::MatrixXd forceMPC_;
        Eigen::MatrixXd forceDes_;

        // Contact variables
        Eigen::Matrix<int, 4, 1> contactIndex_;
        Eigen::Matrix<int, 4, 1> contactDes_;
        int numContact_;

        // Low-level variables
        double* optimOut_;
        Eigen::Matrix<double, TOTAL_DOF, 1>jointTorque_FF_;
        Eigen::Matrix<double, 12, 1> tauEst_;
        Eigen::Matrix<double, TOTAL_DOF, 1>q_;
        Eigen::MatrixXd qsw_;
        Eigen::Matrix<double, TOTAL_DOF, 1>dq_;
        Eigen::MatrixXd dqsw_;
        Eigen::Matrix<double, TOTAL_DOF, 1>ddq_;
        Eigen::Matrix<double,3,4> initToePos_;
        Eigen::Matrix<double, 12, 2> footTraj_;
        Eigen::Matrix<double, 12, 1> comDes_;
        Eigen::MatrixXd h0_, dh0_;
        Eigen::MatrixXd H0_, dH0_;
        Eigen::MatrixXd hd_, dhd_, ddhd_;
        Eigen::MatrixXd y_, dy_;
        Eigen::Matrix<double, 6, 1> y_out_, y_swing_, hd_swing_;
        double kpGain_, kdGain_;
        double V_;
        double Veps_;
        double LfV_;
        Eigen::MatrixXd LgV_;
        double dV_;
        Eigen::MatrixXd QP_force_;


        // Quad parameters
        Eigen::Matrix<double, 18, 18> D_;
		Eigen::Matrix<double, 18, 18> Dinv_;
        Eigen::Matrix<double, 18, 12> B_;
        Eigen::Matrix<double, 18,  1> H_;
        Eigen::Matrix<double, 18,  1> jointPosE_;
        Eigen::Matrix<double, 18,  1> jointVel_;
        Eigen::Matrix<double,  3,  3> R_;
        Eigen::Matrix<double, 18,  1> jointPosE_prev_;
        Eigen::Matrix<double, 18,  1> jointVel_prev_;
        Eigen::Matrix<double, 12,  1> tau_prev_;
        Eigen::Matrix<double,  3,  4> toePos_;
        Eigen::Matrix<double, 12, 18> toeJaco_;
        Eigen::Matrix<double, 12,  1> toeJacoDot_;
        Eigen::Matrix<double,  3,  4> hipPos_;
        Eigen::Matrix<double, 12, 18> hipJaco_;
        Eigen::Matrix<double, 12,  1> hipJacoDot_;
        Eigen::Matrix<double,  3,  1> sat_;
        Eigen::Matrix<double,  3,  1> toeOffset_;
        Eigen::Matrix<double,  3,  4> hipOffset_;
        Eigen::Matrix<double,  3,  1> comFiltered_;
        Eigen::Matrix<double,  3,  300> comHist_;

        Eigen::MatrixXd Jc_, dJc_, Js_, dJs_;

        // Controller parameters
        size_t gait_;
        size_t standStart_;
        size_t locoStart_;
        size_t poseType_;
        size_t locoTick_;
        double tickScale_;
        double phaseVar_;
        bool impact_;
        int newDom_;
        double domainLength_;

        bool FILE_RW;
};




#endif
