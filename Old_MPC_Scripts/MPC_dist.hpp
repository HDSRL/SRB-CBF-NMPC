#ifndef MPCDISTRIBUTED
#define MPCDISTRIBUTED

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include "iswift_qp.h"
#include "iostream"
#include "pack_nlp_data.h"
#include "dec_vars_constr_cost.h"
#include <ifopt/problem.h>
#include <ifopt/snopt_solver.h>
#include <ifopt/ipopt_solver.h>
#include "global_loco_opts.h"
#include "global_loco_structs.hpp"
#include "math_define.h"
#include <fstream>

#define STEPLENGTHX 0.05
#define STEPLENGTHY 0.05
#define FOOTTRAJCOLNUM TOTALSTEPNUM - 2    //constant// totalstepnumber -2

typedef unsigned long size_t;

class MPC_dist 
{
private:
    //size_t locoTick = 0;
    bool isSuccess = true;
    double distance_to_fail = 0;
    double phaseVar = 0;
    double maxPhase = 1.05;
    size_t gaitTemp = STAND;
    size_t forceDomainChange = 0;
    // All the variables MPC might need
    size_t onegaitCycle_;   // how many steps in one gaitcycle: 4
    //MPC
    Eigen::MatrixXd totalFootprint_;
    Eigen::MatrixXd totalFootprintGlobal_;
    Eigen::MatrixXd totalFootprintGlobalOnes_;
    Eigen::MatrixXd footPrintTruncated_;
    Eigen::MatrixXd footPrintGlobalTruncated_;
    Eigen::MatrixXd footPrintGlobalOnesTruncated_;
    
    Eigen::MatrixXd reference_Traj_;
    Eigen::MatrixXd com_desired_Traj_;
    Eigen::MatrixXd com_desired_Traj_vec_;
    Eigen::MatrixXd com_desired_Traj_eventbased_;
    Eigen::MatrixXd com_desired_Traj_vec_eventbased_;

    Eigen::Matrix<double, 4, 8> qref;
    
    double* mpc_state_;
    Eigen::MatrixXd mpc_state_e_;
    Eigen::MatrixXd mpc_state_e_x_;
    Eigen::MatrixXd mpc_state_e_u_;

    Eigen::MatrixXd mpc_state_alpha_buffer_;
    
    double* mpc_state_eventbased_;
    Eigen::MatrixXd qp_solution_eventbased_;
    Eigen::MatrixXd mpc_state_e_eventbased_;
    Eigen::MatrixXd mpc_state_e_x_eventbased_;
    Eigen::MatrixXd mpc_state_e_u_eventbased_;

    Eigen::MatrixXd alpha_COM_traj_e_;
    Eigen::MatrixXd p_leg0_e_;
    Eigen::MatrixXd p_leg1_e_;
    Eigen::MatrixXd p_leg2_e_;
    Eigen::MatrixXd p_leg3_e_;
    
    double* alpha_COM_traj_;
    double* p_leg0_;
    double* p_leg1_;
    double* p_leg2_;
    double* p_leg3_;

    Eigen::MatrixXd Pr_;
    Eigen::MatrixXd Pr_refined_;
    Eigen::MatrixXd Prd_refined_;
    Eigen::MatrixXd Prd_;
    Eigen::MatrixXd q_;
    Eigen::MatrixXd qref_;
    Eigen::Matrix<double, 2*NUMBER_OF_AGENTS, 1> Pstart_;
    Eigen::Matrix<double, 2,NUMBER_OF_OBS> Pobs;
    Eigen::Matrix<double, 2,NUMBER_OF_OBS> Pobs_real;



    size_t gridNum_;        // grid number in domain
    size_t ts_OptTick_;     // time step number of each grid based on tick
    Eigen::MatrixXd oneCycleIndex_; //support index - support phase:0 & swing phase:1
    Eigen::MatrixXd totalCycleIndex_; // support index - support phase:0 & swing phase:1
    Eigen::MatrixXd cycleIndexTruncated_;
    size_t totalStepNum_;  

    size_t gaitDomain_;     // number of current Index column in totalCycleIndex: starting from 0th column =[1,1,1,1]

    int domain_;
    double des_yaw_;
    size_t agent_id_;

    Eigen::Vector2d agent_Initial_;

    double body_R_;
    double body_P_;
    double body_Y_;

    // For the MPC and Virtual Constraints for the COM
    Eigen::Matrix<double, 4, 1> com_start; // For the copPlanner and also serves as the starting point of the MPC
    double comTraj_[2];
    double dcomTraj_[2];
    double ddcomTraj_[2];
    double comOriTraj_[3];
    double dcomOriTraj_[3];
    double ddcomOriTraj_[3];

    // Pointers to structs
    const StateInfo* state;
    // const DynamicsInfo *dyn;
    // const KinematicsInfo *kin;
    // const ContactInfo *con;
    // const TrajInfo *traj;
    // const VCInfo *vcon;
    // const LLInfo *ll;

    // STATE INFORMATION
    double q[18] = {0};
    double dq[18] = {0};
    Eigen::Vector4d state_other; 
    int contactInd[4] = {1};
    Eigen::Matrix<double, 3, 4> toePos_;
    int log_ID = 0;
    std::fstream mpc_file;
    std::fstream qp_file;
    std::fstream obs_dist_file;
    std::fstream agent_dist_file;
public:

    bool use_snopt = false;

    MPC_dist();
    ~MPC_dist() 
    {
        if(mpc_file.is_open()){
            mpc_file.close();
        }
        if(qp_file.is_open()){
            qp_file.close();
        }
        if(obs_dist_file.is_open()){
            obs_dist_file.close();
        }
        if(agent_dist_file.is_open()){
            agent_dist_file.close();
        }
    };

    void run_NMPC();
    void footstepPlanner_eventbase(double& M, double& Ndomain, double& N, double& mu, double& swingPhase, double& row_totalFootPrintGlobal, double& col_totalFootPrintGlobal);
    void copPlanner_eventbase(double M, double Ndomain, double N, double swingPhase);
    void fitComTrajectory_eventbase(double M, double Ndomain, double N, double swingPhase);
    void plannedCycleIndex(size_t gait);
    void totalCycleIndex(size_t gait, size_t gaitCycleNum);
    void oneCycleIndex(size_t gait);
    void totalCycleIndexwHalf(size_t gait, size_t gaitCycleNum);
    void generateReferenceTrajectory();
    void setPstart(Eigen::Matrix<double, 2*NUMBER_OF_AGENTS, 1>& Pstart);
    void printPstart();
    void setPobs(Eigen::Matrix<double, 2,NUMBER_OF_OBS>& Pobs);
    void setPobs_real(Eigen::Matrix<double, 2,NUMBER_OF_OBS>& Pobs);
    void printPobs();
    void setAgentID(size_t agent_id);
    //void getComTrajectoryEventbase(size_t gait, size_t gaitDomain);
    void logMPC_Data();
    void get_MPC_Solution();
    void printSize(const Eigen::MatrixXd& mat);
    void stepSetup(double totalStepNum, double step_X, double step_Y, double body_R, double body_P, double body_Y, Eigen::MatrixXd agent_Initial);
    inline Eigen::MatrixXd get_alphaCOM() {return alpha_COM_traj_e_;}
    inline Eigen::MatrixXd get_MPCsol() {return mpc_state_e_x_eventbased_;}
    Eigen::Vector4d get_lastState();
    inline int getDomain() {return domain_; }
    void updateState(double* qin, double* dqin, int* ind, Eigen::Matrix<double, 3, 4> toePos, Eigen::Vector4d state_vec);
    void footholdsPlanner();
    void getStateOtherAgent(Eigen::Vector4d state_vec);
    MatrixXd bezier2d(double s, MatrixXd a);
    MatrixXd bezierd2d(double s, MatrixXd a);
    void updateDistance_to_fail();
    double getDistance_to_fail() {return distance_to_fail;};
    double calculateDistance(const Eigen::VectorXd& point2);
    // double dist(Vector2d qk_p, Vector2d oi);
}; 

#endif