#ifndef LOCO_WRAPPER
#define LOCO_WRAPPER

#include "global_loco_structs.hpp"
#include "Parameters.hpp"
#include "RobotModel.hpp"
#include "LowLevelCtrl.hpp"
#include "VirtualConstraints.hpp"
#include "ContactEst.hpp"
#include "MotionPlanner.hpp"
#include "DataLog.hpp"
#include <memory>
#include "math_define.h"
#include <string.h>

/////////////////////////////////////////////////////////////
// define variables
#define debg 0
#define EVENTBASE       1                   //changable// event base MPC on(1) / off(0)
#define OFFLINE_MATLAB  0                   //changable // First have to compare the variable setup in matlab
#define STEPLENGTHX     0.1                //changable//
#define STEPLENGTHY     0.00                //changable//
#define STEPHEIGHT      0.08                //changable
//#define COMROWNUM       TOTALSTEPNUM * 4    //constant// totalstepnumber * 4
#define BODYROLL        0.0                 //changable//
#define BODYPITCH       0.0                 //changable//
#define BODYYAW         0.000                 //changable//

#define FOOTTRAJCOLNUM  TOTALSTEPNUM - 2    //constant// totalstepnumber -2
#define START_X
#define START_Y

class LocoWrapper : public Parameters
{
public:
    LocoWrapper(int argc, char *argv[]);
    virtual ~LocoWrapper();

    void calcTau(const double q[18], const double dq[18], const double R[9], const int force[4], size_t gait, size_t ctrlTick, bool* checkrunMPC);
    double* getTorque(){return LL->getTorque();};
    const int* getConDes(){return con->des;};
    Eigen::Matrix<double, 18, 1> getJointPosCmd(){return ll->q;};
    Eigen::Matrix<double, 18, 1> getJointVelCmd(){return ll->dq;};
    void initStandVars(Eigen::Matrix<double,3,1> com, double standTime){ PP->updateStandVars(com,standTime);};
    void updateDesiredForce(Eigen::Matrix<double, 12, 1> fDes){VC->setDesiredForce(fDes);};

    // From Basit's Code
    void lipMPC_eventbase();
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
    void printPobs();
    void setAgentID(size_t agent_id);
    void getComTrajectoryEventbase(size_t gait, size_t gaitDomain);
    void logMPC_Data();
    void get_MPC_Solution();
    void printSize(const Eigen::MatrixXd& mat);
    void stepSetup(double totalStepNum, double step_X, double step_Y, double body_R, double body_P, double body_Y, Eigen::MatrixXd agent_Initial);
    void set_MPC_DATA(Eigen::MatrixXd alpha_COM, Eigen::MatrixXd MPC_sol, bool avail);
    void reset_MPC_DATA();
    Eigen::MatrixXd get_footPrintMtx() { return footPrintTruncated_; }
    int getDomain() {return domain_; }
    bool get_MPC_data_available() {return MPC_data_available;}
    Eigen::Matrix<double, 3, 4> get_toePos() {return kin->toePos;}

    // Pointers to structs
    const StateInfo *state;
    const DynamicsInfo *dyn;
    const KinematicsInfo *kin;
    const ContactInfo *con;
    const TrajInfo *traj;
    const VCInfo *vcon;
    const LLInfo *ll;
    

    // RUN MPC?
    bool runMPC = 1;

    

private:
    // size_t newDom = 0;
    size_t locoTick = 0;
    double phaseVar = 0;
    double maxPhase = 1.05;
    size_t gaitTemp = STAND;
    size_t forceDomainChange = 0;   
    int log_ID = 0; 

    // Pointers to class objects
    std::unique_ptr<DataLog> data;
    RobotModel *quad;
    LowLevelCtrl *LL;
    VirtualConstraints *VC;
    ContactEst *conEst;
    MotionPlanner *PP;

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
    bool MPC_data_available = 0;
    bool runMPC_now = 0;
};

inline double getPhase(double time, double time_0, double time_f){
    return (1.0*time-1.0*time_0)/(1.0*time_f-1.0*time_0);
};

#endif
