#ifndef PLANNER_HPP
#define PLANNER_HPP

#include "global_loco_structs.hpp"
#include "ContactEst.hpp"
#include "Transforms.hpp"
#include "Bezier.h"
#define MAX_SL_F_X 0.16     // Max forward step length (magnitude)
#define MAX_SL_R_X 0.16     // Max backward step length (magnitude)
#define MAX_SL_Y 0.12       // Max lateral step length (magnitude)  

using MP = Settings::Motion_params;

class MotionPlanner
{
public:
    MotionPlanner();
    virtual ~MotionPlanner(){};

    void planTraj(const StateInfo *state, const KinematicsInfo *kin, ContactEst *con_obj, size_t gait, double phase, size_t ctrlTick, MP *params, const Eigen::MatrixXd & Pr_refined_, size_t agent_id_, size_t gaitDomain_, Eigen::MatrixXd com_des_traj);
    void updateStandVars(const Eigen::Matrix<double,3,1> &com, double timeToStand);
    void updateVel(Eigen::Matrix<double,3,1> &desVel, Eigen::Matrix<double, 3, 1> &desOmega, MP *params);
    void printSize(const Eigen::MatrixXd& mat);
    void setComDes(const Eigen::Vector4d& comTraj); // comTraj -> x, y, dx, dy  
    
    const TrajInfo* getTrajInfoPointer(){return &traj;};

protected:
    inline void setStepLen(double x, double y, double z){
        if (abs(y) > 0.09)
        {
            y = 0.6*y;
        }
        if (abs(y) > 0.09)
        {
            y = 0.5*y;
        }
        if (abs(x) > 0.12)
        {
            x = 0.6*x;
        }
        if (abs(x) > 0.12)
        {
            x = 0.6*x;
        }
        
        traj.stepLen[0] = x; traj.stepLen[1] = y; traj.stepLen[2] = z;
    }

private:
    double x0;
    double xf;
    double y0;
    double yf;
    double z0;
    double standTime;
    TrajInfo traj;
    size_t poseType = POSE_Z;
};

#endif
