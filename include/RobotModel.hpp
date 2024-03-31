#ifndef QUADRUPED_MODEL
#define QUADRUPED_MODEL

#include "global_loco_structs.hpp"
#include "A1_Dynamics.h"     
#include "Transforms.hpp"

#define HIST_LEN 50        // Length of moving average window (in samples, not time) for comFiltered

class RobotModel
{
public:
    RobotModel();
    virtual ~RobotModel(){};

    void updateState(const double q[18], const double dq[18], const double R[9]);
    void updateDynamics();
    void updateJacobian();
    void updateJacobianDot();
    void updateFwdKinematics();
    void updateSwingMatrices(const int conInd[4], const int &numCon);
    const StateInfo *getStatePointer() { return &state; };
    const DynamicsInfo *getDynamicsPointer() { return &dyn; };
    const KinematicsInfo *getKinematicsPointer() { return &kin; };

private:
    StateInfo state;
    DynamicsInfo dyn;
    KinematicsInfo kin;

    Eigen::Matrix<double, 3, HIST_LEN> comHist; // Circular array for efficiency
    int histInd = 0;
};

#endif
