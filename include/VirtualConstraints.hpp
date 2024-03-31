#ifndef VIRTUALCONSTRAINTS
#define VIRTUALCONSTRAINTS

#include "global_loco_structs.hpp"
#include "Bezier.h"

using KinInf = KinematicsInfo;
using ConInf = ContactInfo;
using MP = Settings::Motion_params;

class VirtualConstraints{
public:
    VirtualConstraints();
    virtual ~VirtualConstraints(){};

    void updateVirtualConstraints(const StateInfo *state, const KinInf *kin, const TrajInfo *traj, const ConInf *con, size_t gait, double phaseVar, MP *params, const LLInfo *ll);
    void updateTime();
    void setDesiredForce(Eigen::Matrix<double, 12, 1> fDes){VC.fDes = fDes;};
    const VCInfo* getVCPointer(){return &VC;};

private:
    VCInfo VC; // Needed by the low-level controller
    double h_sw = 0.08;
    Eigen::MatrixXd h0, dh0;
};




#endif
