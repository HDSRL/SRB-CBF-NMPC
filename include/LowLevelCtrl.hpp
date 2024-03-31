#ifndef LOWLEVELCONTROLLER
#define LOWLEVELCONTROLLER

#include "global_loco_structs.hpp"
#include "EigenUtils.hpp"
#include "Transforms.hpp"
#include "iswift_qp.h"

using DynInf = DynamicsInfo;
using KinInf = KinematicsInfo;
using ConInf = ContactInfo;
using LLP = Settings::LL_params;

class LowLevelCtrl
{
public:
    LowLevelCtrl();
    virtual ~LowLevelCtrl(){};

    void calcTorque(const StateInfo *state, const DynInf *dyn, const KinInf *kin, const VCInfo *vc, const ConInf *con, LLP *params);
    void calcTorque_2(const StateInfo *state, const DynInf *dyn, const KinInf *kin, const VCInfo *vc, const ConInf *con, LLP *params);
    const LLInfo* getllPointer(){return &ll;};
    double* getTorque(){return tau;};

private:

    void cost(LLP *params, const VCInfo *vc, const ConInf *con, size_t &outDim, size_t &conDim, size_t &numDec, size_t &useCLF);
    void constraints(LLP *params, const DynInf *dyn, const KinInf *kin, const VCInfo *vc, const ConInf *con, size_t &outDim, size_t &conDim, size_t &numDec, size_t &useCLF);
    void cost_2(LLP *params, const DynInf *dyn, const KinInf *kin, const VCInfo *vc, const ConInf *con, size_t &outDim, size_t &conDim, size_t &numDec, size_t &useCLF);
    void constraints_2(LLP *params, const DynInf *dyn, const KinInf *kin, const VCInfo *vc, const ConInf *con, size_t &outDim, size_t &conDim, size_t &numDec, size_t &useCLF);
    void swingInvKin(const StateInfo *state, const DynInf *dyn, const KinInf *kin, const VCInfo *vc, const ConInf *con, LLP *params);
    void saturateTorque();

    LLInfo ll;

    Eigen::Matrix<double, 3, 1> sat = {22,50,50};

    double V;
    double Veps;
    double LfV;
    Eigen::MatrixXd LgV;
    double dV;

    double tau[6+TOTAL_IN] = {0};
    double *optimOut;

    Eigen::Matrix<double, 31, 31> P_QP; // Each QP matrix is set to the max size for any domain
    Eigen::Matrix<double, 31,  1> c_QP;
    Eigen::Matrix<double, 18, 31> A_QP;
    Eigen::Matrix<double, 18,  1> b_QP;
    Eigen::Matrix<double, 45, 31> G_QP;
    Eigen::Matrix<double, 45,  1> h_QP;
};

#endif