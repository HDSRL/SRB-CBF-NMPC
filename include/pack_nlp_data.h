#ifndef PACKNLP
#define PACKNLP

#include "math_define.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using Eigen::Vector2d;
using Eigen::Vector4d;

class nlpTuple {
    public:
        nlpTuple(MatrixXd& Q_qp_, MatrixXd& f_qp_, MatrixXd& Aeq_, MatrixXd& beq_, MatrixXd& Gineq_, MatrixXd& hineq_, VectorXd x0_, Vector2d pos_, Vector4d state_other_, Vector2d obst_) 
        : Q_qp(Q_qp_), f_qp(f_qp_), Aeq(Aeq_), beq(beq_), Gineq(Gineq_), hineq(hineq_), x0(x0_), current_pos(pos_), state_other(state_other_), closest_obs(obst_) {}
        ~nlpTuple() {}

        MatrixXd Q_qp;
        MatrixXd f_qp;
        MatrixXd Aeq;
        MatrixXd beq; 
        MatrixXd Gineq;
        MatrixXd hineq;
        VectorXd x0;
        Vector4d state_other;
        Vector2d current_pos;
        Vector2d closest_obs;

    private:
};

#endif