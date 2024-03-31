
#include "forceUpdate.hpp"

void HighLevel::updateDesiredForce(Eigen::Matrix<double, 6,1> &desAcc, Eigen::MatrixXd &desForce, sharedData *info){
	double mass_ = 12.453;
    Eigen::Matrix<double, 3, 3> inertia;
    inertia  << 0.01683993,   8.3902e-5, 0.000597679,
                 8.3902e-5, 0.056579028,   2.5134e-5,
               0.000597679,   2.5134e-5, 0.064713601;

    // Parse desired vector
    Eigen::Vector3d pd = info->comDes.block(0,0,3,1);
    Eigen::Vector3d wd = info->comDes.block(9,0,3,1);

    // Setup
    Eigen::Vector3d rd;
    Eigen::Matrix3d rd_hat_temp;
    Eigen::MatrixXd rd_hat(3,12);
    for(int i=0;i<4;i++){
        rd = info->toePos.block(0,i,3,1)-pd;
        hatmap(rd,rd_hat_temp);
        rd_hat.block(0,3*i,3,3) = rd_hat_temp;
    }

    // Calculate wd_hat
    Eigen::Matrix3d wd_hat;
    hatmap(wd, wd_hat);

    Eigen::MatrixXd H, b;
    Eigen::Vector3d g;
    Eigen::MatrixXd force(3,12);
    Eigen::MatrixXd torque(3,12);
    g << 0,0,9.81;
    H.setZero(6,12);
    b.setZero(6,1);


    // NOTE: all columns of non-contacting legs are set to zero
    force.setZero();
    torque.setZero();
    for(int i=0; i<4; i++){
        if(info->ind[i]==1){
            force.block(0,3*i,3,3) = Eigen::MatrixXd::Identity(3,3);
            torque.block(0,3*i,3,3) = rd_hat.block(0,3*i,3,3);
        }
    }
    H << force, torque;
    b.block(0,0,3,1) = mass_*(desAcc.block(0,0,3,1) + g);
    b.block(3,0,3,1) = inertia*desAcc.block(3,0,3,1) + wd_hat*inertia*wd;

    double* optimVec = new double[12];
    Eigen::Matrix<double, 12, 12> P_QP;
    Eigen::Matrix<double, 12,  1> c_QP;
    Eigen::Matrix<double,  1, 12> A_QP;
    Eigen::Matrix<double,  1,  1> b_QP;
    Eigen::MatrixXd G_QP(5*info->cnt,12);
    Eigen::MatrixXd h_QP(5*info->cnt,1);
    P_QP = H.transpose()*H;
    c_QP = -H.transpose()*b;
    A_QP.setZero();
    b_QP.setZero();

    // Eigen::MatrixXd Gc(20,12); // contact constraints
    Eigen::Matrix<double, 5, 3> gc;
    G_QP.setZero();
    double mu = 0.7;
    gc << 1,  0, -mu/sqrt(2),
         -1,  0, -mu/sqrt(2),
          0,  1, -mu/sqrt(2),
          0, -1, -mu/sqrt(2),
          0,  0,          -1;

    size_t cnt = 0;
    for(size_t i=0; i<4; i++){
        if (info->ind[i]==1){
            G_QP.block(5*cnt,3*i,5,3) = gc;
            cnt++;
        }
    }
    h_QP.setZero();

    iswiftQp_e(P_QP, c_QP, A_QP, b_QP, G_QP, h_QP, optimVec);
    desForce.setZero(12,1);
    for(size_t i=0; i<12; i++){
        desForce(i) = optimVec[i];
    }
    delete[] optimVec;
}
