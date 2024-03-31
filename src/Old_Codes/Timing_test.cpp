#include "fast_MPC.hpp"
#include "timer.h"
#include <iostream>

int main(int argc, char *argv[]){
    timer tset;
    FastMPC* loco = new FastMPC(argc, argv);


    Eigen::Matrix<double, 18, 1> q,dq,tau;
    Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
    q.setOnes();
    dq.setOnes();

    for(int i=0; i<1000; ++i){
        tic(&tset);
        loco->compute(q.data(),dq.data(),R.data(),0,0);
        tau = loco->getJointTorqueCommand();
        std::cout<< toc(&tset) << std::endl;
    }

}