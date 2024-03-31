#include "fast_MPC.hpp"
#include <iostream>
#include <chrono>
#include "optimization/iSWIFT/include/timer.h"

int main(int argc, char *argv[]){

    

    FastMPC* loco_obj = new FastMPC(argc,argv);

    Eigen::Matrix<double, 19, 1> q,dq;
    Eigen::Matrix<double, 18, 1> tau;
    Eigen::Matrix3d R;
    R.setIdentity();
    q.setOnes();
    dq.setOnes(); 

    size_t gait = STAND;
    size_t tick = 0;

    loco_obj->compute(q,dq,R,gait,tick);
    tau = loco_obj->getJointTorqueCommand();


    std::cout<< toc(&timer) <<std::endl;



    return 0;
}