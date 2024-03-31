#include "LocoWrapper.hpp"
#include "iostream"

int main(int argc, char *argv[]){

    double q[18] = {0};
    double dq[18] = {0};
    double R[9] = {0};

    LocoWrapper loco_obj(argc, argv);
    double force[4] = {500};
    loco_obj.calcTau(q,dq,R,force,TROT,0);

    std::cout<<"success"<<std::endl;
    return 0;
}