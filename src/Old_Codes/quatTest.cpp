/************************************************************************
Copyright (c) 2020, Unitree Robotics.Co.Ltd. All rights reserved.
Use of this source code is governed by the MPL-2.0 license, see LICENSE.
************************************************************************/

#include "multi_pc_comm.h"
#include "timer.h"

#include "fstream"

using namespace UNITREE_LEGGED_SDK;

class ExternalComm
{
public:
    ExternalComm(): udpComp(LOWLEVEL){
    	 fid = fopen("/home/randy/HDSRL/quatTest.csv","w");
    }

    void Calc();

    UDP udpComp;
    FILE *fid;
    
    long motiontime = 0;
    
    double offset = 0.0;
    float gcoeffs[6] = {-0.14286f,-0.08571f,-0.02857f,0.02857f,0.08571f,0.14286f};
    double a[4] = {-9.6300211588509210e-01, 2.9253101348486945e+00, -2.9623014460856600e+00, 1.0000000000000000e+00 };
	double b[4] = {8.2160974279599230e-07, 2.4648292283879769e-06, 2.4648292283879769e-06, 8.2160974279599230e-07};
    float qHist[6][12] = {{0.0f}};
    float filtComHist[4][6] = {{0.0f}}; // last row is most recent
    float rawComHist[4][6] = {{0.0f}}; // last row is most recent
    
    LowState state = {0};
    LowCmd     cmd = {0};
    
    float dt = 0.001f;
};

void ExternalComm::Calc()
{
	udpComp.Recv();
    udpComp.GetRecv(state);
    motiontime++;
    
    printf("%lf || %lf || %lf\n",state.imu.rpy[0]*180/3.14,state.imu.rpy[1]*180/3.14,state.imu.rpy[2]*180/3.14);
    fprintf(fid,"%lf,%lf,%lf,",state.imu.rpy[0],state.imu.rpy[1],state.imu.rpy[2]);
    fprintf(fid,"%lf,%lf,%lf,%f",state.imu.quaternion[0],state.imu.quaternion[1],state.imu.quaternion[2],state.imu.quaternion[3]);
    fprintf(fid,"\n");
    
    udpComp.Send();
    udpComp.SetSend(cmd);
}
 
int main(int argc, char* argv[])
{
    ExternalComm extComm;

    LoopFunc loop_calc("calc_loop", extComm.dt, boost::bind(&ExternalComm::Calc, &extComm));
    loop_calc.start();
    
    InitEnvironment();
    extComm.udpComp.InitCmdData(extComm.cmd);

    while(1){
        sleep(10);
    };

    return 0; 
}
