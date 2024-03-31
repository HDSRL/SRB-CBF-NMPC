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
    	 fid = fopen("/home/randy/HDSRL/directDelayTest.csv","w");
    }

    void Calc();

    UDP udpComp;
    FILE *fid;
    
    long motiontime = 0;
    
    LowState state = {0};
    LowCmd     cmd = {0};
    
    float dt = 0.001f;
};

void ExternalComm::Calc()
{
	udpComp.Recv();
    udpComp.GetRecv(state);
    motiontime+=dt*1000;
    
    for(int i=0; i<12; ++i){
		cmd.motorCmd[i].tau = 0.0f;
	}
	static int dir = 1;
	static int cnt = 0;
	if(motiontime>=1000 && motiontime<=2000){
		cnt++;
		if(cnt>20){
			dir*=-1;
			cnt =0;
		} 
		cmd.motorCmd[5].tau = dir*3.0f;
	}
	
	fprintf(fid,"%ld,%lf,%lf,",motiontime, cmd.motorCmd[FL_2].tau, state.motorState[FL_2].tauEst);
	for(int i=0; i<12; ++i){
		fprintf(fid,"%lf,",state.motorState[i+6].q);
	}
	for(int i=0; i<12; ++i){
		fprintf(fid,"%lf,",state.motorState[i+6].dq);
	}
	fprintf(fid,"\n");
    
    udpComp.SetSend(cmd);
    udpComp.Send();
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
