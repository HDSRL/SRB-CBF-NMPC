/************************************************************************
Copyright (c) 2020, Unitree Robotics.Co.Ltd. All rights reserved.
Use of this source code is governed by the MPL-2.0 license, see LICENSE.
************************************************************************/

#include "multi_pc_comm.h"
#include "timer.h"

#include "stdio.h"
#include "fcntl.h"
#include "unistd.h"

#include "linux/input.h"
#include "sys/stat.h"

#define LOGFILE "/tmp/data"

using namespace UNITREE_LEGGED_SDK;

class ExternalComm
{
public:
    ExternalComm(): udpComp(8018, "192.168.123.161", 8090, sizeof(RobotRecv), sizeof(RobotSend)){
        fid = fopen("/home/randy/HDSRL/delaytest.csv","w");
    }
    
    FILE *fid;


    void UDPRecv();
    void UDPSend();
    void Calc();

    UDP udpComp;
    
    long motiontime = 0;
    
    LowState state = {0};
    LowCmd     cmd = {0};

    RobotRecv csend;
    RobotSend crecv;
    float dt = 0.001f;
};

void ExternalComm::UDPRecv()
{
    udpComp.Recv();
}

void ExternalComm::UDPSend()
{  
    udpComp.Send();
}

void ExternalComm::Calc()
{
    udpComp.GetRecv((char*)&crecv);

	for(int i=0; i<12; ++i){
		csend.tau[i] = 0.0f;
	}
	static int dir = 1;
	static int cnt = 0;
	if(crecv.ctrltick>=1000 && crecv.ctrltick<=2000){
		cnt++;
		if(cnt>20){
			dir*=-1;
			cnt =0;
		} 
		csend.tau[5] = dir*3.0f;
	}
	
	fprintf(fid,"%ld,%lf,%lf,%lf,",crecv.ctrltick, csend.tau[FL_2], crecv.tauEst);
	for(int i=0; i<12; ++i){
		fprintf(fid,"%lf,",crecv.q[i+6]);
	}
	for(int i=0; i<12; ++i){
		fprintf(fid,"%lf,",crecv.dq[i+6]);
	}
	fprintf(fid,"\n");

    udpComp.SetSend((char*)&csend);
}
 
int main(int argc, char* argv[]) 
{
    ExternalComm extComm;

    LoopFunc loop_calc("calc_loop",   extComm.dt,    boost::bind(&ExternalComm::Calc,    &extComm));
    LoopFunc loop_udpCompSend("udp_send", 0.001, 3, boost::bind(&ExternalComm::UDPSend, &extComm));
    LoopFunc loop_udpCompRecv("udp_recv", 0.001, 3, boost::bind(&ExternalComm::UDPRecv, &extComm));

    loop_udpCompSend.start();
    loop_udpCompRecv.start();
    loop_calc.start();
	
    while(1){
        sleep(10);
    };

    return 0; 
}
