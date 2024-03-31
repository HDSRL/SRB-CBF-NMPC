/************************************************************************
Copyright (c) 2020, Unitree Robotics.Co.Ltd. All rights reserved.
Use of this source code is governed by the MPL-2.0 license, see LICENSE.
************************************************************************/

#include <math.h>
#include "multi_pc_comm.h"
#include "A1_Dynamics.h"
#include "kalman.hpp"
#include <iostream>
#include <fstream>
#include "stdio.h"

#include "fcntl.h"
#include "unistd.h"

#include "linux/input.h"
#include "sys/stat.h"

using namespace UNITREE_LEGGED_SDK;

class ExternalComm
{
public:
    ExternalComm(): control(LeggedType::A1, LOWLEVEL), udpComp(8090, "192.168.123.160", 8018, sizeof(RobotSend), sizeof(RobotRecv)), udpRobot(){
    	control.InitCmdData(cmd);
    	fid = fopen("/home/unitree/Two_PC_Code/test.csv","w");
    }
    //ExternalComm(): control(LeggedType::A1, LOWLEVEL), udpRobot(){control.InitCmdData(cmd);}
    
    void UDPCompRecv();
    void UDPCompSend();
    void UDPRobotSend();
    void UDPRobotRecv();
    void Calc();

	Control control;
	KF* est_obj;

    UDP udpComp;
    UDP udpRobot;
    
    FILE *fid;
    
    long motiontime = 0;
    float qInit[12]={0};
    bool beginCommand = false;
    
    float kp, kd;
    
    double offset = 0.0;
    float gcoeffs[6] = {-0.14286f,-0.08571f,-0.02857f,0.02857f,0.08571f,0.14286f};
    double a[4] = {-9.6300211588509210e-01, 2.9253101348486945e+00, -2.9623014460856600e+00, 1.0000000000000000e+00 };
	double b[4] = {8.2160974279599230e-07, 2.4648292283879769e-06, 2.4648292283879769e-06, 8.2160974279599230e-07};
    float qHist[6][12] = {{0.0f}};
    float filtComHist[4][6] = {{0.0f}}; // last row is most recent
    float rawComHist[4][6] = {{0.0f}}; // last row is most recent
    
    LowState state = {0};
    LowCmd     cmd = {0};    
    
    RobotRecv rrecv;
    RobotSend rsend;
    float dt = 0.001f;
};

void ExternalComm::UDPCompRecv()
{
    udpComp.Recv();
}

void ExternalComm::UDPCompSend()
{  
    udpComp.Send();
}

void ExternalComm::UDPRobotSend(){
	udpRobot.Send();
}

void ExternalComm::UDPRobotRecv(){
	udpRobot.Recv();
}

double jointLinearInterpolation(double initPos, double targetPos, double rate)
{
    double p;
    rate = std::min(std::max(rate, 0.0), 1.0);
    p = initPos*(1-rate) + targetPos*rate;
    return p;
}

void ExternalComm::Calc()
{
	motiontime += 1;
	int standtime = 2000;
    int starttime = 1000;

	udpRobot.GetRecv(state);
    udpComp.GetRecv((char*)&rrecv);
    
    // Pack the data for the estimator
    int thresh = 10;
    double relVec[12] = {0};
    int contactIndex[4] = {0};
    contactIndex[0] = state.footForce[0]>thresh ? 1 : 0;
    contactIndex[1] = state.footForce[1]>thresh ? 1 : 0;
    contactIndex[2] = state.footForce[2]>thresh ? 1 : 0;
    contactIndex[3] = state.footForce[3]>thresh ? 1 : 0;
    if(motiontime>2000){
		double fr_toe[3], fl_toe[3], rl_toe[3], rr_toe[3];
    	FK_FR_toe(fr_toe, rsend.q); FK_FL_toe(fl_toe, rsend.q);
		FK_RR_toe(rr_toe, rsend.q); FK_RL_toe(rl_toe, rsend.q);
		
		relVec[0] = -1*fr_toe[0]-rsend.q[0];
		relVec[1] = -1*fr_toe[1]-rsend.q[1];
		relVec[2] = -1*(fr_toe[2]-rsend.q[2]);
		
		relVec[3] = -1*fl_toe[0]-rsend.q[0];
		relVec[4] = -1*fl_toe[1]-rsend.q[1];
		relVec[5] = -1*(fl_toe[2]-rsend.q[2]);
		 
		relVec[6] = -1*rr_toe[0]-rsend.q[0];
		relVec[7] = -1*rr_toe[1]-rsend.q[1];
		relVec[8] = -1*(rr_toe[2]-rsend.q[2]);
		
		relVec[9] = -1*rl_toe[0]-rsend.q[0];
		relVec[10] = -1*rl_toe[1]-rsend.q[1];
		relVec[11] = -1*(rl_toe[2]-rsend.q[2]);
    } else {
    	relVec[0] = 0.183f;  relVec[1]  = -0.132f; relVec[2]  = 0.08f;
		relVec[3] = 0.183f;  relVec[4]  = 0.132f;  relVec[5]  = 0.08f;
		relVec[6] = -0.183f; relVec[7]  = -0.132f; relVec[8]  = 0.08f;
		relVec[9] = -0.183f; relVec[10] = 0.132f;  relVec[11] = 0.08f;
		
		contactIndex[0] = 1; contactIndex[1] = 1;
		contactIndex[2] = 1; contactIndex[3] = 1;
    	
    	rsend.q[2] = 0.08f;
    }
    
    // Run the estimator
    est_obj->updateKalman(contactIndex,state.imu.accelerometer,state.imu.quaternion,relVec);
    double* comPosVel;
    double* Rot;
    comPosVel = est_obj->getComPosVel();
    Rot = est_obj->getRotMat();
    
    // Pack the state with joint position and velocity
    rsend.ctrltick = motiontime;
    for(int i=0; i<12; ++i){
    	// Grab "raw" joint position
    	rsend.q[i+6] = state.motorState[i].q;
    	
    	// Update q history and calculate filtered dq (6th order Golay)
    	rsend.dq[i+6] = 0.0;
    	for(int j=0; j<5; ++j){
    		qHist[j][i] = qHist[j+1][i];
    		rsend.dq[i+6] += qHist[j][i]*gcoeffs[j];
    	}
    	qHist[5][i] = state.motorState[i].q;
    	rsend.dq[i+6] += qHist[5][i]*gcoeffs[5];
    	rsend.dq[i+6] /= dt;
    }
    double temp[6] = {0.0};
    for(int i=0; i<6; ++i){
    	for(int j=0; j<3; ++j){
    		rawComHist[j][i] = rawComHist[j+1][i];
    		filtComHist[j][i] = filtComHist[j+1][i];
    		temp[i] += rawComHist[j][i]*b[j] - filtComHist[j][i]*a[j];
    	}
    	rawComHist[3][i] = comPosVel[i];
    	temp[i] += rawComHist[3][i]*b[3];
    	filtComHist[3][i] = temp[i];
    	printf("%lf || ",temp[i]);
    }
    printf("\n");
    
    
    // Estimator
    double fr_toe[3], fl_toe[3], rl_toe[3], rr_toe[3];
    double Jfr_toe[54], Jfl_toe[54], Jrl_toe[54], Jrr_toe[54];
	rsend.q[0] = 0;rsend.q[1] = 0;rsend.q[2] = 0;
	rsend.q[3] = 0;rsend.q[4] = 0;rsend.q[5] = 0;
	FK_FR_toe(fr_toe, rsend.q); FK_FL_toe(fl_toe, rsend.q);
	FK_RR_toe(rr_toe, rsend.q); FK_RL_toe(rl_toe, rsend.q);
	
	rsend.dq[0] = 0;rsend.dq[1] = 0;rsend.dq[2] = 0;
	rsend.dq[3] = 0;rsend.dq[4] = 0;rsend.dq[5] = 0;
	J_FR_toe(Jfr_toe, rsend.q); J_FL_toe(Jfl_toe, rsend.q);
	J_RR_toe(Jrr_toe, rsend.q); J_RL_toe(Jrl_toe, rsend.q);
	
	//for(int i=6; i<18; ++i){
		//rsend.dq[0] -= ( Jfr_toe[3*i] + Jfl_toe[3*i] + Jfl_toe[3*i] + Jfl_toe[3*i] )*rsend.dq[i];
		//rsend.dq[1] -= ( Jfr_toe[3*i+1] + Jfl_toe[3*i+1] + Jfl_toe[3*i+1] + Jfl_toe[3*i+1] )*rsend.dq[i];
		//rsend.dq[2] -= ( Jfr_toe[3*i+2] + Jfl_toe[3*i+2] + Jfl_toe[3*i+2] + Jfl_toe[3*i+2] )*rsend.dq[i];
	//}
	
	if(beginCommand){
		for(int i=0; i<12; ++i){
			//fprintf(fid,"%lf,",rrecv.tau[i]);
		}
		for(int i=0; i<12; ++i){
			//fprintf(fid,"%lf,",state.motorState[i].tauEst);
		}
		for(int i=0; i<12; ++i){
			//fprintf(fid,"%lf,",rsend.q[i+6]);
		}
		for(int i=0; i<12; ++i){
			//fprintf(fid,"%lf,",rsend.dq[i+6]);
		}
		for(int i=0; i<3; ++i){
			//fprintf(fid,"%lf,",rsend.dq[i]);
		}
		//fprintf(fid,"\n");
	}
	//rsend.q[0] = -0.183-(rr_toe[0]+rl_toe[0])/2;
    //rsend.q[1] = 0.132-(fl_toe[1]+rl_toe[1])/2;
    //rsend.q[2] = -1.0*(fr_toe[2]+fl_toe[2]+rr_toe[2]+rl_toe[2])/4;
    
    rsend.q[0] = filtComHist[3][0];
    rsend.q[1] = filtComHist[3][1];
    rsend.q[2] = filtComHist[3][2];
    rsend.q[3] = 1.0*state.imu.rpy[0];
    rsend.q[4] = 1.0*state.imu.rpy[1];
    rsend.q[5] = 1.0*state.imu.rpy[2]-offset; // Use initial value as offset
    
    rsend.dq[0] = filtComHist[3][3];
    rsend.dq[1] = filtComHist[3][4];
    rsend.dq[2] = filtComHist[3][5];
    
    for(int i=0; i<6; ++i){
    	//printf("%lf || ", rsend.q[i]);
    }
    //printf("\n");
    
    // Standup Routine
    if(motiontime <= starttime){
    	for(int i=0;i<12;++i){
    		qInit[i] = state.motorState[i].q;
    	}
    }  
    double currenttime = 1.0*(motiontime-starttime)/standtime;  
    if( (motiontime>=starttime)  && (motiontime<=(starttime+standtime)) ){
		for(int i=0; i<4; ++i){
			cmd.motorCmd[i*3].q = jointLinearInterpolation(qInit[3*i],0,currenttime);
			cmd.motorCmd[i*3+1].q = jointLinearInterpolation(qInit[3*i+1],M_PI/4,currenttime);
			cmd.motorCmd[i*3+2].q = jointLinearInterpolation(qInit[3*i+2],-1.0*M_PI/2,currenttime);
			
			cmd.motorCmd[i*3].dq = 0.0;
			cmd.motorCmd[i*3+1].dq = 0.0;
			cmd.motorCmd[i*3+2].dq = 0.0;
			
			cmd.motorCmd[i*3].Kp = 180.0f;
			cmd.motorCmd[i*3+1].Kp = 150.0f;
			cmd.motorCmd[i*3+2].Kp = 150.0f;

			cmd.motorCmd[i*3].Kd = 12.0f;
			cmd.motorCmd[i*3+1].Kd = 12.0f;
			cmd.motorCmd[i*3+2].Kd = 12.0f;
			
			cmd.motorCmd[i*3].tau = 0.0f;
			cmd.motorCmd[i*3+1].tau = 0.0f;
			cmd.motorCmd[i*3+2].tau = 0.0f;
		}
    }else if( (motiontime<starttime) && (motiontime<(starttime+standtime)) ) {
    	for(int i=0; i<4; ++i){
			cmd.motorCmd[i*3].q = 0.0f;
			cmd.motorCmd[i*3+1].q = 0.0f;
			cmd.motorCmd[i*3+2].q = 0.0f;
			
			cmd.motorCmd[i*3].dq = 0.0f;
			cmd.motorCmd[i*3+1].dq = 0.0f;
			cmd.motorCmd[i*3+2].dq = 0.0f;
			
			cmd.motorCmd[i*3].Kp = 0.0f;
			cmd.motorCmd[i*3+1].Kp = 0.0f;
			cmd.motorCmd[i*3+2].Kp = 0.0f;
			
			cmd.motorCmd[i*3].Kd = 0.0f;
			cmd.motorCmd[i*3+1].Kd = 0.0f;
			cmd.motorCmd[i*3+2].Kd = 0.0f;
			
			cmd.motorCmd[i*3].tau = 0.0f;
			cmd.motorCmd[i*3+1].tau = 0.0f;
			cmd.motorCmd[i*3+2].tau = 0.0f;
		}
    } else if( (motiontime>(starttime+standtime)) ){
    	rsend.begin = 1;
    	rsend.Rot[0] = 1;
    	rsend.Rot[4] = 1;
    	rsend.Rot[8] = 1;
    	
    	if(beginCommand){
    		rsend.streaming = 1;
    	
			// Set the torque
			float hr_max = 2.0f;
			float hr_min = -2.0f;
			float hp_max = 14.0f;
			float hp_min = -2.2f;
			float k_max = 14.0f;
			float k_min = -2.2f;
			for(int i = 0; i<4; i++){
				rrecv.tau[3*i] = (rrecv.tau[3*i]>hr_max) ? hr_max : rrecv.tau[3*i];
				rrecv.tau[3*i] = (rrecv.tau[3*i]<hr_min) ? hr_min : rrecv.tau[3*i];
				rrecv.tau[3*i+1] = (rrecv.tau[3*i+1]>hp_max) ? hp_max : rrecv.tau[3*i+1];
				rrecv.tau[3*i+1] = (rrecv.tau[3*i+1]<hp_min) ? hp_min : rrecv.tau[3*i+1];
				rrecv.tau[3*i+2] = (rrecv.tau[3*i+2]>k_max) ? k_max : rrecv.tau[3*i+2];
				rrecv.tau[3*i+2] = (rrecv.tau[3*i+2]>k_max) ? k_max : rrecv.tau[3*i+2];
			
				cmd.motorCmd[3*i].tau   = rrecv.tau[3*i];
				cmd.motorCmd[3*i+1].tau = rrecv.tau[3*i+1];
				cmd.motorCmd[3*i+2].tau = rrecv.tau[3*i+2];

				cmd.motorCmd[3*i].q   = rrecv.pos[3*i];
				cmd.motorCmd[3*i+1].q = rrecv.pos[3*i+1];
				cmd.motorCmd[3*i+2].q = rrecv.pos[3*i+2];
			
				cmd.motorCmd[3*i].dq   = rrecv.vel[3*i];
				cmd.motorCmd[3*i+1].dq = rrecv.vel[3*i+1];
				cmd.motorCmd[3*i+2].dq = rrecv.vel[3*i+2];
				
				cmd.motorCmd[i*3].Kp   = kp;
				cmd.motorCmd[i*3+1].Kp = kp;
				cmd.motorCmd[i*3+2].Kp = kp;
			
				cmd.motorCmd[i*3].Kd   = kd;
				cmd.motorCmd[i*3+1].Kd = kd;
				cmd.motorCmd[i*3+2].Kd = kd;
			}
			
		}else {
			printf("Press Space to Continue\n");
			offset = state.imu.rpy[2];
		}
    }
    

    udpComp.SetSend((char*)&rsend);
    udpRobot.SetSend(cmd);
}

int main(void) 
{
	struct input_event ev;
    int k = 1;
    int fd = open("/dev/input/event2", O_RDONLY);
    printf("fd = %d\n", fd);
	printf("Control is set to LOW-level.\nPress Enter to continue...\n");
    
    int num = 0;
    while(num<2){
	    size_t mmm = read(fd, &ev, sizeof(ev));
	    if((ev.type == EV_KEY) && (ev.value == 0)){
			if(ev.code == 28){
				num++;	        
			}
	    }
    };
    printf("Starting\n");

    ExternalComm extComm;
    extComm.est_obj = new KF(extComm.dt);

    LoopFunc loop_calc("calc_loop",   extComm.dt,    boost::bind(&ExternalComm::Calc,    &extComm));
    LoopFunc loop_udpRobotSend("udp_RobotSend", 0.001, 2, boost::bind(&ExternalComm::UDPRobotSend, &extComm));
    LoopFunc loop_udpRobotRecv("udp_RobotRecv", 0.001, 2, boost::bind(&ExternalComm::UDPRobotRecv, &extComm));
    LoopFunc loop_udpCompSend("udp_CompSend", 0.001, 3, boost::bind(&ExternalComm::UDPCompSend, &extComm));
    LoopFunc loop_udpCompRecv("udp_CompRecv", 0.001, 3, boost::bind(&ExternalComm::UDPCompRecv, &extComm));

	loop_udpRobotSend.start();
	loop_udpRobotRecv.start();
    loop_udpCompSend.start();
    loop_udpCompRecv.start();
    loop_calc.start();

	
	extComm.kp = 40.0f;
	extComm.kd = 6.0f;
    while(1){
    	size_t mmm = read(fd, &ev, sizeof(ev));
        if(extComm.beginCommand!=true){
		    if((ev.type == EV_KEY) && (ev.value == 0)){
				if(ev.code == 57){
					printf("Starting...\n");
					extComm.beginCommand = true;		        
				}
		    }
		} else {
		    if( (ev.type == EV_KEY) && (ev.value == 0) ){
				if(ev.code == 103){
					extComm.kp += 0.1f;
					printf("Kp: %lf || Kd: %lf\n",extComm.kp,extComm.kd);
				}else if(ev.code == 108){
					extComm.kp -= (extComm.kp<=0) ? 0 : 0.1f;
					printf("Kp: %lf || Kd: %lf\n",extComm.kp,extComm.kd);
				}
				if(ev.code == 17){
					extComm.kd += 0.1f;
					printf("Kp: %lf || Kd: %lf\n",extComm.kp,extComm.kd);
				}else if(ev.code == 31){
					extComm.kd -= (extComm.kd<=0) ? 0 : 0.1f;
					printf("Kp: %lf || Kd: %lf\n",extComm.kp,extComm.kd);
				}
		    }
			sleep(0.25);
		}
		if( (ev.type == EV_KEY) && (ev.value == 0) && (ev.code == 1) ){
			break;
		}
    };

	//loop_udpRobotSend.shutdown();
	//loop_udpRobotRecv.shutdown();
    //loop_udpCompSend.shutdown();
    //loop_udpCompRecv.shutdown();
    //loop_calc.shutdown();
    
    delete extComm.est_obj;
	fclose(extComm.fid);
    return 0; 
}

























