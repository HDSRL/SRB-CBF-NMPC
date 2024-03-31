/************************************************************************
Copyright (c) 2020, Unitree Robotics.Co.Ltd. All rights reserved.
Use of this source code is governed by the MPL-2.0 license, see LICENSE.
************************************************************************/

#include <math.h>
#include "multi_pc_comm.h"
#include "kalman.hpp"
#include "A1_Dynamics.h"

using namespace UNITREE_LEGGED_SDK;

class ExternalComm
{
public:
    ExternalComm(): control(LeggedType::A1, LOWLEVEL), udpComp(8090, "192.168.123.160", 8017, sizeof(RobotSend), sizeof(RobotRecv)), udpRobot(){}
    
    void UDPRecv();
    void UDPSend();
    void UDPRobotSend();
    void UDPRobotRecv();
    void Calc();

	Control control;	
	KF* est_obj;

    UDP udpComp;
    UDP udpRobot;
    
    long motiontime = 0;
    double q[18] = {0.0};
    double dq[18] = {0.0};
    
    LowState state = {0};
    LowCmd     cmd = {0};
    
    RobotRecv rrecv;
    RobotSend rsend;
    float dt = 0.002f;
};

void ExternalComm::UDPRecv()
{
    udpComp.Recv();
}

void ExternalComm::UDPSend()
{  
    udpComp.Send();
}

void ExternalComm::UDPRobotSend(){
	udpRobot.Send();
}

void ExternalComm::UDPRobotRecv(){
	udpRobot.Recv();
}

void ExternalComm::Calc()
{
	motiontime += 2;

    udpComp.GetRecv((char*)&rrecv);
	udpRobot.GetRecv(state);
	
    
    // Pack the data for the estimator
    int thresh = 10;
    double relVec[12] = {0};
    int contactIndex[4] = {0};
    double fr_toe[3], fl_toe[3], rl_toe[3], rr_toe[3];
    contactIndex[0] = state.footForce[0]>thresh ? 1 : 0;
    contactIndex[1] = state.footForce[1]>thresh ? 1 : 0;
    contactIndex[2] = state.footForce[2]>thresh ? 1 : 0;
    contactIndex[3] = state.footForce[3]>thresh ? 1 : 0;
    if(motiontime>200){
    	double fr_toe[3], fl_toe[3], rr_toe[3], rl_toe[3];
    	FK_FR_toe(fr_toe, q);
		FK_FL_toe(fl_toe, q);
		FK_RR_toe(rr_toe, q);
		FK_RL_toe(rl_toe, q);
		
		relVec[0]  = fr_toe[0]-q[0];
		relVec[1]  = fr_toe[1]-q[1];
		relVec[2]  = -1*(fr_toe[2]-q[2]);
		
		relVec[3]  = fl_toe[0]-q[0];
		relVec[4]  = fl_toe[1]-q[1];
		relVec[5]  = -1*(fl_toe[2]-q[2]);
		 
		relVec[6]  = rr_toe[0]-q[0];
		relVec[7]  = rr_toe[1]-q[1];
		relVec[8]  = -1*(rr_toe[2]-q[2]);
		
		relVec[9]  = rl_toe[0]-q[0];
		relVec[10] = rl_toe[1]-q[1];
		relVec[11] = -1*(rl_toe[2]-q[2]);
    } else {
    	relVec[0]  = 0.183f;
		relVec[1]  = -0.132f;
		relVec[2]  = 0.08f;
		
		relVec[3]  = 0.183f;
		relVec[4]  = 0.132f;
		relVec[5]  = 0.08f;
		 
		relVec[6]  = -0.183f;
		relVec[7]  = -0.132f;
		relVec[8]  = 0.08f;
		
		relVec[9]  = -0.183f;
		relVec[10] = 0.132f;
		relVec[11] = 0.08f;
		
		contactIndex[0] = 1;
		contactIndex[1] = 1;
		contactIndex[2] = 1;
		contactIndex[3] = 1;
    	
    	q[2] = 0.08f;
    }
    
    // Run the estimator
    est_obj->updateKalman(contactIndex,state.imu.accelerometer,state.imu.quaternion,relVec);
	double* comPosVel;
    comPosVel = est_obj->getComPosVel();
    
    // Pack the data for the controller
    q[0] = comPosVel[0];
    q[1] = comPosVel[1];
    q[2] = comPosVel[2];
    q[3] = state.imu.rpy[0];
    q[4] = state.imu.rpy[1];
    q[5] = state.imu.rpy[2];
    
    dq[0] = comPosVel[3];
    dq[1] = comPosVel[4];
    dq[2] = comPosVel[5];
    dq[3] = 0.0f;
    dq[4] = 0.0f;
    dq[5] = 0.0f;
    
	for(int i=0; i<12; i++){
		q[i+6] = state.motorState[i].q;	
		dq[i+6] = state.motorState[i].dq;
	}
    
    
    // Set the torque
    for(int i = 0; i<12; i++){
    	cmd.motorCmd[i].tau = 0.0f;
    	cmd.motorCmd[i].q   = PosStopF;
    	cmd.motorCmd[i].dq  = VelStopF;
    }
    
    //udpRobot.SetSend(cmd);
}

int main(void) 
{
    ExternalComm extComm;
    extComm.est_obj = new KF();

    LoopFunc loop_calc("calc_loop",   extComm.dt,    boost::bind(&ExternalComm::Calc,    &extComm));
    LoopFunc loop_udpRobotRecv("udp_RobotRecv", extComm.dt, 2, boost::bind(&ExternalComm::UDPRobotRecv, &extComm));
    LoopFunc loop_udpRobotSend("udp_RobotSend", extComm.dt, 2, boost::bind(&ExternalComm::UDPRobotSend, &extComm));
    LoopFunc loop_udpCompSend("udp_send", extComm.dt, 3, boost::bind(&ExternalComm::UDPSend, &extComm));
    LoopFunc loop_udpCompRecv("udp_recv", extComm.dt, 3, boost::bind(&ExternalComm::UDPRecv, &extComm));

	loop_udpRobotSend.start();
	loop_udpRobotRecv.start();
    loop_udpCompSend.start();
    loop_udpCompRecv.start();
    loop_calc.start();

    while(1){
        sleep(10);
    };

    return 0; 
}
