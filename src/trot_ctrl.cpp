/************************************************************************
Copyright (c) 2020, Unitree Robotics.Co.Ltd. All rights reserved.
Use of this source code is governed by the MPL-2.0 license, see LICENSE.
************************************************************************/

#include "A1_Dynamics.h"
#include "kalman.hpp"
#include "fast_MPC.hpp"
#include "utils.h"
#include "multi_pc_comm.h"
#include "timer.h"

#include "stdio.h"
#include "fcntl.h"
#include "unistd.h"

#include "linux/input.h"
#include "sys/stat.h"
#include "fstream"

#include "remoteCtrl.h"

using namespace UNITREE_LEGGED_SDK;

class ExternalComm
{
public:
	ExternalComm() : udpComp(LOWLEVEL){
		fid = fopen("/home/kaveh/A1_exp_data/nothing.csv", "w");
	}

	void Calc();

	FastMPC *loco_obj;
	KF *est_obj;
	UDP udpComp;
	FILE *fid;

	long motiontime = 0;
	float qInit[12] = {0};
	bool softFall = false;
	bool beginCommand = false;
	bool beginPose = false;
	bool setup = true;
	double q[18] = {0.0};
	double dq[18] = {0.0};
	double tauEst[12] = {0.0};

	double offset = 0.0;
	float gcoeffs[6] = {-0.14286f, -0.08571f, -0.02857f, 0.02857f, 0.08571f, 0.14286f};
	double a[4] = {-9.6300211588509210e-01, 2.9253101348486945e+00, -2.9623014460856600e+00, 1.0000000000000000e+00};
	double b[4] = {8.2160974279599230e-07, 2.4648292283879769e-06, 2.4648292283879769e-06, 8.2160974279599230e-07};
	float qHist[6][12] = {{0.0f}};
	float rotHist[6][3] = {{0.0f}};
	float filtComHist[4][6] = {{0.0f}}; // last row is most recent
	float rawComHist[4][6] = {{0.0f}};	// last row is most recent

	LowState state = {0};
	LowCmd cmd = {0};
	buttons btn;

	double height = 0.0;
	double tune = 1.0;
	int stop = 0;
	double vel[3] = {0};
	
	int actCon[4] = {0};


	float dt = 0.001f;
};

double jointLinearInterpolation(double initPos, double targetPos, double rate)
{
	double p;
	rate = std::min(std::max(rate, 0.0), 1.0);
	p = initPos * (1 - rate) + targetPos * rate;
	return p;
}

void ExternalComm::Calc()
{
//	timer tset;
//	tic(&tset);
	
	udpComp.Recv();
	udpComp.GetRecv(state);
	btn = getButtonState(state.wirelessRemote, btn);
	int sigfb = 0;
	int siglr = 0;
	if (btn.up == 1){
		sigfb = 1;
	} else if(btn.down == 1) {
		sigfb = -1;
	}
	if (btn.right == 1){
		siglr = -1;
	} else if(btn.left == 1) {
		siglr = 1;
	}
	if (btn.B==1 && btn.L2==0){
		vel[0] = 0.0;
		vel[1] = 0.0;
		sigfb = 0;
		siglr = 0;
	} else if( (btn.B!=0) && (btn.L2!=0) ) {
		stop = 1;
	}
	if ( (btn.A!=0) && (btn.L2!=0) ){
		softFall = true;
	}
	
	motiontime += 1;

	int standtime = 2000;
	int starttime = 1000;

	// ===================================================== //
	// =========== Filter the joint pos and vel ============ //
	// ===================================================== //
	for (int i = 0; i < 12; ++i){
		// Grab "raw" joint position
		q[i + 6] = state.motorState[i].q;
		tauEst[i] = state.motorState[i].tauEst;

		// Update q history and calculate filtered dq (6th order diff Golay)
		dq[i + 6] = 0.0;
		for (int j = 0; j < 5; ++j){
			qHist[j][i] = qHist[j + 1][i];
			dq[i + 6] += qHist[j][i] * gcoeffs[j];
		}
		qHist[5][i] = state.motorState[i].q;
		dq[i + 6] += qHist[5][i] * gcoeffs[5];
		dq[i + 6] /= dt;
	}
	double roll = 0, pitch = 0, yaw = 0;
	quat_to_XYZ(state.imu.quaternion[0], state.imu.quaternion[1], state.imu.quaternion[2], state.imu.quaternion[3], roll, pitch, yaw);
//	q[3]  = roll; q[4]  = pitch; q[5]  = yaw - offset; // Use initial value as offset
	q[3] = state.imu.rpy[0]; q[4] = state.imu.rpy[1]; q[5] = state.imu.rpy[2];
	dq[3] = state.imu.gyroscope[0]; dq[4] = state.imu.gyroscope[1]; dq[5] = state.imu.gyroscope[2];
	
	std::cout<<roll << "\t" << state.imu.rpy[0] <<std::endl;
	std::cout<<pitch << "\t" << state.imu.rpy[1] <<std::endl;
	std::cout<<yaw << "\t" << state.imu.rpy[2] <<std::endl;
	
	double eul[3] = {roll,pitch,yaw};

	int footForce[4];
	footForce[0] = state.footForce[0]; footForce[1] = state.footForce[1];
	footForce[2] = state.footForce[2]; footForce[3] = state.footForce[3];

	// ================================== //
	// ========= Kin Estimator ========== //
	// ================================== //
	static int contactIndex[4] = {1,1,1,1};
	int thresh = 10;
	actCon[0] = (footForce[0]>thresh) ? 1 : 0;
	actCon[1] = (footForce[1]>thresh) ? 1 : 0;
	actCon[2] = (footForce[2]>thresh) ? 1 : 0;
	actCon[3] = (footForce[3]>thresh) ? 1 : 0;
	
	float weightedCon[4];
	weightedCon[0] = actCon[0]+contactIndex[0];
	weightedCon[1] = actCon[1]+contactIndex[1];
	weightedCon[2] = actCon[2]+contactIndex[2];
	weightedCon[3] = actCon[3]+contactIndex[3];
	float numContact = (weightedCon[0]+weightedCon[1]+weightedCon[2]+weightedCon[3]);
	
//	int weightedCon[4];
//	weightedCon[0] = contactIndex[0];
//	weightedCon[1] = contactIndex[1];
//	weightedCon[2] = contactIndex[2];
//	weightedCon[3] = contactIndex[3];
//	int numContact = (weightedCon[0]+weightedCon[1]+weightedCon[2]+weightedCon[3]);

//	float weightedCon[4];
//	weightedCon[0] = (contactIndex[0]==1 || actCon[0]==1) ? 1 : 0;
//	weightedCon[1] = (contactIndex[0]==1 || actCon[0]==1) ? 1 : 0;;
//	weightedCon[2] = (contactIndex[0]==1 || actCon[0]==1) ? 1 : 0;;
//	weightedCon[3] = (contactIndex[0]==1 || actCon[0]==1) ? 1 : 0;;
//	float numContact = (weightedCon[0]+weightedCon[1]+weightedCon[2]+weightedCon[3]);
	
	
//	contactIndex[0] = (actCon[0]==1 || contactIndex[0]==1) ? 1 : 0;
//	contactIndex[1] = (actCon[1]==1 || contactIndex[1]==1) ? 1 : 0;
//	contactIndex[2] = (actCon[2]==1 || contactIndex[2]==1) ? 1 : 0;
//	contactIndex[3] = (actCon[3]==1 || contactIndex[3]==1) ? 1 : 0;
//	float numContact = (contactIndex[0]+contactIndex[1]+contactIndex[2]+contactIndex[3]);
	
	

	// COM pos estimate
	double fr_toe[3], fl_toe[3], rl_toe[3], rr_toe[3];
	static double COM[3]= {0,0,0};
	q[0] = 0; q[1] = 0; q[2] = 0;
//	q[3] = 0; q[4] = 0; q[5] = 0;
	FK_FR_toe(fr_toe, q); FK_FL_toe(fl_toe, q);
	FK_RR_toe(rr_toe, q); FK_RL_toe(rl_toe, q);
	
	static double fr_prev[3] = {fr_toe[0],fr_toe[1],fr_toe[2]};
	static double fl_prev[3] = {fl_toe[0],fl_toe[1],fl_toe[2]};
	static double rr_prev[3] = {rr_toe[0],rr_toe[1],rr_toe[2]};
	static double rl_prev[3] = {rl_toe[0],rl_toe[1],rl_toe[2]};
	double deltaPos[2] = {0.0};
	for(int i=0; i<2; ++i){
		deltaPos[i] -= (fr_toe[i]-fr_prev[i])*weightedCon[0];
		deltaPos[i] -= (fl_toe[i]-fl_prev[i])*weightedCon[1];
		deltaPos[i] -= (rr_toe[i]-rr_prev[i])*weightedCon[2];
		deltaPos[i] -= (rl_toe[i]-rl_prev[i])*weightedCon[3];
		deltaPos[i] /= numContact;
	}
	COM[0] += deltaPos[0];
	COM[1] += deltaPos[1];
	COM[2]  = -1.0*(fr_toe[2]*weightedCon[0]+fl_toe[2]*weightedCon[1]+rr_toe[2]*weightedCon[2]+rl_toe[2]*weightedCon[3])/numContact;
	
	for(int i=0; i<3; ++i){
		fr_prev[i] = fr_toe[i];
		fl_prev[i] = fl_toe[i];
		rr_prev[i] = rr_toe[i];
		rl_prev[i] = rl_toe[i];		
	}
	
	double Jfr_toe[54], Jfl_toe[54], Jrl_toe[54], Jrr_toe[54];
	static double COM_vel[3] = {0,0,0};
	J_FR_toe(Jfr_toe, q); J_FL_toe(Jfl_toe, q);
	J_RR_toe(Jrr_toe, q); J_RL_toe(Jrl_toe, q);
	COM_vel[0] = 0; COM_vel[1] = 0; COM_vel[2] = 0;
	for (int i = 3; i < 18; ++i){
		COM_vel[0] -= (Jfr_toe[3*i+0]*weightedCon[0] + Jfl_toe[3*i+0]*weightedCon[1] + Jfl_toe[3*i+0]*weightedCon[2] + Jfl_toe[3*i+0]*weightedCon[3])*dq[i];
	 	COM_vel[1] -= (Jfr_toe[3*i+1]*weightedCon[0] + Jfl_toe[3*i+1]*weightedCon[1] + Jfl_toe[3*i+1]*weightedCon[2] + Jfl_toe[3*i+1]*weightedCon[3])*dq[i];
	 	COM_vel[2] -= (Jfr_toe[3*i+2]*weightedCon[0] + Jfl_toe[3*i+2]*weightedCon[1] + Jrr_toe[3*i+2]*weightedCon[2] + Jrl_toe[3*i+2]*weightedCon[3])*dq[i];
	}
	COM_vel[0] /= numContact;
	COM_vel[1] /= numContact;
	COM_vel[2] /= numContact;
	
	// ================================== //
	// ============= Kalman ============= //
	// ================================== //
	double relVec[12] = {0.0};
	if(motiontime<500){
		relVec[0] = 0.183f;  relVec[1]  = -0.132f; relVec[2]  = 0.09f;
		relVec[3] = 0.183f;  relVec[4]  = 0.132f;  relVec[5]  = 0.09f;
		relVec[6] = -0.183f; relVec[7]  = -0.132f; relVec[8]  = 0.09f;
		relVec[9] = -0.183f; relVec[10] = 0.132f;  relVec[11] = 0.09f;
    } else {
    	relVec[0] = -1.0*fr_toe[0]; relVec[1]  = -1.0*fr_toe[1]; relVec[2]  = -1.0*fr_toe[2];
		relVec[3] = -1.0*fl_toe[0]; relVec[4]  = -1.0*fl_toe[1]; relVec[5]  = -1.0*fl_toe[2];
		relVec[6] = -1.0*rr_toe[0]; relVec[7]  = -1.0*rr_toe[1]; relVec[8]  = -1.0*rr_toe[2];
		relVec[9] = -1.0*rl_toe[0]; relVec[10] = -1.0*rl_toe[1]; relVec[11] = -1.0*rl_toe[2];
    }
//	est_obj->updateKalman(contactIndex,state.imu.accelerometer,eul,relVec);	// Using Euler (includes yaw offset)
	est_obj->updateKalman(contactIndex,state.imu.accelerometer,state.imu.quaternion,relVec); // Using Quaternions
	double *comPosVel, *rotMat, *footPos;
//	comPosVel = est_obj->getComPosVel();
//	footPos = est_obj->getFootPos();
	rotMat = est_obj->getRotMat();

//	q[0]  = comPosVel[0]; q[1]  = comPosVel[1]; q[2]  = comPosVel[2];
//	dq[0] = comPosVel[3]; dq[1] = comPosVel[4]; dq[2] = comPosVel[5];
	
	 q[0]  = COM[0];     q[1] = COM[1];      q[2] = COM[2];
	dq[0] = COM_vel[0]; dq[1] = COM_vel[1]; dq[2] = COM_vel[2];

	// ===================================================== //
	// ============= Quad Initialization time ============== //
	// ===================================================== //
	if ( (motiontime < starttime) ){
		for(int i=0; i<12; ++i){
			cmd.motorCmd[i].dq = 0.0;
			cmd.motorCmd[i].Kp = 0.0f;
			cmd.motorCmd[i].Kd = 0.0f;
			cmd.motorCmd[i].tau = 0.0f;
		}
	}

	// ===================================================== //
	// ================== LL Controller ==================== //
	// ===================================================== //
	static long int endStand = standtime;
	if ( motiontime >= starttime ){
		if (beginCommand & !softFall){
			Eigen::Matrix<double, 18, 1> tau_ctrl, dq_ctrl, q_ctrl;
			Eigen::Matrix<double,  6, 1> y_out, y_swing, y_sw_des;
			loco_obj->setTauEst(tauEst);
			double phaseVar;
			if (setup){
				Eigen::Matrix<double, 3, 1> com;
				com(0) = q[0], com(1) = q[1], com(2) = q[2];
				endStand = motiontime+standtime;
				loco_obj->timeSetup(motiontime, motiontime+standtime);
				loco_obj->posSetup(com);
				loco_obj->setPoseType(POSE_COMB);
				loco_obj->setTauEst(tauEst);
				loco_obj->setTauPrev(tauEst);
//				loco_obj->compute(q, dq, rotMat, STAND, motiontime, 0.0, 0.0, height);
				setup = false;
			}
			if(motiontime<endStand){
				beginPose = 0;
			} else {
				beginPose = 1;
			}
			if (beginPose == 0){
				vel[0] = 0; vel[1] = 0; vel[2] = 0;
				loco_obj->compute(q, dq, rotMat, STAND, motiontime, vel);
				if (motiontime >= endStand){
					printf("Press X to continue\n");
				}
			}else if (beginPose == 1 && motiontime>=endStand){
				vel[0] += 0.01*sigfb;
				vel[1] += 0.01*siglr;
				printf("Fwd Speed: %lf\t||\tLat Speed: %lf\n",vel[0],vel[1]);
				loco_obj->compute(q, dq, rotMat, TROT, motiontime, vel);
				static Eigen::Matrix<int, 4, 1> contactMat;
				contactMat = loco_obj->getContactMatrix();
				contactIndex[0] = contactMat(0); contactIndex[1] = contactMat(1);
				contactIndex[2] = contactMat(2); contactIndex[3] = contactMat(3);
			}

			// Set the command
			tau_ctrl = loco_obj->getJointTorqueCommand();
			dq_ctrl  = loco_obj->getJointVelocityCommand();
			q_ctrl   = loco_obj->getJointPositionCommand();
			for (int i = 0; i < 12; ++i){
				cmd.motorCmd[i].tau = tau_ctrl(i + 6);
				cmd.motorCmd[i].q  = q_ctrl(i + 6);
				cmd.motorCmd[i].dq = dq_ctrl(i + 6);

				cmd.motorCmd[i].Kp = 80;
				cmd.motorCmd[i].Kd = 8;
				if(beginPose==true){
					for(int i=0; i<4; ++i){
						if(contactIndex[i]==0){
							cmd.motorCmd[3*i].Kp = 2;
							cmd.motorCmd[3*i+1].Kp = 2;
							cmd.motorCmd[3*i+2].Kp = 2;

							cmd.motorCmd[3*i].Kd = 5;
							cmd.motorCmd[3*i+1].Kd = 5;
							cmd.motorCmd[3*i+2].Kd = 5; 
						}
					}	
				}
			}

			// Saturate the command
			float hr_max = 8.0f,  hr_min = -8.0f;
			float hp_max = 20.0f, hp_min = -8.0f;
			float kn_max = 25.0f, kn_min = -8.0f;
			for (int i = 0; i < 4; i++){
				cmd.motorCmd[3 * i + 0].tau = (cmd.motorCmd[3 * i + 0].tau > hr_max) ? hr_max : cmd.motorCmd[3 * i + 0].tau;
				cmd.motorCmd[3 * i + 0].tau = (cmd.motorCmd[3 * i + 0].tau < hr_min) ? hr_min : cmd.motorCmd[3 * i + 0].tau;
				cmd.motorCmd[3 * i + 1].tau = (cmd.motorCmd[3 * i + 1].tau > hp_max) ? hp_max : cmd.motorCmd[3 * i + 1].tau;
				cmd.motorCmd[3 * i + 1].tau = (cmd.motorCmd[3 * i + 1].tau < hp_min) ? hp_min : cmd.motorCmd[3 * i + 1].tau;
				cmd.motorCmd[3 * i + 2].tau = (cmd.motorCmd[3 * i + 2].tau > kn_max) ? kn_max : cmd.motorCmd[3 * i + 2].tau;
				cmd.motorCmd[3 * i + 2].tau = (cmd.motorCmd[3 * i + 2].tau < kn_min) ? kn_min : cmd.motorCmd[3 * i + 2].tau;
			}

		}else if ( (!beginCommand) && (!softFall) ){
			printf("Press A to continue\n");
			offset = yaw;
		}else if ( softFall ) {
			for (int i = 0; i < 12; ++i){
				cmd.motorCmd[i].tau = 0.0;
				cmd.motorCmd[i].q  = 0.0;
				cmd.motorCmd[i].dq = 0.0;

				cmd.motorCmd[i].Kp = 0;
				cmd.motorCmd[i].Kd = 4;
				if ( ((i+1)%3) == 0){
					cmd.motorCmd[i].Kd = 8;
				}
			}
		}
	}

	udpComp.Send();
	udpComp.SetSend(cmd);
	
//	std::cout<< toc(&tset) << std::endl;
}


int main(int argc, char *argv[])
{
	ExternalComm extComm;

	extComm.loco_obj = new FastMPC(argc, argv);
	extComm.est_obj = new KF(extComm.dt);

	LoopFunc loop_calc("calc_loop", extComm.dt, boost::bind(&ExternalComm::Calc, &extComm));
	loop_calc.start();

	InitEnvironment();
	extComm.udpComp.InitCmdData(extComm.cmd);

//	struct input_event ev;
//    int k = 1;
//    int fd = open("/dev/input/event2", O_RDONLY);
//    printf("fd = %d\n", fd);
//	int loopthrough = 1;
//	size_t mmm;
//	while ( (loopthrough==1) && (extComm.stop==0) ){
//		mmm = read(fd, &ev, sizeof(ev));
//		if( (ev.type==EV_KEY) && (ev.value==0) ){
//			switch (ev.code){
//				case KEY_P:
//					extComm.beginPose = true;
//					break;
//				case KEY_SPACE:
//					extComm.beginCommand = true;
//					break;
//				case KEY_ESC:
//					loopthrough = 0;
//					break;
//				case KEY_UP:
//					break;
//				case KEY_DOWN:
//					break;
//			}
//		}
//		sleep(0.1);
//	};
	
	while ( (extComm.stop==0) ){
		if(extComm.btn.X > 0){
			extComm.beginPose = 1;
		}
		if(extComm.btn.A > 0){
			extComm.beginCommand = 1;
		}
		sleep(0.1);
	};

	delete extComm.loco_obj;
	delete extComm.est_obj;
	fclose(extComm.fid);
	return 0;
}




