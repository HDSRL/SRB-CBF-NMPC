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

using namespace UNITREE_LEGGED_SDK;

class ExternalComm
{
public:
	ExternalComm() : udpComp(LOWLEVEL){
		fid = fopen("/home/randy/HDSRL/inplace_.csv", "w");
	}

	void Calc();

	FastMPC *loco_obj;
	KF *est_obj;
	UDP udpComp;
	FILE *fid;

	long motiontime = 0;
	float qInit[12] = {0};
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

	double height = 0.28;
	double tune = 1.0;


	float dt = 0.002f;
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
	udpComp.Recv();
	udpComp.GetRecv(state);
	motiontime += 2;

	int standtime = 2000;
	int starttime = 1000;

	double roll = 0, pitch = 0, yaw = 0;
	quat_to_XYZ(state.imu.quaternion[0], state.imu.quaternion[1], state.imu.quaternion[2], state.imu.quaternion[3], roll, pitch, yaw);

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
	// // \dot{w} from Euler angles
	// float rot[3] = {roll,pitch,yaw};
	// float drot[3] = {0.0f};
	// float omega[3] = {0.0f};
	// for(int i=0; i<3; ++i){
	// 	for(int j=0; j<5; ++j){
	// 		rotHist[j][i] = rotHist[j+1][i];
	// 		drot[i] += rotHist[j][i] * gcoeffs[j];
	// 	}
	// 	rotHist[5][i] = rot[i];
	// 	drot[i] += rotHist[5][i] * gcoeffs[5];
	// 	drot[i] /= dt;
	// }
	// omega[0] = drot[1]*sin(rot[0])*sin(rot[2])+drot[0]*cos(rot[2]);
	// omega[1] = drot[1]*sin(rot[0])*cos(rot[2])-drot[0]*sin(rot[2]);
	// omega[2] = drot[1]*cos(rot[0])+drot[2];
	// printf("%lf || %lf || %lf || ",omega[0],omega[1],omega[2]);

	// ================================== //
	// =========== Estimator ============ //
	// ================================== //

	// COM pos estimate
	double fr_toe[3], fl_toe[3], rl_toe[3], rr_toe[3];
	double Jfr_toe[54], Jfl_toe[54], Jrl_toe[54], Jrr_toe[54];
	q[0] = 0; q[1] = 0; q[2] = 0;
	q[3] = 0; q[4] = 0; q[5] = 0;
	FK_FR_toe(fr_toe, q); FK_FL_toe(fl_toe, q);
	FK_RR_toe(rr_toe, q); FK_RL_toe(rl_toe, q);


	// q[0] = -0.183 - (rr_toe[0] + rl_toe[0]) / 2;
	q[0] = 0.183 - fr_toe[0];
	// q[0] = (-0.183 - rr_toe[0])/2 + (0.183 - fr_toe[0])/2; 
	q[1] = -0.132 - (fr_toe[1] + rr_toe[1]) / 2;
	q[2] = -1.0 * (fr_toe[2] + rr_toe[2] + rl_toe[2]) / 3;
	q[3] = roll;
	q[4] = pitch;
	q[5] = yaw - offset; // Use initial value as offset
	// fprintf(fid,"%lf,%lf,%lf,",q[0],q[1],q[2]);


	// COM vel estimate
	// dq[0] = 0; dq[1] = 0; dq[2] = 0;
	// dq[3] = 0; dq[4] = 0; dq[5] = 0;
	// J_FR_toe(Jfr_toe, q); J_FL_toe(Jfl_toe, q);
	// J_RR_toe(Jrr_toe, q); J_RL_toe(Jrl_toe, q);
	// for (int i = 6; i < 18; ++i){
	// 	dq[0] -= (Jfr_toe[3 * i + 0] + Jfl_toe[3 * i + 0] + Jfl_toe[3 * i + 0] + Jfl_toe[3 * i + 0]) * dq[i];
	// 	dq[1] -= (Jfr_toe[3 * i + 1] + Jfl_toe[3 * i + 1] + Jfl_toe[3 * i + 1] + Jfl_toe[3 * i + 1]) * dq[i];
	// 	dq[2] -= (Jfr_toe[3 * i + 2] + Jfl_toe[3 * i + 2] + Jrr_toe[3 * i + 2] + Jrl_toe[3 * i + 2]) * dq[i];
	// }

	// Kalman Estimate
	double relVec[12] = {0.0};
	static int contactIndex[4] = {1,1,1,1};
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
	est_obj->updateKalman(contactIndex,state.imu.accelerometer,state.imu.quaternion,relVec);
	double *comPosVel, *rotMat;
	comPosVel = est_obj->getComPosVel();
	rotMat = est_obj->getRotMat();
	// double temp[6] = {0.0};
    // for(int i=0; i<6; ++i){
    // 	for(int j=0; j<3; ++j){
    // 		rawComHist[j][i] = rawComHist[j+1][i];
    // 		filtComHist[j][i] = filtComHist[j+1][i];
    // 		temp[i] += rawComHist[j][i]*b[j] - filtComHist[j][i]*a[j];
    // 	}
    // 	rawComHist[3][i] = comPosVel[i];
    // 	temp[i] += rawComHist[3][i]*b[3];
    // 	filtComHist[3][i] = temp[i];
	// 	//printf("%lf || ",temp[i]);
    // }
	//printf("\n");
	// q[0]  = filtComHist[3][0]; q[1]  = filtComHist[3][1]; q[2]  = filtComHist[3][2];
	// dq[0] = filtComHist[3][3]; dq[1] = filtComHist[3][4]; dq[2] = filtComHist[3][5];
	q[0]  = comPosVel[0]; q[1]  = comPosVel[1]; q[2]  = comPosVel[2];
	dq[0] = comPosVel[3]; dq[1] = comPosVel[4]; dq[2] = comPosVel[5];
	dq[3] = state.imu.gyroscope[0]; dq[4] = state.imu.gyroscope[1]; dq[5] = state.imu.gyroscope[2];

	// ===================================================== //
	// ================== Standup Routine ================== //
	// ===================================================== //
	if (motiontime <= starttime){
		for (int i = 0; i < 12; ++i){
			qInit[i] = q[i+6];
		}
	}
	double currenttime = 1.0 * (motiontime - starttime) / standtime;
	if ((motiontime >= starttime) && (motiontime <= (starttime + standtime))){
		for (int i = 0; i < 4; ++i){
			cmd.motorCmd[i * 3].q = jointLinearInterpolation(qInit[3 * i], 0, currenttime);
			cmd.motorCmd[i * 3 + 1].q = jointLinearInterpolation(qInit[3 * i + 1], M_PI / 4, currenttime);
			cmd.motorCmd[i * 3 + 2].q = jointLinearInterpolation(qInit[3 * i + 2], -1.0 * M_PI / 2, currenttime);
		}
		for(int i=0; i<12; ++i){
			cmd.motorCmd[i].dq = 0.0;
			cmd.motorCmd[i].Kp = 180.0f;
			cmd.motorCmd[i].Kd = 12.0f;
			cmd.motorCmd[i].tau = 0.0f;
		}
	}else if ( (motiontime < starttime) ){
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
	static double initTime = 1.0*motiontime/1000;
	static double initConf[2] = {0.0};
	if ( motiontime > (starttime + standtime) ){
		double q_nominal[18] = {-0.054013,-0.034594,0.277691,0.034452,-0.007816,0.000162,0.178894,0.675204,-1.712116,0.084453,0.610600,-1.543547,0.154241,0.614561,-1.626020,0.086771,0.479917,-1.414170};
		double dq_nominal[18] = {0.0};
		if (beginCommand){
			Eigen::Matrix<double, 18, 1> tau_ctrl, dq_ctrl, q_ctrl;
			Eigen::Matrix<double,  6, 1> y_out, y_swing, y_sw_des;
			loco_obj->setTauEst(tauEst);
			double phaseVar;
			if (setup){
				Eigen::Matrix<double, 3, 1> com;
				com(0) = q[0], com(1) = q[1], com(2) = q[2];
				loco_obj->timeSetup(motiontime, motiontime+2000);
				loco_obj->posSetup(com);
				loco_obj->setPoseType(POSE_Z);
				loco_obj->setTauEst(tauEst);
				loco_obj->setTauPrev(tauEst);
				loco_obj->compute(q, dq, rotMat, STAND, motiontime, 0.0, 0.0, height);
				setup = false;
			}
			if (beginPose == false){
				initTime = 1.0*motiontime/1000;
				initConf[0] = q[10];
				initConf[1] = q[11];
				loco_obj->compute(q, dq, rotMat, STAND, motiontime, 0.0, 0.0, height);
			}else{
				// static double bezTime = 0;
				// double len = 0.25;
				// bezTime+=dt;
				// bezTime *= (bezTime>len) ? 0 : 1;
				// double s = bezTime/len;

				// Eigen::Vector3d K_traj, H_traj;
				// Eigen::VectorXd aH(8);
				// Eigen::VectorXd aK(8);
				// aH << initConf[0],initConf[0],1.02,1.02,1.02,1.02,initConf[0],initConf[0];
				// aK << initConf[1],initConf[1],-2.074,-2.074,-2.074,-2.074,initConf[1],initConf[1];
				// calcBezierAll(aH, s, H_traj);
				// calcBezierAll(aK, s, K_traj);

				// q_nominal[10] = H_traj(0);
				// dq_nominal[10] = H_traj(1);

				// q_nominal[11] = K_traj(0);
				// dq_nominal[11] = K_traj(1);

				// q_nominal[13] = H_traj(0);
				// dq_nominal[13] = H_traj(1);

				// q_nominal[14] = K_traj(0);
				// dq_nominal[14] = K_traj(1);

				loco_obj->compute(q, dq, rotMat, INPLACE, motiontime, 0.0, 0.0, height);
				static Eigen::Matrix<int, 4, 1> contactMat;
				contactMat = loco_obj->getContactMatrix();
				contactIndex[0] = contactMat(0); contactIndex[1] = contactMat(1);
				contactIndex[2] = contactMat(2); contactIndex[3] = contactMat(3);
			}

			// Set the command
			tau_ctrl = loco_obj->getJointTorqueCommand();
			dq_ctrl  = loco_obj->getJointVelocityCommand();
			q_ctrl   = loco_obj->getJointPositionCommand();
			y_out    = loco_obj->getHZDOutputs();
			y_swing  = loco_obj->getSwingOutputs();
			y_sw_des = loco_obj->getSwingDes();
			phaseVar = loco_obj->getPhaseVar();
			for (int i = 0; i < 12; ++i){
				cmd.motorCmd[i].tau = tau_ctrl(i + 6);
				// cmd.motorCmd[i].tau = 0.0f;
				// if(beginPose==false){
					cmd.motorCmd[i].q  = q_ctrl(i + 6);
					cmd.motorCmd[i].dq = dq_ctrl(i + 6);
				// }
				// if(beginPose==true){
				// 	cmd.motorCmd[i].q  = q_nominal[i+6];
				// 	cmd.motorCmd[i].dq  = dq_nominal[i+6];
				// 	if( (i>=4 && i<=5) || (i>=7 && i<=7) ){
				// 		cmd.motorCmd[i].tau = 0.0f;
				// 		// cmd.motorCmd[i].Kp = 0.0f;
				// 		// cmd.motorCmd[i].Kd = 0.0f;
				// 	}
				// }
				cmd.motorCmd[i].Kp = 400;
				cmd.motorCmd[i].Kd = 12;
				if(beginPose==true){
					for(int i=0; i<4; ++i){
						if(contactIndex[i]==0){
							cmd.motorCmd[3*i].Kp = 80;
							cmd.motorCmd[3*i+1].Kp = 80;
							cmd.motorCmd[3*i+2].Kp = 80;

							cmd.motorCmd[3*i].Kd = 6;
							cmd.motorCmd[3*i+1].Kd = 6;
							cmd.motorCmd[3*i+2].Kd = 6;
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

			// Write the data to a file
			for(int i=0; i<18; ++i){
				fprintf(fid,"%lf,",q[i]);
			}
			for(int i=0; i<6; ++i){
				fprintf(fid,"%lf,",y_out(i));
			}
			for(int i=0; i<6; ++i){
				fprintf(fid,"%lf,",y_swing(i));
			}
			for(int i=0; i<6; ++i){
				fprintf(fid,"%lf,",y_sw_des(i));
			}
			for(int i=0; i<12; ++i){
				fprintf(fid,"%lf,",cmd.motorCmd[i].q);
			}
			for(int i=0; i<12; ++i){
				fprintf(fid,"%lf,",cmd.motorCmd[i].dq);
			}
			for(int i=0; i<12; ++i){
				fprintf(fid,"%lf,",cmd.motorCmd[i].tau);
			}
			fprintf(fid,"%lf,",phaseVar);
			for(int i=0; i<18; ++i){
				fprintf(fid,"%lf,",dq[i]);
			}
			for(int i=0; i<4; ++i){
				fprintf(fid,"%lf,",state.footForceEst[i]);
			}
			for(int i=0; i<12; ++i){
				fprintf(fid, "%lf,",state.motorState[i].tauEst);
			}
			// for(int i=0; i<18; ++i){
			// 	fprintf(fid,"%lf,",q[i]);
			// }
			// for(int i=0; i<6; ++i){
			// 	fprintf(fid,"%lf,",y_out(i));
			// }
			// for(int i=0; i<6; ++i){
			// 	fprintf(fid,"%lf,",y_swing(i));
			// }
			// for(int i=0; i<6; ++i){
			// 	fprintf(fid,"%lf,",y_sw_des(i));
			// }
			// for(int i=0; i<12; ++i){
			// 	fprintf(fid,"%lf,",cmd.motorCmd[i].tau);
			// }
			fprintf(fid,"\n");

		}else{
			printf("Press Space to Continue\n");
			offset = yaw;
		}
	}

	udpComp.Send();
	udpComp.SetSend(cmd);
}

// -0.054013,-0.034594,0.277691,0.034452,-0.007816,0.000162,0.178894,0.675204,-1.712116,0.084453,0.610600,-1.543547,0.154241,0.614561,-1.626020,0.086771,0.479917,-1.414170

int main(int argc, char *argv[])
{
	ExternalComm extComm;

	extComm.loco_obj = new FastMPC(argc, argv);
	extComm.est_obj = new KF(extComm.dt);

	LoopFunc loop_calc("calc_loop", extComm.dt, boost::bind(&ExternalComm::Calc, &extComm));
	loop_calc.start();

	InitEnvironment();
	extComm.udpComp.InitCmdData(extComm.cmd);

	struct input_event ev;
    int k = 1;
    int fd = open("/dev/input/event4", O_RDONLY);
    printf("fd = %d\n", fd);
	int loopthrough = 1;
	while (loopthrough==1){
		size_t mmm = read(fd, &ev, sizeof(ev));
		if( (ev.type==EV_KEY) && (ev.value==0) ){
			switch (ev.code){
				case KEY_P:
					extComm.beginPose = true;
					break;
				case KEY_SPACE:
					extComm.beginCommand = true;
					break;
				case KEY_ESC:
					loopthrough = 0;
					break;
				case KEY_UP:
					if (extComm.tune >1){
						break;
					}
					extComm.tune +=0.01;
					break;
				case KEY_DOWN:
					if(extComm.tune<0){
						break;
					}	
					extComm.tune -=0.01;
					break;
			}
			printf("%lf\n",extComm.tune);
		} 
		sleep(0.1);
	};

	delete extComm.loco_obj;
	delete extComm.est_obj;
	fclose(extComm.fid);
	return 0;
}
