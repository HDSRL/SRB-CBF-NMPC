/************************************************************************
Copyright (c) 2020, Unitree Robotics.Co.Ltd. All rights reserved.
Use of this source code is governed by the MPL-2.0 license, see LICENSE.
************************************************************************/

#include "A1_Dynamics.h"
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
	ExternalComm() : udpComp(LOWLEVEL)
	{
		fid = fopen("/home/randy/HDSRL/directSinTest_Kp200_500_Kd5_10.csv", "w");
	}

	void Calc();

	FastMPC *loco_obj;
	UDP udpComp;
	FILE *fid;

	long motiontime = 0;
	float qInit[12] = {0};
	bool beginCommand = false;
	bool beginPose = false;
	bool setup = true;
	double q[18] = {0.0};
	double dq[18] = {0.0};

	double offset = 0.0;
	float gcoeffs[6] = {-0.14286f, -0.08571f, -0.02857f, 0.02857f, 0.08571f, 0.14286f};
	double a[4] = {-9.6300211588509210e-01, 2.9253101348486945e+00, -2.9623014460856600e+00, 1.0000000000000000e+00};
	double b[4] = {8.2160974279599230e-07, 2.4648292283879769e-06, 2.4648292283879769e-06, 8.2160974279599230e-07};
	float qHist[6][12] = {{0.0f}};
	float filtComHist[4][6] = {{0.0f}}; // last row is most recent
	float rawComHist[4][6] = {{0.0f}};	// last row is most recent

	LowState state = {0};
	LowCmd cmd = {0};

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
	motiontime += dt * 1000;

	int standtime = 2000;
	int starttime = 1000;

	// ===================================================== //
	// =========== Filter the joint pos and vel ============ //
	// ===================================================== //
	for (int i = 0; i < 12; ++i){
		// Grab "raw" joint position
		q[i + 6] = state.motorState[i].q;

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

	double roll = 0, pitch = 0, yaw = 0;
	quat_to_XYZ(state.imu.quaternion[0], state.imu.quaternion[1], state.imu.quaternion[2], state.imu.quaternion[3], roll, pitch, yaw);
	q[0] = -0.183 - (rr_toe[0] + rl_toe[0]) / 2;
	q[1] = 0.132 - (fl_toe[1] + rl_toe[1]) / 2;
	q[2] = -1.0 * (fr_toe[2] + fl_toe[2] + rr_toe[2] + rl_toe[2]) / 4;
	q[3] = roll;
	q[4] = pitch;
	q[5] = yaw - offset; // Use initial value as offset

	// COM vel estimate
	dq[0] = 0; dq[1] = 0; dq[2] = 0;
	dq[3] = 0; dq[4] = 0; dq[5] = 0;
	J_FR_toe(Jfr_toe, q); J_FL_toe(Jfl_toe, q);
	J_RR_toe(Jrr_toe, q); J_RL_toe(Jrl_toe, q);
	for (int i = 6; i < 18; ++i){
		dq[0] -= (Jfr_toe[3 * i] + Jfl_toe[3 * i] + Jfl_toe[3 * i] + Jfl_toe[3 * i]) * dq[i];
		dq[1] -= (Jfr_toe[3 * i + 1] + Jfl_toe[3 * i + 1] + Jfl_toe[3 * i + 1] + Jfl_toe[3 * i + 1]) * dq[i];
		dq[2] -= (Jfr_toe[3 * i + 2] + Jfl_toe[3 * i + 2] + Jfl_toe[3 * i + 2] + Jfl_toe[3 * i + 2]) * dq[i];
	}

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

			cmd.motorCmd[i * 3].dq = 0.0;
			cmd.motorCmd[i * 3 + 1].dq = 0.0;
			cmd.motorCmd[i * 3 + 2].dq = 0.0;

			cmd.motorCmd[i * 3].Kp = 180.0f;
			cmd.motorCmd[i * 3 + 1].Kp = 150.0f;
			cmd.motorCmd[i * 3 + 2].Kp = 150.0f;

			cmd.motorCmd[i * 3].Kd = 12.0f;
			cmd.motorCmd[i * 3 + 1].Kd = 12.0f;
			cmd.motorCmd[i * 3 + 2].Kd = 12.0f;

			cmd.motorCmd[i * 3].tau = 0.0f;
			cmd.motorCmd[i * 3 + 1].tau = 0.0f;
			cmd.motorCmd[i * 3 + 2].tau = 0.0f;
		}
	}else if ((motiontime < starttime) && (motiontime < (starttime + standtime))){
		for (int i = 0; i < 4; ++i){
			cmd.motorCmd[i * 3].q = 0.0f;
			cmd.motorCmd[i * 3 + 1].q = 0.0f;
			cmd.motorCmd[i * 3 + 2].q = 0.0f;

			cmd.motorCmd[i * 3].dq = 0.0f;
			cmd.motorCmd[i * 3 + 1].dq = 0.0f;
			cmd.motorCmd[i * 3 + 2].dq = 0.0f;

			cmd.motorCmd[i * 3].Kp = 0.0f;
			cmd.motorCmd[i * 3 + 1].Kp = 0.0f;
			cmd.motorCmd[i * 3 + 2].Kp = 0.0f;

			cmd.motorCmd[i * 3].Kd = 0.0f;
			cmd.motorCmd[i * 3 + 1].Kd = 0.0f;
			cmd.motorCmd[i * 3 + 2].Kd = 0.0f;

			cmd.motorCmd[i * 3].tau = 0.0f;
			cmd.motorCmd[i * 3 + 1].tau = 0.0f;
			cmd.motorCmd[i * 3 + 2].tau = 0.0f;
		}
	}

	// ===================================================== //
	// ================== LL Controller ==================== //
	// ===================================================== //
	if ( motiontime > (starttime + standtime) ){
		if (beginCommand){
			Eigen::Matrix<double, 18, 1> tau_ctrl, dq_ctrl, q_ctrl;
			Eigen::Matrix<double,  6, 1> y_out;
			double Rot[9];
			if (setup){
				Eigen::Matrix<double, 3, 1> com;
				com(0) = q[0];
				com(1) = q[1];
				com(2) = q[2];
				loco_obj->timeSetup(motiontime, motiontime);
				loco_obj->posSetup(com);
				loco_obj->setPoseType(POSE_PITCH);
				loco_obj->compute(q, dq, Rot, STAND, motiontime, 0.0, 0.0, 0.0);
				setup = false;
			}
			if (beginPose == false){
				loco_obj->compute(q, dq, Rot, STAND, motiontime, 0.0, 0.0, 0.0);
			}else{
				loco_obj->compute(q, dq, Rot, POSE, motiontime, 0.0, 0.0, 0.0);
			}

			// Set the command
			tau_ctrl = loco_obj->getJointTorqueCommand();
			dq_ctrl  = loco_obj->getJointVelocityCommand();
			q_ctrl   = loco_obj->getJointPositionCommand();
			y_out    = loco_obj->getHZDOutputs();
			for (int i = 0; i < 12; ++i){
				cmd.motorCmd[i].tau = tau_ctrl(i + 6);
				cmd.motorCmd[i].q = q_ctrl(i + 6);
				cmd.motorCmd[i].dq = dq_ctrl(i + 6);
				cmd.motorCmd[i].Kp = 260;
				cmd.motorCmd[i].Kd = 8;
			}

			// Saturate the command
			float hr_max = 2.0f;
			float hr_min = -2.0f;
			float hp_max = 14.0f;
			float hp_min = -2.2f;
			float k_max = 14.0f;
			float k_min = -2.2f;
			for (int i = 0; i < 4; i++){
				cmd.motorCmd[3 * i].tau = (cmd.motorCmd[3 * i].tau > hr_max) ? hr_max : cmd.motorCmd[3 * i].tau;
				cmd.motorCmd[3 * i].tau = (cmd.motorCmd[3 * i].tau < hr_min) ? hr_min : cmd.motorCmd[3 * i].tau;
				cmd.motorCmd[3 * i + 1].tau = (cmd.motorCmd[3 * i + 1].tau > hp_max) ? hp_max : cmd.motorCmd[3 * i + 1].tau;
				cmd.motorCmd[3 * i + 1].tau = (cmd.motorCmd[3 * i + 1].tau < hp_min) ? hp_min : cmd.motorCmd[3 * i + 1].tau;
				cmd.motorCmd[3 * i + 2].tau = (cmd.motorCmd[3 * i + 2].tau > k_max) ? k_max : cmd.motorCmd[3 * i + 2].tau;
				cmd.motorCmd[3 * i + 2].tau = (cmd.motorCmd[3 * i + 2].tau > k_max) ? k_max : cmd.motorCmd[3 * i + 2].tau;
			}
			for(int i=0; i<18; ++i){
				fprintf(fid,"%lf,",q[i]);
			}
			for(int i=0; i<6; ++i){
				fprintf(fid,"%lf,",y_out[i]);
			}
			fprintf(fid,"\n");

		}else{
			printf("Press Space to Continue\n");
			offset = yaw;
		}
	}

	

	udpComp.Send();
	udpComp.SetSend(cmd);
}

int main(int argc, char *argv[])
{
	ExternalComm extComm;

	extComm.loco_obj = new FastMPC(argc, argv);

	LoopFunc loop_calc("calc_loop", extComm.dt, boost::bind(&ExternalComm::Calc, &extComm));
	loop_calc.start();

	InitEnvironment();
	extComm.udpComp.InitCmdData(extComm.cmd);

	struct input_event ev;
    int k = 1;
    int fd = open("/dev/input/event4", O_RDONLY);
    printf("fd = %d\n", fd);
	int continueLoop = 1;
	while (continueLoop==1){
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
					continueLoop = 0;
					break;
			}
		} 
		sleep(0.1);
	};

	fclose(extComm.fid);
	return 0;
}
