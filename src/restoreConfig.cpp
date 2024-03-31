/************************************************************************
Copyright (c) 2020, Unitree Robotics.Co.Ltd. All rights reserved.
Use of this source code is governed by the MPL-2.0 license, see LICENSE.
************************************************************************/
#include <stdint.h>
#include "unitree_legged_sdk/unitree_legged_sdk.h"
#include "fstream"
#define M_PI 3.14159265359

using namespace UNITREE_LEGGED_SDK;

class ExternalComm
{
public:
	ExternalComm() : udpComp(LOWLEVEL){}

	void Calc();

	UDP udpComp;

	long motiontime = 0;
	float qInit[12] = {0};
	double q[18] = {0.0};
	double dq[18] = {0.0};
	
	int starttime = 500;
	int standtime = 1000;

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
	motiontime += 2;

	// ===================================================== //
	// =================== Tuck Legs Up ==================== //
	// ===================================================== //
	if (motiontime <= starttime){
		for (int i = 0; i < 12; ++i){
			qInit[i] = state.motorState[i].q;
		}
	}
	
	double currenttime = 1.0 * (motiontime - starttime) / standtime;
	if ((motiontime >= starttime) && (motiontime <= (starttime + standtime))){
		for (int i = 0; i < 4; ++i){
			cmd.motorCmd[i * 3].q = jointLinearInterpolation(qInit[3 * i], 0, currenttime);
			cmd.motorCmd[i * 3 + 1].q = jointLinearInterpolation(qInit[3 * i + 1], 1.310163, currenttime);
			cmd.motorCmd[i * 3 + 2].q = jointLinearInterpolation(qInit[3 * i + 2], -2.525674, currenttime);
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

	udpComp.Send();
	udpComp.SetSend(cmd);
}


int main(int argc, char *argv[])
{
	ExternalComm extComm;

	LoopFunc loop_calc("calc_loop", extComm.dt, boost::bind(&ExternalComm::Calc, &extComm));
	loop_calc.start();

	InitEnvironment();
	extComm.udpComp.InitCmdData(extComm.cmd);


	while (1){
		if(extComm.motiontime>extComm.starttime+extComm.standtime){
			printf("\nA1 is back in the nominal position\n\n");
			break;
		}
		sleep(0.1);
	};


	return 0;
}




