//
// Authror: Randy Fawcett on 12/2021.
//
// Copyright (c) Hybrid Dynamic Systems and Robot Locomotion Lab, Virginia Tech
//
#include "LocoWrapper.hpp"
#include "MPC_dist.hpp"

#include "unitree_legged_sdk/unitree_legged_sdk.h"
#include "unitree_legged_sdk/unitree_joystick.h"
#include "timer.h"
#include "Transforms.hpp"
#include "Filters.h"
#include "OtherUtils.hpp"

#include "stdio.h"
#include "fstream"

#include "shared_structs.hpp"

using namespace UNITREE_LEGGED_SDK;

bool turn_off_both = false;
bool soft_fall_both = false;
bool start_both = false;

class ExternalComm0 {
public:
	 	ExternalComm0() : udpComp0(8019, "10.0.0.20", 8020, sizeof(LowCmd), sizeof(LowState)){
		// ExternalComm0() : udpComp0(LOWLEVEL) {
		// udpComp0.InitCmdData(cmd);
		// 60 Hz		udpComp0(LOWLEVEL)

		double ad[3] = {1.0, -1.47548044359265, 0.58691950806119};
		double bd[3] = {0.02785976611714, 0.05571953223427, 0.02785976611714};
		populate_filter_d(jointfilter,ad,bd,3,12);

		// 0.75 Hz
		float af[3] = {1.00000000,-1.99333570,0.99335783};
		float bf[3] = {0.00000553,0.00001107,0.00000553};
		populate_filter_f(remotefilter, af, bf, 3, 2);

		// 2 Hz 
		float aa[3] = {1.00000000,-1.98222893,0.98238545};
		float ba[3] = {0.00003913,0.00007826,0.00003913};
		populate_filter_f(angfilter, aa, ba, 3, 2);
		
	}

	virtual ~ExternalComm0(){
		clear_filter_d(jointfilter);
		clear_filter_f(remotefilter);
		clear_filter_f(angfilter);
	}

	// for first agent
	void Calc();
	void HighLevel();

	std::unique_ptr<LocoWrapper> loco_obj;
	std::unique_ptr<MPC_dist> mpc_obj;
	UDP udpComp0;
	FILE *fid;

	long motiontime = 0;
	bool softFall = false;
	bool beginCommand = false;
	bool beginPose = false;
	bool setup = true;
	double q[18] = {0.0};
	double dq[18] = {0.0};
	double tauEst[12] = {0.0};

	FiltStruct_d* jointfilter  = (FilterStructure_d*)malloc(sizeof(FilterStructure_d));
	FiltStruct_f* angfilter    = (FilterStructure_f*)malloc(sizeof(FilterStructure_f));
	FiltStruct_f* remotefilter = (FilterStructure_f*)malloc(sizeof(FilterStructure_f));

	LowState state = {0};
	LowCmd cmd = {0};
	xRockerBtnDataStruct remote;

	int stop = 0;
	float vel[3] = {0};
	float pose[6] = {0};
	float ang[2] = {0};
	int actCon[4] = {0};

	float dt = 0.001f;

};

class ExternalComm1 {
public:
	 ExternalComm1() : udpComp0(8021, "10.0.0.30", 8022, sizeof(LowCmd), sizeof(LowState)){
		// 60 Hz		udpComp0(LOWLEVEL)
		// ExternalComm() : udpComp0(8019, "10.0.0.20", 8020, sizeof(LowCmd), sizeof(LowState)){

		double ad[3] = {1.0, -1.47548044359265, 0.58691950806119};
		double bd[3] = {0.02785976611714, 0.05571953223427, 0.02785976611714};
		populate_filter_d(jointfilter,ad,bd,3,12);

		// 0.75 Hz
		float af[3] = {1.00000000,-1.99333570,0.99335783};
		float bf[3] = {0.00000553,0.00001107,0.00000553};
		populate_filter_f(remotefilter, af, bf, 3, 2);

		// 2 Hz 
		float aa[3] = {1.00000000,-1.98222893,0.98238545};
		float ba[3] = {0.00003913,0.00007826,0.00003913};
		populate_filter_f(angfilter, aa, ba, 3, 2);
		
	}

	virtual ~ExternalComm1(){
		clear_filter_d(jointfilter);
		clear_filter_f(remotefilter);
		clear_filter_f(angfilter);
	}

	// for first agent
	void Calc();
	void HighLevel();

	std::unique_ptr<LocoWrapper> loco_obj;
	std::unique_ptr<MPC_dist> mpc_obj;
	UDP udpComp0;
	FILE *fid;

	long motiontime = 0;
	bool softFall = false;
	bool beginCommand = false;
	bool beginPose = false;
	bool setup = true;
	double q[18] = {0.0};
	double dq[18] = {0.0};
	double tauEst[12] = {0.0};

	FiltStruct_d* jointfilter  = (FilterStructure_d*)malloc(sizeof(FilterStructure_d));
	FiltStruct_f* angfilter    = (FilterStructure_f*)malloc(sizeof(FilterStructure_f));
	FiltStruct_f* remotefilter = (FilterStructure_f*)malloc(sizeof(FilterStructure_f));

	LowState state = {0};
	LowCmd cmd = {0};
	xRockerBtnDataStruct remote;

	int stop = 0;
	float vel[3] = {0};
	float pose[6] = {0};
	float ang[2] = {0};
	int actCon[4] = {0};

	float dt = 0.001f;

};

void ExternalComm0::HighLevel(){
	timer tset;
	tic(&tset);
	// printf("%f\n",1000*toc(&tset));
	static sharedData HLData0;

	// Get the data for the high level
	updateData(GET_DATA, HL_DATA, &HLData0, 0);
	// updateData0(GET_DATA, HL_DATA, &HLData0);
	mpc_obj->updateState(HLData0.q, HLData0.dq, HLData0.ind, HLData0.toePos, HLData0.last_state1);

	// Update the MPC
	if (HLData0.runMPC)
	{
		// mpc_obj->updateState(HLData.q, HLData.dq);
		// std::cout << "HLData.q = " << HLData.q[0] << "     HLData.q = " << HLData.q[1] << std::endl;
		// std::cout << "Bezier Fit:\n" << HLData.alpha_COM << "\n";
		// mpc_obj->plannedCycleIndex(TROT); 
		mpc_obj->lipMPC_eventbase();

		HLData0.alpha_COM = mpc_obj->get_alphaCOM();
		HLData0.MPC_sol_ = mpc_obj->get_MPCsol();
		HLData0.domain = mpc_obj->getDomain();
		HLData0.last_state0 = mpc_obj->get_lastState();
		
		HLData0.runMPC = 0;
		HLData0.MPC_data_available = 1;
		HLData0.resetRun = -1;
		updateData(SET_DATA, HL_DATA, &HLData0, 0);
		// updateData0(SET_DATA, HL_DATA, &HLData0);
	}
	printf("%f\n",1000*toc(&tset));
}

void ExternalComm0::Calc() {
	static sharedData LLData0;
	updateData(GET_DATA, LL_DATA, &LLData0, 0);
	// updateData0(GET_DATA, LL_DATA, &LLData0);
	//	timer tset;
	//	tic(&tset);
	
	udpComp0.Recv();
	udpComp0.GetRecv((char*)&state);
	motiontime += 1;
	int standtime = 2000;
	int starttime = 1000;

	// ===================================================== //
	// ============== Wireless Remote Stuff  =============== //
	// ===================================================== //
	memcpy(&remote, state.wirelessRemote, 40);
	if((int)remote.btn.components.B!=0 && (int)remote.btn.components.R2!=0){
		stop = 1;
		cmd.reserve = 0;
	}
	if((int)remote.btn.components.B!=0 && (int)remote.btn.components.L2!=0){
		softFall = true;
		// soft_fall_both = true;
	}
	if( ((int)remote.btn.components.start!=0) && ((int)remote.btn.components.L1!=0) ){
		beginCommand = 1;
		cmd.reserve = 1;
		// start_both = true;
	}
	vel[0] = 0.75f*remote.ly; // x vel
	vel[1] = -0.4f*remote.rx; // y vel
	ang[0] = 20.0f*3.14f/180.0f*remote.ry; // yaw vel
	ang[1] = -2.0f*remote.lx; // pitch pos
	discrete_butter_f(remotefilter,vel);
	discrete_butter_f(angfilter,ang);
	pose[4] = ang[0]; // set filtered pose
	vel[2] = ang[1];  // set filtered ang vel

	// ===================================================== //
	// ================= Update the State ================== //
	// ===================================================== //
	for (int i=0; i<12; ++i){
		q[i+6] = state.motorState[i].q;
		dq[i+6] = state.motorState[i].dq;
	}
	discrete_butter_d(jointfilter,&dq[6]);

	q[3] = state.imu.rpy[0]; q[4] = state.imu.rpy[1]; q[5] = state.imu.rpy[2];
	dq[3] = state.imu.gyroscope[0]; dq[4] = state.imu.gyroscope[1]; dq[5] = state.imu.gyroscope[2];
	
	quat_to_XYZ(state.imu.quaternion[0],state.imu.quaternion[1],state.imu.quaternion[2],state.imu.quaternion[3],
	            q[3],q[4],q[5]);
	q[4] += 1.8*MY_PI/180.0;
	//dq[5] += 5.0*MY_PI/180.0;

	double *rotMat;
	Eigen::Matrix3d R;
	quat_to_R(state.imu.quaternion,R);
	rotMat = R.data();

	// Kinematic Estimation
	int footForce[4];
	footForce[0] = state.footForce[0]; footForce[1] = state.footForce[1];
	footForce[2] = state.footForce[2]; footForce[3] = state.footForce[3];
	static int contactIndex[4] = {1,1,1,1};
	kinEst0(footForce,contactIndex,q,dq,R); // Defined in OtherUtils.hpp
	
	// ===================================================== //
	// ============= Quad Initialization Time ============== //
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
			double phaseVar;
			if (setup){
				Eigen::Matrix<double, 3, 1> com;
				com(0) = q[0], com(1) = q[1], com(2) = q[2];
				endStand = motiontime+standtime;
				loco_obj->initStandVars(com,standtime);
				setup = false;
			}
			
			beginPose = (motiontime<endStand) ? 0 : 1;
			if (beginPose == 0){
				loco_obj->set_MPC_DATA(LLData0.alpha_COM, LLData0.MPC_sol_, LLData0.MPC_data_available);
				// Calculate the torque
				loco_obj->calcTau(q, dq, rotMat, footForce, STAND, motiontime, &(LLData0.runMPC));		//runMPC pointer not passed in the last argument
			}else if (beginPose == 1 && motiontime>=endStand){
				loco_obj->set_MPC_DATA(LLData0.alpha_COM, LLData0.MPC_sol_, LLData0.MPC_data_available);
				// Calculate the torque
				loco_obj->calcTau(q, dq, rotMat, footForce, TROT, motiontime, &(LLData0.runMPC));		//runMPC pointer not passed in the last argument


				const int* contactMat = loco_obj->getConDes();
				
				memcpy(contactIndex,contactMat,4*sizeof(int));
			}
			LLData0.MPC_data_available = loco_obj->get_MPC_data_available();
			LLData0.toePos = loco_obj->get_toePos();

			memcpy(LLData0.ind,contactIndex,4*sizeof(int));
			if(LLData0.resetRun == -1){
				LLData0.resetRun = -2;
			}

			// Set the command
			for (int i = 0; i < 12; ++i){
				cmd.motorCmd[i].tau = loco_obj->ll->tau[i + 6];
				cmd.motorCmd[i].q  = loco_obj->ll->q(i + 6);
				cmd.motorCmd[i].dq = loco_obj->ll->dq(i + 6);

				cmd.motorCmd[i].Kp = 10;
				cmd.motorCmd[i].Kd = 8;
				if(beginPose==true){
					for(int i=0; i<4; ++i){
						if(contactIndex[i]==0){
			//							cmd.motorCmd[i].tau = 0;
							cmd.motorCmd[3*i].Kp = 20;
							cmd.motorCmd[3*i+1].Kp = 20;
							cmd.motorCmd[3*i+2].Kp = 20;

							cmd.motorCmd[3*i].Kd = 1;
							cmd.motorCmd[3*i+1].Kd = 1;
							cmd.motorCmd[3*i+2].Kd = 1;  
						}
					}	
				}
			}

			// Saturate the command
			float hr_max = 10.0f, hr_min = -10.0f;
			float hp_max = 30.0f, hp_min = -30.0f;
			float kn_max = 33.0f, kn_min = -33.0f;
			for (int i = 0; i < 4; i++){
				cmd.motorCmd[3 * i + 0].tau = (cmd.motorCmd[3 * i + 0].tau > hr_max) ? hr_max : cmd.motorCmd[3 * i + 0].tau;
				cmd.motorCmd[3 * i + 0].tau = (cmd.motorCmd[3 * i + 0].tau < hr_min) ? hr_min : cmd.motorCmd[3 * i + 0].tau;
				cmd.motorCmd[3 * i + 1].tau = (cmd.motorCmd[3 * i + 1].tau > hp_max) ? hp_max : cmd.motorCmd[3 * i + 1].tau;
				cmd.motorCmd[3 * i + 1].tau = (cmd.motorCmd[3 * i + 1].tau < hp_min) ? hp_min : cmd.motorCmd[3 * i + 1].tau;
				cmd.motorCmd[3 * i + 2].tau = (cmd.motorCmd[3 * i + 2].tau > kn_max) ? kn_max : cmd.motorCmd[3 * i + 2].tau;
				cmd.motorCmd[3 * i + 2].tau = (cmd.motorCmd[3 * i + 2].tau < kn_min) ? kn_min : cmd.motorCmd[3 * i + 2].tau;
			}

		}else if ( softFall ) {
			for (int i = 0; i < 12; ++i){
				cmd.motorCmd[i].tau = 0.0;
				cmd.motorCmd[i].q   = 0.0;
				cmd.motorCmd[i].dq  = 0.0;

				cmd.motorCmd[i].Kp = 0;
				cmd.motorCmd[i].Kd = ( ( (i+1)%3 ) == 0 ) ? 6 : 3;
			}
		}else if ( (!beginCommand) && (!softFall) ){
			static bool printedA = false;
			if(!printedA){
				printf("\nPress L1 + start to continue\n");
				printedA = true;
			}
		}
	}

	const int* contactMat = loco_obj->getConDes();
	memcpy(LLData0.q,q,18*sizeof(double));
    memcpy(LLData0.dq,dq,18*sizeof(double));
	memcpy(LLData0.ind, contactMat,4*sizeof(int));

	//	printf("%f\n",1000*toc(&tset));
	updateData(SET_DATA, LL_DATA, &LLData0, 0);
	// updateData0(SET_DATA, LL_DATA, &LLData0);
	udpComp0.SetSend((char*)&cmd);
	udpComp0.Send();
}

void ExternalComm1::HighLevel(){
	timer tset;
	tic(&tset);
	// printf("%f\n",1000*toc(&tset));
	static sharedData HLData1;

	// Get the data for the high level
	updateData(GET_DATA, HL_DATA, &HLData1, 1);
	// updateData1(GET_DATA, HL_DATA, &HLData1);

	
	mpc_obj->updateState(HLData1.q, HLData1.dq, HLData1.ind, HLData1.toePos, HLData1.last_state0);

	// Update the MPC
	if (HLData1.runMPC)
	{
		// mpc_obj->updateState(HLData.q, HLData.dq);
		// std::cout << "HLData.q = " << HLData.q[0] << "     HLData.q = " << HLData.q[1] << std::endl;
		// std::cout << "Bezier Fit:\n" << HLData.alpha_COM << "\n";
		// mpc_obj->plannedCycleIndex(TROT);
		mpc_obj->lipMPC_eventbase();

		HLData1.alpha_COM = mpc_obj->get_alphaCOM();
		HLData1.MPC_sol_ = mpc_obj->get_MPCsol();
		HLData1.domain = mpc_obj->getDomain();
		HLData1.last_state1 = mpc_obj->get_lastState();
		
		HLData1.runMPC = 0;
		HLData1.MPC_data_available = 1;
		HLData1.resetRun = -1;
		updateData(SET_DATA, HL_DATA, &HLData1, 1);
		// updateData1(SET_DATA, HL_DATA, &HLData1);
	}
	printf("%f\n",1000*toc(&tset));
}

void ExternalComm1::Calc() {
	static sharedData LLData1;
	updateData(GET_DATA, LL_DATA, &LLData1, 1);
	// updateData1(GET_DATA, LL_DATA, &LLData1);cmd

	// udpComp0.GetRecv(state);
	udpComp0.Recv();
	udpComp0.GetRecv((char*)&state);

	motiontime += 1;
	int standtime = 2000;
	int starttime = 1000;

	// if (soft_fall_both)
	// {
	// 	softFall = true;
	// }
	// if (start_both)
	// {
	// 	beginCommand = 1;
	// 	cmd.reserve = 1;
	// }

	// ===================================================== //
	// ============== Wireless Remote Stuff  =============== //
	// ===================================================== //
	memcpy(&remote, state.wirelessRemote, 40);
	if((int)remote.btn.components.B!=0 && (int)remote.btn.components.R2!=0){
		stop = 1;
		cmd.reserve = 0;
	}
	if((int)remote.btn.components.B!=0 && (int)remote.btn.components.L2!=0){
		softFall = true;
	}
	if( ((int)remote.btn.components.start!=0) && ((int)remote.btn.components.L1!=0) ){
		beginCommand = 1;
		cmd.reserve = 1;
	}
	vel[0] = 0.75f*remote.ly; // x vel
	vel[1] = -0.4f*remote.rx; // y vel
	ang[0] = 20.0f*3.14f/180.0f*remote.ry; // yaw vel
	ang[1] = -2.0f*remote.lx; // pitch pos
	discrete_butter_f(remotefilter,vel);
	discrete_butter_f(angfilter,ang);
	pose[4] = ang[0]; // set filtered pose
	vel[2] = ang[1];  // set filtered ang vel

	// ===================================================== //
	// ================= Update the State ================== //
	// ===================================================== //
	for (int i=0; i<12; ++i){
		q[i+6] = state.motorState[i].q;
		dq[i+6] = state.motorState[i].dq;
	}
	discrete_butter_d(jointfilter,&dq[6]);

	q[3] = state.imu.rpy[0]; q[4] = state.imu.rpy[1]; q[5] = state.imu.rpy[2];
	dq[3] = state.imu.gyroscope[0]; dq[4] = state.imu.gyroscope[1]; dq[5] = state.imu.gyroscope[2];
	
	quat_to_XYZ(state.imu.quaternion[0],state.imu.quaternion[1],state.imu.quaternion[2],state.imu.quaternion[3],
	            q[3],q[4],q[5]);
	q[4] += 0.0*MY_PI/180.0;

	double *rotMat;
	Eigen::Matrix3d R;
	quat_to_R(state.imu.quaternion,R);
	rotMat = R.data();
	// updateData(SET_DATA, LL_DATA, &LLData1, 1);
	// updateData1(SET_DATA, LL_DATA, &LLData1);
	// Kinematic Estimation
	int footForce[4];
	footForce[0] = state.footForce[0]; footForce[1] = state.footForce[1];
	footForce[2] = state.footForce[2]; footForce[3] = state.footForce[3];
	static int contactIndex[4] = {1,1,1,1};
	kinEst1(footForce,contactIndex,q,dq,R); // Defined in OtherUtils.hpp
	
	// ===================================================== //
	// ============= Quad Initialization Time ============== //
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
			double phaseVar;
			if (setup){
				Eigen::Matrix<double, 3, 1> com;
				com(0) = q[0], com(1) = q[1], com(2) = q[2];
				endStand = motiontime+standtime;
				loco_obj->initStandVars(com,standtime);
				setup = false;
			}
			
			beginPose = (motiontime<endStand) ? 0 : 1;
			if (beginPose == 0){
				loco_obj->set_MPC_DATA(LLData1.alpha_COM, LLData1.MPC_sol_, LLData1.MPC_data_available);
				// Calculate the torque
				loco_obj->calcTau(q, dq, rotMat, footForce, STAND, motiontime, &(LLData1.runMPC));		//runMPC pointer not passed in the last argument
			}else if (beginPose == 1 && motiontime>=endStand){
				loco_obj->set_MPC_DATA(LLData1.alpha_COM, LLData1.MPC_sol_, LLData1.MPC_data_available);
				// Calculate the torque
				loco_obj->calcTau(q, dq, rotMat, footForce, TROT, motiontime, &(LLData1.runMPC));		//runMPC pointer not passed in the last argument


				const int* contactMat = loco_obj->getConDes();
				
				memcpy(contactIndex,contactMat,4*sizeof(int));
			}
			LLData1.MPC_data_available = loco_obj->get_MPC_data_available();
			LLData1.toePos = loco_obj->get_toePos();

			memcpy(LLData1.ind,contactIndex,4*sizeof(int));
			if(LLData1.resetRun == -1){
				LLData1.resetRun = -2;
			}

			// Set the command
			for (int i = 0; i < 12; ++i){
				cmd.motorCmd[i].tau = loco_obj->ll->tau[i + 6];
				cmd.motorCmd[i].q  = loco_obj->ll->q(i + 6);
				cmd.motorCmd[i].dq = loco_obj->ll->dq(i + 6);

				cmd.motorCmd[i].Kp = 10;
				cmd.motorCmd[i].Kd = 8;
				if(beginPose==true){
					for(int i=0; i<4; ++i){
						if(contactIndex[i]==0){
			//							cmd.motorCmd[i].tau = 0;
							cmd.motorCmd[3*i].Kp = 20;
							cmd.motorCmd[3*i+1].Kp = 20;
							cmd.motorCmd[3*i+2].Kp = 20;

							cmd.motorCmd[3*i].Kd = 1;
							cmd.motorCmd[3*i+1].Kd = 1;
							cmd.motorCmd[3*i+2].Kd = 1;  
						}
					}	
				}
			}

			// Saturate the command
			float hr_max = 10.0f, hr_min = -10.0f;
			float hp_max = 30.0f, hp_min = -30.0f;
			float kn_max = 33.0f, kn_min = -33.0f;
			for (int i = 0; i < 4; i++){
				cmd.motorCmd[3 * i + 0].tau = (cmd.motorCmd[3 * i + 0].tau > hr_max) ? hr_max : cmd.motorCmd[3 * i + 0].tau;
				cmd.motorCmd[3 * i + 0].tau = (cmd.motorCmd[3 * i + 0].tau < hr_min) ? hr_min : cmd.motorCmd[3 * i + 0].tau;
				cmd.motorCmd[3 * i + 1].tau = (cmd.motorCmd[3 * i + 1].tau > hp_max) ? hp_max : cmd.motorCmd[3 * i + 1].tau;
				cmd.motorCmd[3 * i + 1].tau = (cmd.motorCmd[3 * i + 1].tau < hp_min) ? hp_min : cmd.motorCmd[3 * i + 1].tau;
				cmd.motorCmd[3 * i + 2].tau = (cmd.motorCmd[3 * i + 2].tau > kn_max) ? kn_max : cmd.motorCmd[3 * i + 2].tau;
				cmd.motorCmd[3 * i + 2].tau = (cmd.motorCmd[3 * i + 2].tau < kn_min) ? kn_min : cmd.motorCmd[3 * i + 2].tau;
			}

		}else if ( softFall ) {
			for (int i = 0; i < 12; ++i){
				cmd.motorCmd[i].tau = 0.0;
				cmd.motorCmd[i].q   = 0.0;
				cmd.motorCmd[i].dq  = 0.0;

				cmd.motorCmd[i].Kp = 0;
				cmd.motorCmd[i].Kd = ( ( (i+1)%3 ) == 0 ) ? 6 : 3;
			}
		}else if ( (!beginCommand) && (!softFall) ){
			static bool printedA = false;
			if(!printedA){
				printf("\nPress L1 + start to continue\n");
				printedA = true;
			}
		}
	}

	const int* contactMat = loco_obj->getConDes();
	memcpy(LLData1.q,q,18*sizeof(double));
    memcpy(LLData1.dq,dq,18*sizeof(double));
	memcpy(LLData1.ind, contactMat,4*sizeof(int));

	//	printf("%f\n",1000*toc(&tset));
	updateData(SET_DATA, LL_DATA, &LLData1, 1);
	// updateData1(SET_DATA, LL_DATA, &LLData1);
	udpComp0.SetSend((char*)&cmd);
	udpComp0.Send();
}

int main(int argc, char *argv[])
{
	InitEnvironment();
	ExternalComm0 extComm0;
	ExternalComm1 extComm1;

	Eigen::Matrix<double, 2,9> Pobs;
	Eigen::Matrix<double, 2*NUMBER_OF_AGENTS, 1> Pstart;

	// Pstart << 0.0, 0.0, 0, -1.5;//, 1.0, -2, 1, -3;	//2.0, -2.0
	
    Pstart << 0.0, 0.0, 0.0, -0.9; //, 1.0, -2, 1, -3;	//2.0, -2.0
    // Pobs   <<   1,   1,   1.5,   1.5,      2,        2,   2.5,     2.5,  1,
    //             1,  -2,     1,    -2,   0.5,    -1.5,     0.0,    -1.0,    -0.5;
	// Exp 01 t0 Exp 04
    // Pobs   <<      5,       2,      2,             2,           2,              4,             4,     2.5,       -100,
    //                1,       1,     -1,             2,          -2,              0,            -2,    -1.0 + 100,    -0.5 + 100; 

	// EXP 03 
    Pobs   <<       3.2,       2,      2,             2,           2,                4,             4,     2.5,       -100,
                    0.9,       1,     -1,             2,          -2,              0.2,            -2,    -1.0 + 100,    -0.5 + 100; 
	
	// EXP04
	// Pobs   <<   1-0.3,       1.5-0.3,      2-0.3,             2.5-0.3,             3-0.3,              3.2-0.3,             5.2-0.3,       5-0.3,       -100,
	// 			2,               1.5,          1,                 0.5,              -0.0,             -1.0,                -1.5,            -2.5,     -0.5 + 100; 

	// EXP5
	// Pobs   <<   3,       3,      3,             3,             3,              3,           5.2,     4.9,      4,
	// 		   -2,    -1.5,      1,           0.1,             2,             -3,          -0.9,    -0.1,    0.1;

	extComm0.loco_obj = std::unique_ptr<LocoWrapper>(new LocoWrapper(argc, argv));
	extComm0.mpc_obj    = std::unique_ptr<MPC_dist>(new MPC_dist());

	extComm0.loco_obj->setAgentID(0); //Agent ID = 0
    extComm0.loco_obj->setPstart(Pstart);
    extComm0.loco_obj->setPobs(Pobs);
    extComm0.loco_obj->generateReferenceTrajectory();

	extComm0.mpc_obj->setAgentID(0);    //MPC_Agent1 = new MPC_dist();
    extComm0.mpc_obj->setPstart(Pstart);
    extComm0.mpc_obj->setPobs(Pobs);
    extComm0.mpc_obj->generateReferenceTrajectory();

	// SECOND AGENT

	extComm1.loco_obj = std::unique_ptr<LocoWrapper>(new LocoWrapper(argc, argv));
	extComm1.mpc_obj    = std::unique_ptr<MPC_dist>(new MPC_dist());

	extComm1.loco_obj->setAgentID(1); //Agent ID = 0
    extComm1.loco_obj->setPstart(Pstart);
    extComm1.loco_obj->setPobs(Pobs);
    extComm1.loco_obj->generateReferenceTrajectory();

	extComm1.mpc_obj->setAgentID(1);    //MPC_Agent1 = new MPC_dist();
    extComm1.mpc_obj->setPstart(Pstart);
    extComm1.mpc_obj->setPobs(Pobs);
    extComm1.mpc_obj->generateReferenceTrajectory();


	data0.alpha_COM.setZero(4,5);
	data1.alpha_COM.setZero(4,5);

	
	LoopFunc loop_calc0("calc_loop0", extComm0.dt,1, boost::bind(&ExternalComm0::Calc, &extComm0));
	LoopFunc loop_mpc0("mpc_loop0", 0.020001f,2, boost::bind(&ExternalComm0::HighLevel, &extComm0));
	LoopFunc loop_calc1("calc_loop1", extComm1.dt,3, boost::bind(&ExternalComm1::Calc, &extComm1));
	LoopFunc loop_mpc1("mpc_loop1", 0.020001f,2, boost::bind(&ExternalComm1::HighLevel, &extComm1));

	// LoopFunc loop_calc0("calc_loop0", extComm0.dt, boost::bind(&ExternalComm0::Calc, &extComm0));
	// LoopFunc loop_mpc0("mpc_loop0", 0.020001f, boost::bind(&ExternalComm0::HighLevel, &extComm0));
	// LoopFunc loop_calc1("calc_loop1", extComm1.dt, boost::bind(&ExternalComm1::Calc, &extComm1));
	// LoopFunc loop_mpc1("mpc_loop1", 0.020001f, boost::bind(&ExternalComm1::HighLevel, &extComm1));

	loop_calc0.start();
	loop_calc1.start();
	loop_mpc0.start();
	loop_mpc1.start();
	
	extComm0.udpComp0.InitCmdData(extComm0.cmd);
	extComm1.udpComp0.InitCmdData(extComm1.cmd);
	
	while ( (extComm0.stop==0) ){
		sleep(0.1);
	};

	return 0;
}

	// EXP B
	// Pstart << 0.0, 0.0, 0, -1, 1.0, -2, 1, -3;	//2.0, -2.0
	// Pobs   <<   1,   1,      2.5,   2.5, -100, -100, -100, -100, -100,
	// 		0,  -1,    -0.35,  0.65,    0,    0,    0,    0,    0; 

	// EXP C
	// Pstart << 0.0, 0.0, 0, -0.5;//, 1.0, -2, 1, -3;	//2.0, -2.0
    // Pobs   <<   1,   1,   1.5,   1.5,      2,        2,   2.5,     2.5,  3.5,
                // 1,  -2,     1,    -2,   0.5,    -1.5,     0.0,    -1.0,    -0.5; 