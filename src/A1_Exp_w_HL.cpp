#include "A1_Dynamics.h"
#include "kalman.hpp"
#include "LocoWrapper.hpp"
#include "multi_pc_comm.h"
#include "timer.h"

#include "stdio.h"
#include "fcntl.h"
#include "unistd.h"

#include "linux/input.h"
#include "sys/stat.h"
#include "fstream"

#include "remoteCtrl.h"

#include "shared_structs.hpp"
#include "forceUpdate.hpp"
#include "mutex"

#include <boost/thread/mutex.hpp>
#include <boost/thread/locks.hpp>



#define SET_DATA 1
#define GET_DATA 0
#define HL_DATA 1
#define LL_DATA 0
sharedData data;
boost::mutex mtx;
void updateData(int setget, int highlow, sharedData *newData){
	// set=1,  get=0
	// high=1, low=0
	boost::lock_guard<boost::mutex> guard(mtx);
	if (setget==SET_DATA){
		if (highlow==HL_DATA){ // set high level data
			data.fDes = newData->fDes;
		}else{ // set low level data
			Eigen::Matrix<double, 12, 1> fDes_temp = data.fDes;
			memcpy(&data,newData,sizeof(sharedData));
			data.fDes = fDes_temp;
		}
	}else {
		if (highlow==HL_DATA){ // get data for high level
			Eigen::Matrix<double, 12, 1> fDes_temp = newData->fDes;
			memcpy(newData,&data,sizeof(sharedData));
			newData->fDes = fDes_temp;
		}else{ // get data for low level
			newData->fDes = data.fDes;
		}
	}
};


using namespace UNITREE_LEGGED_SDK;

class ExternalComm
{
public:
	ExternalComm() : udpComp(LOWLEVEL){
//		fid = fopen("/home/kaveh/A1_exp_data/stateData_0003.csv", "w");
	}

	void Calc();
	void MPC();

	std::unique_ptr<LocoWrapper> loco_obj;
	std::unique_ptr<KF> est_obj;
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
	float dqHist[3][12] = {{0.0f}}; // first row is most recent
	float dqFilt[3][12] = {{0.0f}}; // first row is most recent
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

static void quat_to_XYZ(float qw, float qx, float qy, float qz, float &roll, float &pitch, float &yaw){
	double aSinInput = 2*(qx*qz + qy*qw);
    roll = atan2( -2*(qy*qz-qx*qw), pow(qw,2)-pow(qx,2)-pow(qy,2)+pow(qz,2) );
    pitch = asin( aSinInput );
    yaw = atan2( -2*(qx*qy - qz*qw), pow(qw,2) + pow(qx,2) - pow(qy,2) - pow(qz,2) );
};

void ExternalComm::MPC(){
	static sharedData HLData;
	// Get most recent data
	updateData(GET_DATA, HL_DATA, &HLData);
	
	// Update the desired force
	Eigen::Matrix<double, 6, 1> accDes = Eigen::Matrix<double, 6, 1>::Zero();
	Eigen::MatrixXd fDes;
	HighLevel::updateDesiredForce(accDes, fDes, &HLData);
	HLData.fDes = fDes;
	
	std::cout << HLData.QP_force.transpose()<<std::endl;
	std::cout << HLData.fDes.transpose() << std::endl << std::endl;
	std::cout << fDes.transpose() << std::endl << std::endl;
	
	// Set new force desired
	updateData(SET_DATA, HL_DATA, &HLData);
};

void ExternalComm::Calc()
{
//	timer tset;
//	tic(&tset);
	static sharedData LLData;
	// Get most recent force desired
	updateData(GET_DATA, LL_DATA, &LLData);
	loco_obj->updateDesiredForce(LLData.fDes);

	
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
	double a1 = -1.47548044359265, a2 = 0.58691950806119;
	double b0 = 0.02785976611714, b1 = 0.05571953223427, b2 = 0.02785976611714;

	for(int i=0; i<12; ++i){
		for(int j=2; j>0; --j){
			dqFilt[j][i] = dqFilt[j-1][i];
			dqHist[j][i] = dqHist[j-1][i];
		}
		dqHist[0][i] = state.motorState[i].dq;
	}	
	
	for(int i=0; i<12; ++i){
		dqFilt[0][i] = (b0*dqHist[0][i]+b1*dqHist[1][i]+b2*dqHist[2][i])-(a1*dqFilt[1][i]+a2*dqFilt[2][i]);
	}

	for (int i=0; i<12; ++i){
		q[i+6] = state.motorState[i].q;
//		dq[i+6] = state.motorState[i].dq;
		dq[i+6] = dqFilt[0][i];
	}


	
	q[3] = state.imu.rpy[0]; q[4] = state.imu.rpy[1]; q[5] = state.imu.rpy[2];
	dq[3] = state.imu.gyroscope[0]; dq[4] = state.imu.gyroscope[1]; dq[5] = state.imu.gyroscope[2];
	
	
	quat_to_XYZ(state.imu.quaternion[0],state.imu.quaternion[1],state.imu.quaternion[2],state.imu.quaternion[3],
	            q[3],q[4],q[5]);
	q[4] += 2.2*MY_PI/180.0;

	// ================================== //
	// ========= Con Estimator ========== //
	// ================================== //
	int footForce[4];
	footForce[0] = state.footForce[0]; footForce[1] = state.footForce[1];
	footForce[2] = state.footForce[2]; footForce[3] = state.footForce[3];
	
	
	static int contactIndex[4] = {1,1,1,1};
	int thresh = 20;
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

	// ================================== //
	// ========= Kin Estimator ========== //
	// ================================== //

	// toe pos
	double fr_toe[3], fl_toe[3], rl_toe[3], rr_toe[3];
	static double COM[3]= {0,0,0};
	q[0] = 0; q[1] = 0; q[2] = 0;
	FK_FR_toe(fr_toe, q); FK_FL_toe(fl_toe, q);
	FK_RR_toe(rr_toe, q); FK_RL_toe(rl_toe, q);
	
	// update change in com pos
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
	est_obj->updateKalman(contactIndex,state.imu.accelerometer,state.imu.quaternion,relVec); // Using Quaternions
	double *comPosVel, *rotMat, *footPos;
	rotMat = est_obj->getRotMat();
	
	// This part is temporary until the kalman filter is done
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
				loco_obj->calcTau(q, dq, rotMat, footForce, STAND, motiontime);
				if (motiontime >= endStand){
					static bool printedX = false;
					if (!printedX){
						printf("\nPress X to continue\n");
						printedX = true;
					}
				}
			}else if (beginPose == 1 && motiontime>=endStand){
				loco_obj->calcTau(q, dq, rotMat, footForce, TROT, motiontime);
				const int* contactMat = loco_obj->getConDes();
				contactIndex[0] = contactMat[0]; contactIndex[1] = contactMat[1];
				contactIndex[2] = contactMat[2]; contactIndex[3] = contactMat[3];
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
			float hr_max = 8.0f,  hr_min = -8.0f;
			float hp_max = 25.0f, hp_min = -20.0f;
			float kn_max = 30.0f, kn_min = -30.0f;
			for (int i = 0; i < 4; i++){
				cmd.motorCmd[3 * i + 0].tau = (cmd.motorCmd[3 * i + 0].tau > hr_max) ? hr_max : cmd.motorCmd[3 * i + 0].tau;
				cmd.motorCmd[3 * i + 0].tau = (cmd.motorCmd[3 * i + 0].tau < hr_min) ? hr_min : cmd.motorCmd[3 * i + 0].tau;
				cmd.motorCmd[3 * i + 1].tau = (cmd.motorCmd[3 * i + 1].tau > hp_max) ? hp_max : cmd.motorCmd[3 * i + 1].tau;
				cmd.motorCmd[3 * i + 1].tau = (cmd.motorCmd[3 * i + 1].tau < hp_min) ? hp_min : cmd.motorCmd[3 * i + 1].tau;
				cmd.motorCmd[3 * i + 2].tau = (cmd.motorCmd[3 * i + 2].tau > kn_max) ? kn_max : cmd.motorCmd[3 * i + 2].tau;
				cmd.motorCmd[3 * i + 2].tau = (cmd.motorCmd[3 * i + 2].tau < kn_min) ? kn_min : cmd.motorCmd[3 * i + 2].tau;
			}

		}else if ( (!beginCommand) && (!softFall) ){
			static bool printedA = false;
			if(!printedA){
				printf("\nPress A to continue\n");
				printedA = true;
			}
		}else if ( softFall ) {
			for (int i = 0; i < 12; ++i){
				cmd.motorCmd[i].tau = 0.0;
				cmd.motorCmd[i].q   = 0.0;
				cmd.motorCmd[i].dq  = 0.0;

				cmd.motorCmd[i].Kp = 0;
				cmd.motorCmd[i].Kd = ( ( (i+1)%3 ) == 0 ) ? 8 : 4;
			}
		}
	}

//	printf("%f\n",1000*toc(&tset));
	
	// Set all new data
	memcpy(&(LLData.ind), loco_obj->con->ind, 4*sizeof(int));
	LLData.cnt = loco_obj->con->cnt;
	LLData.q = loco_obj->state->q;
	LLData.dq = loco_obj->state->dq;
	LLData.toePos = loco_obj->kin->toePos;
	LLData.comDes = loco_obj->traj->comDes;
	LLData.QP_force = loco_obj->ll->QP_force;
	updateData(SET_DATA, LL_DATA, &LLData);

	udpComp.Send();
	udpComp.SetSend(cmd);
}


int main(int argc, char *argv[])
{
	ExternalComm extComm;
	
	InitEnvironment();
	extComm.udpComp.InitCmdData(extComm.cmd);

	extComm.loco_obj = std::unique_ptr<LocoWrapper>(new LocoWrapper(argc, argv));
	extComm.est_obj  = std::unique_ptr<KF>(new KF(extComm.dt));

	LoopFunc loop_calc("calc_loop", extComm.dt, boost::bind(&ExternalComm::Calc, &extComm));
	LoopFunc loop_MPC("MPC_loop", 0.004f, boost::bind(&ExternalComm::MPC, &extComm));
	loop_calc.start();
	loop_MPC.start();

	
	while ( (extComm.stop==0) ){
		if(extComm.btn.X > 0){
			extComm.beginPose = 1;
		}
		if(extComm.btn.A > 0){
			extComm.beginCommand = 1;
		}
		sleep(0.1);
	};

//	std::cout<<"closing  file"<<std::endl;
//	fclose(extComm.fid);
	return 0;
}




