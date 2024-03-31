#ifndef OTHERUTILS
#define OTHERUTILS

#include "math.h"
#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Core"
#include "A1_Dynamics.h"
#include "Transforms.hpp"
#include "global_loco_opts.h"

inline void kinEst0(const int footForce[4], const int contactIndex[4], double q[18], double dq[18], Eigen::Matrix3d &R) {
    // ================================== //
	// ========= Con Estimator ========== //
	// ================================== //
    int actCon[4] = {0};
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
	static double COM[3]= {0.0,0.0,0};
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
		fr_prev[i] = fr_toe[i]; fl_prev[i] = fl_toe[i];
		rr_prev[i] = rr_toe[i]; rl_prev[i] = rl_toe[i];		
	}
	
	double Jfr_toe[54], Jfl_toe[54], Jrl_toe[54], Jrr_toe[54];
	double COM_vel[3] = {0,0,0};
	J_FR_toe(Jfr_toe, q); J_FL_toe(Jfl_toe, q);
	J_RR_toe(Jrr_toe, q); J_RL_toe(Jrl_toe, q);
	Eigen::Matrix<double,3,1> dq_temp = {dq[3],dq[4],dq[5]};
	toWorld(&dq[3],dq_temp,R);
	for (int i = 3; i < 18; ++i){
		COM_vel[0] -= (Jfr_toe[3*i+0]*weightedCon[0] + Jfl_toe[3*i+0]*weightedCon[1] + Jrr_toe[3*i+0]*weightedCon[2] + Jrl_toe[3*i+0]*weightedCon[3])*dq[i];
	 	COM_vel[1] -= (Jfr_toe[3*i+1]*weightedCon[0] + Jfl_toe[3*i+1]*weightedCon[1] + Jrr_toe[3*i+1]*weightedCon[2] + Jrl_toe[3*i+1]*weightedCon[3])*dq[i];
	 	COM_vel[2] -= (Jfr_toe[3*i+2]*weightedCon[0] + Jfl_toe[3*i+2]*weightedCon[1] + Jrr_toe[3*i+2]*weightedCon[2] + Jrl_toe[3*i+2]*weightedCon[3])*dq[i];
	}
	COM_vel[0] /= numContact;
	COM_vel[1] /= numContact;
	COM_vel[2] /= numContact;
	
	dq_temp = {dq[3],dq[4],dq[5]};
	toBody(&dq[3],dq_temp,R);

	// Set results
	q[0] = COM[0]; q[1] = COM[1]; q[2] = COM[2]+Z_TOE_OFFSET;
	dq[0] = COM_vel[0]; dq[1] = COM_vel[1]; dq[2] = COM_vel[2];
};

inline void kinEst1(const int footForce[4], const int contactIndex[4], double q[18], double dq[18], Eigen::Matrix3d &R) {
    // ================================== //
	// ========= Con Estimator ========== //
	// ================================== //
    int actCon[4] = {0};
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
	static double COM[3]= {0.0, -0.9,0};
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
		fr_prev[i] = fr_toe[i]; fl_prev[i] = fl_toe[i];
		rr_prev[i] = rr_toe[i]; rl_prev[i] = rl_toe[i];		
	}
	
	double Jfr_toe[54], Jfl_toe[54], Jrl_toe[54], Jrr_toe[54];
	double COM_vel[3] = {0,0,0};
	J_FR_toe(Jfr_toe, q); J_FL_toe(Jfl_toe, q);
	J_RR_toe(Jrr_toe, q); J_RL_toe(Jrl_toe, q);
	Eigen::Matrix<double,3,1> dq_temp = {dq[3],dq[4],dq[5]};
	toWorld(&dq[3],dq_temp,R);
	for (int i = 3; i < 18; ++i){
		COM_vel[0] -= (Jfr_toe[3*i+0]*weightedCon[0] + Jfl_toe[3*i+0]*weightedCon[1] + Jrr_toe[3*i+0]*weightedCon[2] + Jrl_toe[3*i+0]*weightedCon[3])*dq[i];
	 	COM_vel[1] -= (Jfr_toe[3*i+1]*weightedCon[0] + Jfl_toe[3*i+1]*weightedCon[1] + Jrr_toe[3*i+1]*weightedCon[2] + Jrl_toe[3*i+1]*weightedCon[3])*dq[i];
	 	COM_vel[2] -= (Jfr_toe[3*i+2]*weightedCon[0] + Jfl_toe[3*i+2]*weightedCon[1] + Jrr_toe[3*i+2]*weightedCon[2] + Jrl_toe[3*i+2]*weightedCon[3])*dq[i];
	}
	COM_vel[0] /= numContact;
	COM_vel[1] /= numContact;
	COM_vel[2] /= numContact;
	
	dq_temp = {dq[3],dq[4],dq[5]};
	toBody(&dq[3],dq_temp,R);

	// Set results
	q[0] = COM[0]; q[1] = COM[1]; q[2] = COM[2]+Z_TOE_OFFSET;
	dq[0] = COM_vel[0]; dq[1] = COM_vel[1]; dq[2] = COM_vel[2];
};

inline void kinEst2(const int footForce[4], const int contactIndex[4], double q[18], double dq[18], Eigen::Matrix3d &R) {
    // ================================== //
	// ========= Con Estimator ========== //
	// ================================== //
    int actCon[4] = {0};
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
	static double COM[3]= {-1,0,0};
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
		fr_prev[i] = fr_toe[i]; fl_prev[i] = fl_toe[i];
		rr_prev[i] = rr_toe[i]; rl_prev[i] = rl_toe[i];		
	}
	
	double Jfr_toe[54], Jfl_toe[54], Jrl_toe[54], Jrr_toe[54];
	double COM_vel[3] = {0,0,0};
	J_FR_toe(Jfr_toe, q); J_FL_toe(Jfl_toe, q);
	J_RR_toe(Jrr_toe, q); J_RL_toe(Jrl_toe, q);
	Eigen::Matrix<double,3,1> dq_temp = {dq[3],dq[4],dq[5]};
	toWorld(&dq[3],dq_temp,R);
	for (int i = 3; i < 18; ++i){
		COM_vel[0] -= (Jfr_toe[3*i+0]*weightedCon[0] + Jfl_toe[3*i+0]*weightedCon[1] + Jrr_toe[3*i+0]*weightedCon[2] + Jrl_toe[3*i+0]*weightedCon[3])*dq[i];
	 	COM_vel[1] -= (Jfr_toe[3*i+1]*weightedCon[0] + Jfl_toe[3*i+1]*weightedCon[1] + Jrr_toe[3*i+1]*weightedCon[2] + Jrl_toe[3*i+1]*weightedCon[3])*dq[i];
	 	COM_vel[2] -= (Jfr_toe[3*i+2]*weightedCon[0] + Jfl_toe[3*i+2]*weightedCon[1] + Jrr_toe[3*i+2]*weightedCon[2] + Jrl_toe[3*i+2]*weightedCon[3])*dq[i];
	}
	COM_vel[0] /= numContact;
	COM_vel[1] /= numContact;
	COM_vel[2] /= numContact;
	
	dq_temp = {dq[3],dq[4],dq[5]};
	toBody(&dq[3],dq_temp,R);

	// Set results
	q[0] = COM[0]; q[1] = COM[1]; q[2] = COM[2]+Z_TOE_OFFSET;
	dq[0] = COM_vel[0]; dq[1] = COM_vel[1]; dq[2] = COM_vel[2];
};

inline void kinEst3(const int footForce[4], const int contactIndex[4], double q[18], double dq[18], Eigen::Matrix3d &R) {
    // ================================== //
	// ========= Con Estimator ========== //
	// ================================== //
    int actCon[4] = {0};
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
	static double COM[3]= {-1.0, -0.9,0};
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
		fr_prev[i] = fr_toe[i]; fl_prev[i] = fl_toe[i];
		rr_prev[i] = rr_toe[i]; rl_prev[i] = rl_toe[i];		
	}
	
	double Jfr_toe[54], Jfl_toe[54], Jrl_toe[54], Jrr_toe[54];
	double COM_vel[3] = {0,0,0};
	J_FR_toe(Jfr_toe, q); J_FL_toe(Jfl_toe, q);
	J_RR_toe(Jrr_toe, q); J_RL_toe(Jrl_toe, q);
	Eigen::Matrix<double,3,1> dq_temp = {dq[3],dq[4],dq[5]};
	toWorld(&dq[3],dq_temp,R);
	for (int i = 3; i < 18; ++i){
		COM_vel[0] -= (Jfr_toe[3*i+0]*weightedCon[0] + Jfl_toe[3*i+0]*weightedCon[1] + Jrr_toe[3*i+0]*weightedCon[2] + Jrl_toe[3*i+0]*weightedCon[3])*dq[i];
	 	COM_vel[1] -= (Jfr_toe[3*i+1]*weightedCon[0] + Jfl_toe[3*i+1]*weightedCon[1] + Jrr_toe[3*i+1]*weightedCon[2] + Jrl_toe[3*i+1]*weightedCon[3])*dq[i];
	 	COM_vel[2] -= (Jfr_toe[3*i+2]*weightedCon[0] + Jfl_toe[3*i+2]*weightedCon[1] + Jrr_toe[3*i+2]*weightedCon[2] + Jrl_toe[3*i+2]*weightedCon[3])*dq[i];
	}
	COM_vel[0] /= numContact;
	COM_vel[1] /= numContact;
	COM_vel[2] /= numContact;
	
	dq_temp = {dq[3],dq[4],dq[5]};
	toBody(&dq[3],dq_temp,R);

	// Set results
	q[0] = COM[0]; q[1] = COM[1]; q[2] = COM[2]+Z_TOE_OFFSET;
	dq[0] = COM_vel[0]; dq[1] = COM_vel[1]; dq[2] = COM_vel[2];
};

#endif