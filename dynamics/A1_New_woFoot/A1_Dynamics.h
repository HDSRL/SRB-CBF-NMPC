#ifndef __A1_DYNAMICS_H__
#define __A1_DYNAMICS_H__

#define MAX(X,Y)  ((X) < (Y) ? (Y) : (X)) 
#define MIN(X,Y)  ((X) > (Y) ? (Y) : (X)) 

#include "math.h"
#include "dynamicsSupportFunctions.h"

void D_mat(double *p_output1,const double *var1);

void C_vec(double *p_output1,const double *var1,const double *var2);

void B_mat(double *p_output1,const double *var1);

void G_vec(double *p_output1,const double *var1);

void FK_FL_toe(double *p_output1,const double *var1);

void FK_FR_toe(double *p_output1,const double *var1);

void FK_RL_toe(double *p_output1,const double *var1);

void FK_RR_toe(double *p_output1,const double *var1);

void J_FL_toe(double *p_output1,const double *var1);

void J_FR_toe(double *p_output1,const double *var1);

void J_RL_toe(double *p_output1,const double *var1);

void J_RR_toe(double *p_output1,const double *var1);

void dJ_FL_toe(double *p_output1,const double *var1,const double *var2);

void dJ_FR_toe(double *p_output1,const double *var1,const double *var2);

void dJ_RL_toe(double *p_output1,const double *var1,const double *var2);

void dJ_RR_toe(double *p_output1,const double *var1,const double *var2);

void FK_FL_hip(double *p_output1,const double *var1);

void FK_FR_hip(double *p_output1,const double *var1);

void FK_RL_hip(double *p_output1,const double *var1);

void FK_RR_hip(double *p_output1,const double *var1);

void J_FL_hip(double *p_output1,const double *var1);

void J_FR_hip(double *p_output1,const double *var1);

void J_RL_hip(double *p_output1,const double *var1);

void J_RR_hip(double *p_output1,const double *var1);

void dJ_FL_hip(double *p_output1,const double *var1,const double *var2);

void dJ_FR_hip(double *p_output1,const double *var1,const double *var2);

void dJ_RL_hip(double *p_output1,const double *var1,const double *var2);

void dJ_RR_hip(double *p_output1,const double *var1,const double *var2);

#endif