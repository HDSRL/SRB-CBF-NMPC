/************************************************************************
Copyright (c) 2020, Unitree Robotics.Co.Ltd. All rights reserved.
Use of this source code is governed by the MPL-2.0 license, see LICENSE.
************************************************************************/

#include "utils.h"
#include "multi_pc_comm.h"
#include "unitree_legged_sdk/unitree_joystick.h"
#include "unitree_legged_sdk/remote.h"
#include "timer.h"

#include "fstream"

using namespace UNITREE_LEGGED_SDK;

class ExternalComm
{
public:
	ExternalComm() : udpComp(LOWLEVEL){
//		fid = fopen("/home/kaveh/A1_LL_Exp/wireless.csv", "w");
	}

	void Calc();

	UDP udpComp;
	FILE *fid;

	long motiontime = 0;
	float qInit[12] = {0};
	double q[18] = {0.0};
	double dq[18] = {0.0};
	double tauEst[12] = {0.0};

	float gcoeffs[6] = {-0.14286f, -0.08571f, -0.02857f, 0.02857f, 0.08571f, 0.14286f};
	float qHist[6][12] = {{0.0f}};

	LowState state = {0};
	LowCmd cmd = {0};
	double offset = 0.0;

	float dt = 0.001f;
	int stop = 0;
};

int printCommands(uint8_t remote[40]){
	buttons btn;

	uint8_t head[2];
	head[0] = remote[0];
	head[1] = remote[1];
	uint8_t btnset1 = remote[2];
	uint8_t btnset2 = remote[3];
	
	int stop = 0;
	int result;
	
	// Buttonset 1
	for (int c = 5; c >= 0; c--){
		result = btnset1 >> c;
		if(result & 1){
			if(c==0){
				btn.R1 = 1;	
			}else if(c==1){
				btn.L1 = 1;
			}else if(c==2){
				btn.start = 1;
			}else if(c==3){
				btn.select = 1;
			}else if(c==4){
				btn.R2 = 1;
			}else if(c==5){
				btn.L2 = 1;
			}
		}				  
	}
	
	// Buttonset 2
	for (int c = 7; c >= 0; c--){
		result = btnset2 >> c;
		if(result & 1){
			if(c==0){
				btn.A = 1;
			}else if(c==1){
				btn.B = 1;
			}else if(c==2){
				btn.X = 1;
			}else if(c==3){
				btn.Y = 1;
			}else if(c==4){
				btn.up = 1;
			}else if(c==5){
				btn.right = 1;
			}else if(c==6){
				btn.down = 1;
			}else if(c==7){
				btn.left = 1;
			}
			printf("1");
		}				  
	}
	
	// Joysticks
	int negMin = 186;
	int negMax = 191;
	int posMin = 58;
	int posMax = 63;
	uint8_t LX[4], LY[4], RX[4], RY[4]; 
	for(int i=0; i<24; ++i){
//		RX[i] = remote[i+8];
//		LX[i] = remote[i+4];
//		RY[i] = remote[i+12];
//		LY[i] = remote[i+20];
		//std::cout << remote[i]<< "\t";//std::endl;
		printf("%d\t",remote[i]);
		//printf("%lf",RX[i]);
//		std::cout << RX[i] << "\t";
	}	
	printf("\n");
	int32_t temp = 0;
    btn.RX = ((RX[3] << 24) |
            (RX[2] << 16) |
            (RX[1] <<  8) |
             RX[0]);
    btn.RY = ((RY[3] << 24) |
            (RY[2] << 16) |
            (RY[1] <<  8) |
             RY[0]);
    btn.LX = ((LX[3] << 24) |
            (LX[2] << 16) |
            (LX[1] <<  8) |
             LX[0]);
    btn.LY = ((LY[3] << 24) |
            (LY[2] << 16) |
            (LY[1] <<  8) |
             LY[0]);
    btn.RX -= (btn.RX>0) ? 983574538 : ( (btn.RX<0) ? -1170776472 : 0);
    btn.RY -= (btn.RY>0) ? 985556643 : ( (btn.RY<0) ? -1170402017 : 0);
    btn.LX -= (btn.LX>0) ? 983574538 : ( (btn.LX<0) ? -1170776472 : 0);
    btn.LY -= (btn.LY>0) ? 983574538 : ( (btn.LY<0) ? -1170776472 : 0);
//    printf("%d\n",btn.RY);
	
	return stop;

}

void ExternalComm::Calc()
{
//	timer tset;
//	tic(&tset);
	
	udpComp.Recv();
	udpComp.GetRecv(state);
	motiontime += 1;
	
//	for(int i=4; i<12; ++i){
//		printf("%i\t|| ",state.wirelessRemote[i]); 	
//	}	
//	printf("\n");
	
	stop = printCommands(state.wirelessRemote);
	

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

	while (extComm.stop==0){
		sleep(0.1);
	};

	fclose(extComm.fid);
	return 0;
}




