/************************************************************************
Copyright (c) 2020, Unitree Robotics.Co.Ltd. All rights reserved.
Use of this source code is governed by the MPL-2.0 license, see LICENSE.
************************************************************************/

#include "fast_MPC.hpp"
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
    ExternalComm(): udpComp(8018, "192.168.123.161", 8090, sizeof(RobotRecv), sizeof(RobotSend)){
    	 fid = fopen("/home/randy/HDSRL/sinTest.csv","w");
    }

    void UDPRecv();
    void UDPSend();
    void Calc();

    UDP udpComp;
    FastMPC *loco_obj;
    FILE *fid;
    
    long motiontime = 0;
    double q[18] = {0.0};
    double dq[18] = {0.0};
    bool startTap = false;

    Eigen::Matrix<double, 18, 1> qe;
    Eigen::Matrix<double, 18, 1> dqe;
    
    LowState state = {0};
    LowCmd     cmd = {0};

    double Kp;
    double Kd;
    double height;

    bool setup = true;
    
    RobotRecv csend;
    RobotSend crecv;
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

void ExternalComm::Calc()
{
    udpComp.GetRecv((char*)&crecv);

    motiontime++;

//     crecv.begin = 1;
//     crecv.q[0] = 0;
//     crecv.q[1] = 0;
//     crecv.q[2] = 0.27;
//     crecv.q[3] = 0;
//     crecv.q[4] = 0;
//     crecv.q[5] = 0;
//     for(int i=0; i<4; ++i){
//         crecv.q[3*i+6] = 0;
//         crecv.q[3*i+7] = M_PI/4;
//         crecv.q[3*i+8] = -M_PI/2;
//     }
//     crecv.Rot[0] = 1;
//     crecv.Rot[4] = 1;
//     crecv.Rot[8] = 1;
//     crecv.ctrltick = motiontime;

    Eigen::Matrix<double, 18, 1> tau, dq, q;
    Eigen::Matrix<double,  6, 1> y_out;
    if( (crecv.begin == 1) && (setup) ){
        Eigen::Matrix<double, 3, 1> com;
        com(0) = crecv.q[0];
        com(1) = crecv.q[1];
        com(2) = crecv.q[2];
        loco_obj->timeSetup(crecv.ctrltick,crecv.ctrltick);
        loco_obj->posSetup(com);
        loco_obj->setPoseType(POSE_Z);
        loco_obj->compute(crecv.q,crecv.dq,crecv.Rot,STAND,crecv.ctrltick,Kp,Kd,height);
        setup = false;
    }
    if( (crecv.begin == 1) ){
    	if(startTap==false){
    		loco_obj->compute(crecv.q,crecv.dq,crecv.Rot,STAND,crecv.ctrltick,Kp,Kd,height);
    	} else {
        	loco_obj->compute(crecv.q,crecv.dq,crecv.Rot,POSE,crecv.ctrltick,Kp,Kd,height);
        }
        
        tau   = loco_obj->getJointTorqueCommand();
        dq    = loco_obj->getJointVelocityCommand();
        q     = loco_obj->getJointPositionCommand();
        y_out = loco_obj->getHZDOutputs();
        for(int i=0; i<12; ++i){
            csend.tau[i] = tau(i+6); 
            csend.pos[i] = q(i+6);
            csend.vel[i] = dq(i+6);
        } 
        if(crecv.streaming == 0){
            std::cout << tau.block(6,0,12,1).transpose() << std::endl;
//            std::cout << q.block(6,0,12,1).transpose() << std::endl;
//            std::cout << dq.block(6,0,12,1).transpose() << std::endl << std::endl;
        } else {
        	 fprintf(fid, "%li,%lf,%lf,%lf,%lf,%lf,%lf,",crecv.ctrltick,y_out(0),y_out(1),y_out(2),y_out(3),y_out(4),y_out(5));
        	 for(int i=0; i<12; ++i){
        	 	fprintf(fid, "%lf,", tau(i+6));
        	 }
        	 fprintf(fid, "\n");
        }
    }
    
    udpComp.SetSend((char*)&csend);
}
 
int main(int argc, char* argv[]) 
{
    ExternalComm extComm;
    extComm.loco_obj = new FastMPC(argc,argv);

    LoopFunc loop_calc("calc_loop",   extComm.dt,    boost::bind(&ExternalComm::Calc,    &extComm));
    LoopFunc loop_udpCompSend("udp_send", 0.001, 3, boost::bind(&ExternalComm::UDPSend, &extComm));
    LoopFunc loop_udpCompRecv("udp_recv", 0.001, 3, boost::bind(&ExternalComm::UDPRecv, &extComm));

    loop_udpCompSend.start();
    loop_udpCompRecv.start();
    loop_calc.start();

	double wn = 10.0;
	double eps = 0.5;
	double kdtune = 2.0;
    extComm.height = 0.275;
    
    extComm.Kp = 400;
	extComm.Kd = 5;

    struct input_event ev;
    int k = 1;
    int fd = open("/dev/input/event4", O_RDONLY);
    printf("fd = %d\n", fd);
    while(1){
        size_t mmm = read(fd, &ev, sizeof(ev));
        
        // Update the gains
        if( (ev.type == EV_KEY) && (ev.value == 0 || ev.value == 2) ){
            if (ev.code == 103) { 
                extComm.Kp = 400;
                extComm.Kd += 0.1;
                printf("Wn: %lf || Kp: %lf || Kd: %lf\n", wn, extComm.Kp, extComm.Kd);
            } else if(ev.code == 108) {
            	extComm.Kp = 400;
                extComm.Kd -= 0.1;
                printf("Wn: %lf || Kp: %lf || Kd: %lf\n", wn, extComm.Kp, extComm.Kd);
            }
        }
        
        // Update the position of the COM
        if( (ev.type == EV_KEY) && (ev.value == 0 || ev.value == 2) ){
            if (ev.code == 17) { 
                extComm.height += 0.001;
                printf("Height: %lf\n", extComm.height);
            } else if(ev.code == 31) {
                extComm.height -= 0.001;
                printf("Height: %lf\n", extComm.height);
            }
        }
        
        // Update startTap
        if( (ev.type == EV_KEY) && (ev.value == 0) ){
            if (ev.code == 57) { 
                extComm.startTap = true;
            }
        }
        
        // Update startTap
        if( (ev.type == EV_KEY) && (ev.value == 0) ){
            if (ev.code == 1) { 
               break;
            }
        }
        
        sleep(0.1);
    };


	// fclose(extComm.fid);
    return 0; 
}
