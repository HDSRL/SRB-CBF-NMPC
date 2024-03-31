#ifndef _MULTI_PC_COMM_H_
#define _MULTI_PC_COMM_H_

#include <stdint.h>
#include "unitree_legged_sdk/unitree_legged_sdk.h"

struct RobotSend{
    long ctrltick;
    int begin = 0;
    int streaming = 0;
    double q[18] = {0.0};
    double dq[18] = {0.0};
    double Rot[9];
    double tauEst = 0.0;
    uint32_t crc;
};

struct RobotRecv{
	double pos[12] = {0.0};
	double vel[12] = {0.0};
    double tau[12] = {0.0};
    uint32_t crc;
};

#endif
