//
// Authror: Randy Fawcett on 03/2021.
//
// Copyright (c) Hybrid Dynamic Systems and Robot Locomotion Lab, Virginia Tech
//

#ifndef PARAMETERS_H
#define PARAMETERS_H

#include "fstream"
#include <stdio.h>
#include <string.h>
#include "iostream"
#include <errno.h>
#include "global_loco_structs.hpp"

using MPC    = Settings::MPC_params;
using LL     = Settings::LL_params;
using Motion = Settings::Motion_params;

class Parameters {
    public: 
        Parameters(int argc, char *argv[]);
        ~Parameters();

        void loadLowLevelParams(char* filename);
        void loadMPCParams(char* filename);
        void loadWalkingParams(char* filename);

        MPC mpc_params;
        LL ll_params;
        Motion motion_params;
};

#endif