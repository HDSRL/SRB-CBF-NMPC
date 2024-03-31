//
// Authror: Randy Fawcett on 09/2021.
//
// Copyright (c) Hybrid Dynamic Systems and Robot Locomotion Lab, Virginia Tech
//

#include "Parameters.hpp"

Parameters::Parameters(int argc, char *argv[]){
    int loadMPC = 0; 
    int loadLL = 0;
    int loadWalking = 0;
    for(int i=1; i<argc; i++){
        std::string path = argv[i];
        std::string filenameOnly = path.substr(path.find_last_of("/\\") + 1);
        if(filenameOnly.find("LL")!=std::string::npos){
            loadLL = i;
        }
        if(filenameOnly.find("MPC")!=std::string::npos){
            loadMPC = i;
        }
        if(filenameOnly.find("Walking")!=std::string::npos){
            loadWalking = i;
        }
    }

    if(loadMPC>0){
        std::cout << "Loading: "<< argv[loadMPC] << "..." <<std::endl;
        loadMPCParams(argv[loadMPC]);
    } else {
        // Default MPC Parameters
        mpc_params.mu_MPC = 0.7;

        mpc_params.qpx = 1e3;
        mpc_params.qpy = 1e3;
        mpc_params.qpz = 1e3;
        mpc_params.qvx = 1e3;
        mpc_params.qvy = 1e3;
        mpc_params.qvz = 1e3;
        mpc_params.qrr = 1e3;
        mpc_params.qrp = 1e3;
        mpc_params.qry = 1e3;
        mpc_params.qwr = 1e3;
        mpc_params.qwp = 1e3;
        mpc_params.qwy = 1e3;
        mpc_params.pp = 40;
        mpc_params.pv = 40;
        mpc_params.pr = 40;
        mpc_params.pw = 40;
        mpc_params.rx = 1e-2;
        mpc_params.ry = 1e-2;
        mpc_params.rz = 1e-2;
        mpc_params.updateEveryItter = 0;
        mpc_params.useMPCTraj = 0;
    }

    if(loadLL>0){
        std::cout<<"Loading: "<< argv[loadLL] << "..." <<std::endl;
        loadLowLevelParams(argv[loadLL]);
    } else {
        // Default Low Level
        ll_params.mu = 0.7;
        ll_params.kp = 700;   // proportional gain on IO linearization
        ll_params.kd = 40;    // derivative gain on IO linearization
        ll_params.useCLF = true;

        // Cost
        ll_params.tauPen = 1e0; // input weight (cost)
        ll_params.dfPen  = 1e-1; // ||F-F_d|| weight (cost)
        ll_params.auxPen = 1e6; // auxiliary var weight (cost)
        ll_params.clfPen = 1e8; // clf defect var weight (cost)
        
        // Constraints
        ll_params.auxMax = 100;  // max aux value (constraint)
        ll_params.clfEps = 0.8; // CLF convergence tuning variable
    }

    if(loadWalking>0){
        std::cout<<"Loading: "<< argv[loadWalking] << "..." <<std::endl;
        loadWalkingParams(argv[loadWalking]);
    } else {
        // Default 
        motion_params.standHeight = 0.26;
        motion_params.swingHeight = 0.06;
        motion_params.fwdSpeed = 0.0;
        motion_params.latSpeed = 0.0;
        motion_params.yawSpeed = 0.0;
        motion_params.neverStopTrot = 1;
    }
    
};

Parameters::~Parameters(){};

void Parameters::loadLowLevelParams(char* filename){
    FILE *fid;
    errno = 0;
    fid = fopen(filename,"r");
    if(fid==NULL){
        std::cout<<strerror(errno)<<std::endl;
    }
    
    int j;
    j = fscanf(fid, "%lf\n", &ll_params.mu);
    j = fscanf(fid, "%lf\n", &ll_params.kp);
    j = fscanf(fid, "%lf\n", &ll_params.kd);
    j = fscanf(fid, "%d\n",  &ll_params.useCLF);
    j = fscanf(fid, "%lf\n", &ll_params.tauPen);
    j = fscanf(fid, "%lf\n", &ll_params.dfPen);
    j = fscanf(fid, "%lf\n", &ll_params.auxPen);
    j = fscanf(fid, "%lf\n", &ll_params.clfPen);
    j = fscanf(fid, "%lf\n", &ll_params.auxMax);
    j = fscanf(fid, "%lf\n", &ll_params.clfEps);
    j = fclose(fid);
};

void Parameters::loadMPCParams(char* filename){
    FILE *fid;
    errno = 0;
    fid = fopen(filename,"r");
    if(fid==NULL){
        std::cout<<strerror(errno)<<std::endl;
    }
    int j;
    j = fscanf(fid, "%lf", &mpc_params.mu_MPC);
    j = fscanf(fid, "%lf", &mpc_params.qpx);
    j = fscanf(fid, "%lf", &mpc_params.qpy);
    j = fscanf(fid, "%lf", &mpc_params.qpz);
    j = fscanf(fid, "%lf", &mpc_params.qvx);
    j = fscanf(fid, "%lf", &mpc_params.qvy);
    j = fscanf(fid, "%lf", &mpc_params.qvz);
    j = fscanf(fid, "%lf", &mpc_params.qrr);
    j = fscanf(fid, "%lf", &mpc_params.qrp);
    j = fscanf(fid, "%lf", &mpc_params.qry);
    j = fscanf(fid, "%lf", &mpc_params.qwr);
    j = fscanf(fid, "%lf", &mpc_params.qwp);
    j = fscanf(fid, "%lf", &mpc_params.qwy);
    j = fscanf(fid, "%lf", &mpc_params.pp);
    j = fscanf(fid, "%lf", &mpc_params.pv);
    j = fscanf(fid, "%lf", &mpc_params.pr);
    j = fscanf(fid, "%lf", &mpc_params.pw);
    j = fscanf(fid, "%lf", &mpc_params.rx);
    j = fscanf(fid, "%lf", &mpc_params.ry);
    j = fscanf(fid, "%lf", &mpc_params.rz);
    j = fscanf(fid, "%lf", &mpc_params.updateEveryItter);
    j = fscanf(fid, "%lf", &mpc_params.useMPCTraj);
    j = fclose(fid);
};

void Parameters::loadWalkingParams(char* filename){
    FILE *fid;
    errno = 0;
    fid = fopen(filename,"r");
    if(fid==NULL){
        std::cout<<strerror(errno)<<std::endl;
    }
    
    int j;
    j = fscanf(fid, "%lf\n", &motion_params.standHeight);
    j = fscanf(fid, "%lf\n", &motion_params.swingHeight);
    j = fscanf(fid, "%lf\n", &motion_params.fwdSpeed);
    j = fscanf(fid, "%lf\n", &motion_params.latSpeed);
    j = fscanf(fid, "%lf\n", &motion_params.yawSpeed);
    j = fscanf(fid, "%lf\n", &motion_params.neverStopTrot);
    j = fclose(fid);
}