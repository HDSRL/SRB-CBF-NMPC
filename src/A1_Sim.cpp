//
// Authror: Randy Fawcett on 03/2021.
//
// Copyright (c) Hybrid Dynamic Systems and Robot Locomotion Lab, Virginia Tech
//

#include "raisim/OgreVis.hpp"
#include "randyImguiPanel.hpp"
#include "raisimKeyboardCallback.hpp"
#include "helper.hpp"
#include "helper2.hpp"
#include "MPC_dist.hpp"
#include "OtherUtils.hpp"
#include "timer.h"
#include <cstdlib> // For rand() and srand()
#include <ctime>   // For time()
#include <fstream>
#include <iostream>
//HDSRL header
#include "LocoWrapper.hpp"
#include "shared_structs.hpp"

bool rough_terrain_en = 0;
bool obstacle_en = 0;

using std::cout;
using std::cin;

bool first_time0 = 1;
bool first_time1 = 1;
bool first_time2 = 1;
bool first_time3 = 1; 

bool sec_time = 1;
sharedData HLData0;
sharedData LLData0;

sharedData HLData1;
sharedData LLData1;

sharedData HLData2;
sharedData LLData2;

sharedData HLData3;
sharedData LLData3;

sharedData HLData0_Backup, LLData0_Backup;
sharedData HLData1_Backup, LLData1_Backup;
sharedData HLData2_Backup, LLData2_Backup;
sharedData HLData3_Backup, LLData3_Backup;




void setupCallback() {
    raisim::OgreVis *vis = raisim::OgreVis::get();

    /// light
    vis->getLight()->setDiffuseColour(1, 1, 1);
    vis->getLight()->setCastShadows(false);
    Ogre::Vector3 lightdir(-3,3,-0.5); // Light shines on ROBOTS top/front/right side
    // Ogre::Vector3 lightdir(-3,-3,-0.5); // Light shines on ROBOTS top/front/left side
    lightdir.normalise();
    vis->getLightNode()->setDirection({lightdir});
    vis->setCameraSpeed(300);

    vis->addResourceDirectory(raisim::loadResource("material"));
    vis->loadMaterialFile("myMaterials.material");

    vis->addResourceDirectory(vis->getResourceDir() + "/material/skybox/violentdays");
    vis->loadMaterialFile("violentdays.material");

    /// shdow setting
    vis->getSceneManager()->setShadowTechnique(Ogre::SHADOWTYPE_TEXTURE_ADDITIVE);
    vis->getSceneManager()->setShadowTextureSettings(2048, 3);

    /// scale related settings!! Please adapt it depending on your map size
    // beyond this distance, shadow disappears
    vis->getSceneManager()->setShadowFarDistance(10);
    // size of contact points and contact forces
    vis->setContactVisObjectSize(0.03, 0.6);
    // speed of camera motion in freelook mode
    vis->getCameraMan()->setTopSpeed(5);
}

void disturbance(std::vector<raisim::ArticulatedSystem *> A1,std::map<std::string, raisim::VisualObject> &objList,size_t start,size_t stop,size_t ctrlTick){
    Eigen::VectorXd pos = Eigen::MatrixXd::Zero(TOTAL_DOF+1,1);
    Eigen::VectorXd vel = Eigen::MatrixXd::Zero(TOTAL_DOF,1);
    A1.back()->getState(pos, vel);
    
    // ====================== External Forces ====================== //
    raisim::Vec<3> extForce;
    raisim::Mat<3,3> rot;
    raisim::Vec<3> dir;
    if (ctrlTick>=start && ctrlTick<stop){
        extForce = {0,-20,0}; // Pulse
        // extForce = {50*sin(4*ctrlTick*simfreq_raisim),0,0}; // Fwd Sine
        //extForce = {0,20*sin(4*ctrlTick*simfreq_raisim),0}; // Lat Sine
    } else{
        extForce = {0,0,0};
    }
    A1.back()->setExternalForce(1,extForce);
    dir = extForce;
    dir /= dir.norm();
    raisim::zaxisToRotMat(dir, rot);
    objList["extForceArrow"].offset = {pos(0),pos(1),pos(2)};
    objList["extForceArrow"].scale = {0.2,0.2,0.01*extForce.norm()};
    objList["extForceArrow"].rotationOffset = rot;
}

void controller0(std::vector<raisim::ArticulatedSystem *> A1, LocoWrapper *loco_obj, MPC_dist* mpc_obj,size_t controlTick, bool* runMPC) {
    /////////////////////////////////////////////////////////////////////
    //////////////////////////// INITIALIZE
    /////////////////////////////////////////////////////////////////////
    size_t loco_kind = TROT;                        // Gait pattern to use
    size_t pose_kind = POSE_COMB;                   // Pose type to use (if loco_kind is set to POSE)
    size_t settling = 0.2*ctrlHz;                   // Settling down
    size_t duration = 0.8*ctrlHz;                   // Stand up
    size_t loco_start = settling + duration;        // Start the locomotion pattern

    double *tau;
    Eigen::VectorXd jointTorqueFF = Eigen::MatrixXd::Zero(TOTAL_DOF,1);
    Eigen::VectorXd jointPosTotal = Eigen::MatrixXd::Zero(TOTAL_DOF+1,1); // +1 is for 4th Component of Quaternion 
    Eigen::VectorXd jointVelTotal = Eigen::MatrixXd::Zero(TOTAL_DOF,1);
    static int conIndDes[4] = {1};
    
    /////////////////////////////////////////////////////////////////////
    //////////////////////////// UPDATE STATE
    /////////////////////////////////////////////////////////////////////

    A1.back()->getState(jointPosTotal, jointVelTotal);
    raisim::Mat<3,3> rotMat;
    Eigen::Matrix3d rotE;
    A1.back()->getBaseOrientation(rotMat);
    double rotMatrixDouble[9];
    for(size_t i=0;i<9;i++){
        rotMatrixDouble[i] = rotMat[i];
        rotE(i) = rotMat[i];
    }
    jointVelTotal.segment(3,3) = rotE.transpose()*jointVelTotal.segment(3,3); // convert to body frame, like robot measurements

    double jpos[18], jvel[18];
    Eigen::Matrix<double, 3, 1> eul;
    Eigen::Matrix<double, 4, 1> quat;
    quat = jointPosTotal.block(3,0,4,1);
    quat_to_XYZ(quat,eul);
    for(size_t i=0; i<3; ++i){
        jpos[i] = jointPosTotal(i);
        jvel[i] = jointVelTotal(i);
        jpos[i+3] = eul(i);
        jvel[i+3] = jointVelTotal(i+3);
    }
    for(size_t i=6; i<18; ++i){
        jpos[i] = jointPosTotal(i+1);
        jvel[i] = jointVelTotal(i);
    }

    int force[4] = {0};
    for(auto &con: A1.back()->getContacts()){
        int conInd = con.getlocalBodyIndex();
        force[conInd/3-1] = 0;
        // force[conInd/3-1] = con.getNormal().e().norm();
    }

    // kinEst0(force,conIndDes,jpos,jvel,rotE);

    /////////////////////////////////////////////////////////////////////
    //////////////////////////// CONTROL
    /////////////////////////////////////////////////////////////////////
    // Update the desired torques
    if(controlTick < settling){ // Settle down
        // loco_obj->posSetup(jointPosTotal.head(3));
        double temp[18] = {0};
        tau = temp;
        loco_obj->initStandVars(jointPosTotal.block(0,0,3,1),(int)duration);
    }
    else if(controlTick >= settling & controlTick <= loco_start){ // Start standing
    
        //================================================================================================
        //========================================== HIGH LEVEL ==========================================
        //================================================================================================
        updateData0(GET_DATA, HL_DATA, &HLData0);
        mpc_obj->updateState(HLData0.q, HLData0.dq, HLData0.ind, HLData0.toePos, HLData0.last_state1);
        // mpc_obj->plannedCycleIndex(TROT);

        if ((*runMPC == 1 || first_time0) && LLData0.domain < TOTALSTEPNUM)
        {
            mpc_obj->lipMPC_eventbase();
            *runMPC = 0;
            first_time0 = 0;

            HLData0.MPC_data_available = 1;
            HLData0.alpha_COM = mpc_obj->get_alphaCOM();
            HLData0.MPC_sol_ = mpc_obj->get_MPCsol();
            HLData0.domain = mpc_obj->getDomain();
            HLData0.last_state0 = mpc_obj->get_lastState();
            // std::cout << "TEST LAST_STATE 0: " << HLData0.last_state0.transpose(); std::cin.get();
        }
        updateData0(SET_DATA, HL_DATA, &HLData0);

        //================================================================================================
        //========================================== LOW LEVEL ===========================================
        //================================================================================================
        updateData0(GET_DATA, LL_DATA, &LLData0);
        loco_obj->set_MPC_DATA(LLData0.alpha_COM, LLData0.MPC_sol_, LLData0.MPC_data_available);
        // loco_obj->plannedCycleIndex(TROT);
        loco_obj->calcTau(jpos,jvel,rotMatrixDouble,force,STAND,controlTick, runMPC);
        tau = loco_obj->getTorque();

        LLData0.MPC_data_available = loco_obj->get_MPC_data_available();
        const int* contactMat = loco_obj->getConDes();
        Eigen::Matrix<double, 3, 4> toePosTmp = loco_obj->get_toePos();
        LLData0.toePos = toePosTmp;
        memcpy(LLData0.ind, contactMat,4*sizeof(int));
        updateData0(SET_DATA, LL_DATA, &LLData0);
    }
    else if(controlTick > loco_start){ // Start locomotion

        //================================================================================================
        //========================================== HIGH LEVEL ==========================================
        //================================================================================================
        size_t loco_kind = TROT;
        // loco_kind = (LLData.domain >= 40) ? STAND : TROT;
        // std::cout<<LLData.domain<<std::endl;

        // timer tset;
        // tic(&tset);

        updateData0(GET_DATA, HL_DATA, &HLData0);
        mpc_obj->updateState(HLData0.q, HLData0.dq, HLData0.ind, HLData0.toePos, HLData0.last_state1);

        if ((*runMPC == 1 || sec_time) && LLData0.domain < TOTALSTEPNUM)
        {
            sec_time = 0;
            // mpc_obj->plannedCycleIndex(loco_kind);
            mpc_obj->lipMPC_eventbase();
            *runMPC = 0;

            HLData0.MPC_data_available = 1;
            HLData0.alpha_COM = mpc_obj->get_alphaCOM();
            HLData0.MPC_sol_ = mpc_obj->get_MPCsol();
            HLData0.domain = mpc_obj->getDomain();
            HLData0.last_state0 = mpc_obj->get_lastState();
            // std::cout << "HLData0.last_state0: " << HLData0.last_state0.transpose(); std::cin.get();
        }
        updateData0(SET_DATA, HL_DATA, &HLData0);

        // printf("%f\n",1000*toc(&tset));

        // cout << "MPC_sol: \n" << HLData.MPC_sol_;

        //================================================================================================
        //========================================== LOW LEVEL ==========================================
        //================================================================================================
        
        updateData0(GET_DATA, LL_DATA, &LLData0);

        loco_obj->set_MPC_DATA(LLData0.alpha_COM, LLData0.MPC_sol_, LLData0.MPC_data_available);
        loco_obj->calcTau(jpos,jvel,rotMatrixDouble,force,loco_kind,controlTick, runMPC);
        tau = loco_obj->getTorque();

        LLData0.MPC_data_available = loco_obj->get_MPC_data_available();
        const int* contactMat = loco_obj->getConDes();
        Eigen::Matrix<double, 3, 4> toePosTmp = loco_obj->get_toePos();
        LLData0.toePos = toePosTmp;
        memcpy(LLData0.ind, contactMat,4*sizeof(int));
        updateData0(SET_DATA, LL_DATA, &LLData0);
    }

    memcpy(data0.q,jpos,18*sizeof(double));
    memcpy(data0.dq,jvel,18*sizeof(double));

    const int* contactMat = loco_obj->getConDes();
    memcpy(data0.ind,contactMat,4*sizeof(int));
    memcpy(conIndDes,contactMat,4*sizeof(int));

    jointTorqueFF = Eigen::Map< Eigen::Matrix<double,18,1> >(tau,18);
    jointTorqueFF.block(0,0,6,1).setZero();

    // Set the desired torques
    A1.back()->setControlMode(raisim::ControlMode::FORCE_AND_TORQUE);

    A1.back()->setGeneralizedForce(jointTorqueFF);
};

void controller1(std::vector<raisim::ArticulatedSystem *> A1, LocoWrapper *loco_obj, MPC_dist* mpc_obj,size_t controlTick, bool* runMPC) {
    /////////////////////////////////////////////////////////////////////
    //////////////////////////// INITIALIZE
    /////////////////////////////////////////////////////////////////////
    size_t loco_kind = TROT;                        // Gait pattern to use
    size_t pose_kind = POSE_COMB;                   // Pose type to use (if loco_kind is set to POSE)
    size_t settling = 0.2*ctrlHz;                   // Settling down
    size_t duration = 0.8*ctrlHz;                   // Stand up
    size_t loco_start = settling + duration;        // Start the locomotion pattern

    double *tau;
    Eigen::VectorXd jointTorqueFF = Eigen::MatrixXd::Zero(TOTAL_DOF,1);
    Eigen::VectorXd jointPosTotal = Eigen::MatrixXd::Zero(TOTAL_DOF+1,1); // +1 is for 4th Component of Quaternion 
    Eigen::VectorXd jointVelTotal = Eigen::MatrixXd::Zero(TOTAL_DOF,1);
    static int conIndDes[4] = {1};
    
    /////////////////////////////////////////////////////////////////////
    //////////////////////////// UPDATE STATE
    /////////////////////////////////////////////////////////////////////

    A1.back()->getState(jointPosTotal, jointVelTotal);
    raisim::Mat<3,3> rotMat;
    Eigen::Matrix3d rotE;
    A1.back()->getBaseOrientation(rotMat);
    double rotMatrixDouble[9];
    for(size_t i=0;i<9;i++){
        rotMatrixDouble[i] = rotMat[i];
        rotE(i) = rotMat[i];
    }
    jointVelTotal.segment(3,3) = rotE.transpose()*jointVelTotal.segment(3,3); // convert to body frame, like robot measurements

    double jpos[18], jvel[18];
    Eigen::Matrix<double, 3, 1> eul;
    Eigen::Matrix<double, 4, 1> quat;
    quat = jointPosTotal.block(3,0,4,1);
    quat_to_XYZ(quat,eul);
    for(size_t i=0; i<3; ++i){
        jpos[i] = jointPosTotal(i);
        jvel[i] = jointVelTotal(i);
        jpos[i+3] = eul(i);
        jvel[i+3] = jointVelTotal(i+3);
    }
    for(size_t i=6; i<18; ++i){
        jpos[i] = jointPosTotal(i+1);
        jvel[i] = jointVelTotal(i);
    }

    int force[4] = {0};
    for(auto &con: A1.back()->getContacts()){
        int conInd = con.getlocalBodyIndex();
        force[conInd/3-1] = 0;
        // force[conInd/3-1] = con.getNormal().e().norm();
    }

    // kinEst1(force,conIndDes,jpos,jvel,rotE);

    /////////////////////////////////////////////////////////////////////
    //////////////////////////// CONTROL
    /////////////////////////////////////////////////////////////////////
    // Update the desired torques
    if(controlTick < settling){ // Settle down
        // loco_obj->posSetup(jointPosTotal.head(3));
        double temp[18] = {0};
        tau = temp;
        loco_obj->initStandVars(jointPosTotal.block(0,0,3,1),(int)duration);
    }
    else if(controlTick >= settling & controlTick <= loco_start){ // Start standing
    
        //================================================================================================
        //========================================== HIGH LEVEL ==========================================
        //================================================================================================
        updateData1(GET_DATA, HL_DATA, &HLData1);
        mpc_obj->updateState(HLData1.q, HLData1.dq, HLData1.ind, HLData1.toePos, HLData1.last_state0);
        // mpc_obj->plannedCycleIndex(TROT);

        if ((*runMPC == 1 || first_time1) && LLData1.domain < TOTALSTEPNUM)
        {   
            mpc_obj->lipMPC_eventbase();
            *runMPC = 0;
            first_time1 = 0;

            HLData1.MPC_data_available = 1;
            HLData1.alpha_COM = mpc_obj->get_alphaCOM();
            HLData1.MPC_sol_ = mpc_obj->get_MPCsol();
            HLData1.domain = mpc_obj->getDomain();
            HLData1.last_state1 = mpc_obj->get_lastState();
        }
        updateData1(SET_DATA, HL_DATA, &HLData1);

        //================================================================================================
        //========================================== LOW LEVEL ===========================================
        //================================================================================================
        updateData1(GET_DATA, LL_DATA, &LLData1);
        loco_obj->set_MPC_DATA(LLData1.alpha_COM, LLData1.MPC_sol_, LLData1.MPC_data_available);
        // loco_obj->plannedCycleIndex(TROT);
        loco_obj->calcTau(jpos,jvel,rotMatrixDouble,force,STAND,controlTick, runMPC);
        tau = loco_obj->getTorque();

        LLData1.MPC_data_available = loco_obj->get_MPC_data_available();
        const int* contactMat = loco_obj->getConDes();
        Eigen::Matrix<double, 3, 4> toePosTmp = loco_obj->get_toePos();
        LLData1.toePos = toePosTmp;
        memcpy(LLData1.ind, contactMat,4*sizeof(int));
        updateData1(SET_DATA, LL_DATA, &LLData1);
    }
    else if(controlTick > loco_start){ // Start locomotion

        //================================================================================================
        //========================================== HIGH LEVEL ==========================================
        //================================================================================================
        size_t loco_kind = TROT;
        // loco_kind = (LLData.domain >= 40) ? STAND : TROT;
        // std::cout<<LLData.domain<<std::endl;

        // timer tset;
        // tic(&tset);

        updateData1(GET_DATA, HL_DATA, &HLData1);
        mpc_obj->updateState(HLData1.q, HLData1.dq, HLData1.ind, HLData1.toePos, HLData1.last_state0);

        if ((*runMPC == 1 || sec_time) && LLData1.domain < TOTALSTEPNUM)
        {
            sec_time = 0;
            mpc_obj->plannedCycleIndex(loco_kind);
            mpc_obj->lipMPC_eventbase();
            *runMPC = 0;

            HLData1.MPC_data_available = 1;
            HLData1.alpha_COM = mpc_obj->get_alphaCOM();
            HLData1.MPC_sol_ = mpc_obj->get_MPCsol();
            HLData1.domain = mpc_obj->getDomain();
            HLData1.last_state1 = mpc_obj->get_lastState();
        }
        updateData1(SET_DATA, HL_DATA, &HLData1);
        // std::cout << "5   DEBUG POINT CONTROLLER\n";

        // printf("%f\n",1000*toc(&tset));

        // cout << "MPC_sol: \n" << HLData.MPC_sol_;

        //================================================================================================
        //========================================== LOW LEVEL ==========================================
        //================================================================================================
        
        updateData1(GET_DATA, LL_DATA, &LLData1);

        loco_obj->set_MPC_DATA(LLData1.alpha_COM, LLData1.MPC_sol_, LLData1.MPC_data_available);
        loco_obj->calcTau(jpos,jvel,rotMatrixDouble,force,loco_kind,controlTick, runMPC);
        tau = loco_obj->getTorque();

        LLData1.MPC_data_available = loco_obj->get_MPC_data_available();
        const int* contactMat = loco_obj->getConDes();
        Eigen::Matrix<double, 3, 4> toePosTmp = loco_obj->get_toePos();
        LLData1.toePos = toePosTmp;
        memcpy(LLData1.ind, contactMat,4*sizeof(int));
        updateData1(SET_DATA, LL_DATA, &LLData1);
        // std::cout << "6   DEBUG POINT CONTROLLER\n";
    }

    memcpy(data1.q,jpos,18*sizeof(double));
    memcpy(data1.dq,jvel,18*sizeof(double));

    const int* contactMat = loco_obj->getConDes();
    memcpy(data1.ind,contactMat,4*sizeof(int));
    memcpy(conIndDes,contactMat,4*sizeof(int));

    jointTorqueFF = Eigen::Map< Eigen::Matrix<double,18,1> >(tau,18);
    jointTorqueFF.block(0,0,6,1).setZero();

    // Set the desired torques
    A1.back()->setControlMode(raisim::ControlMode::FORCE_AND_TORQUE);

    A1.back()->setGeneralizedForce(jointTorqueFF);
    // std::cout << "7   DEBUG POINT CONTROLLER\n";
};

void controller2(std::vector<raisim::ArticulatedSystem *> A1, LocoWrapper *loco_obj, MPC_dist* mpc_obj,size_t controlTick, bool* runMPC) {
    /////////////////////////////////////////////////////////////////////
    //////////////////////////// INITIALIZE
    /////////////////////////////////////////////////////////////////////
    size_t loco_kind = TROT;                        // Gait pattern to use
    size_t pose_kind = POSE_COMB;                   // Pose type to use (if loco_kind is set to POSE)
    size_t settling = 0.2*ctrlHz;                   // Settling down
    size_t duration = 0.8*ctrlHz;                   // Stand up
    size_t loco_start = settling + duration;        // Start the locomotion pattern

    double *tau;
    Eigen::VectorXd jointTorqueFF = Eigen::MatrixXd::Zero(TOTAL_DOF,1);
    Eigen::VectorXd jointPosTotal = Eigen::MatrixXd::Zero(TOTAL_DOF+1,1); // +1 is for 4th Component of Quaternion 
    Eigen::VectorXd jointVelTotal = Eigen::MatrixXd::Zero(TOTAL_DOF,1);
    static int conIndDes[4] = {1};
    
    /////////////////////////////////////////////////////////////////////
    //////////////////////////// UPDATE STATE
    /////////////////////////////////////////////////////////////////////

    A1.back()->getState(jointPosTotal, jointVelTotal);
    raisim::Mat<3,3> rotMat;
    Eigen::Matrix3d rotE;
    A1.back()->getBaseOrientation(rotMat);
    double rotMatrixDouble[9];
    for(size_t i=0;i<9;i++){
        rotMatrixDouble[i] = rotMat[i];
        rotE(i) = rotMat[i];
    }
    jointVelTotal.segment(3,3) = rotE.transpose()*jointVelTotal.segment(3,3); // convert to body frame, like robot measurements

    double jpos[18], jvel[18];
    Eigen::Matrix<double, 3, 1> eul;
    Eigen::Matrix<double, 4, 1> quat;
    quat = jointPosTotal.block(3,0,4,1);
    quat_to_XYZ(quat,eul);
    for(size_t i=0; i<3; ++i){
        jpos[i] = jointPosTotal(i);
        jvel[i] = jointVelTotal(i);
        jpos[i+3] = eul(i);
        jvel[i+3] = jointVelTotal(i+3);
    }
    for(size_t i=6; i<18; ++i){
        jpos[i] = jointPosTotal(i+1);
        jvel[i] = jointVelTotal(i);
    }

    int force[4] = {0};
    for(auto &con: A1.back()->getContacts()){
        int conInd = con.getlocalBodyIndex();
        force[conInd/3-1] = 0;
        // force[conInd/3-1] = con.getNormal().e().norm();
    }

    // kinEst2(force,conIndDes,jpos,jvel,rotE);

    /////////////////////////////////////////////////////////////////////
    //////////////////////////// CONTROL
    /////////////////////////////////////////////////////////////////////
    // Update the desired torques
    if(controlTick < settling){ // Settle down
        // loco_obj->posSetup(jointPosTotal.head(3));
        double temp[18] = {0};
        tau = temp;
        loco_obj->initStandVars(jointPosTotal.block(0,0,3,1),(int)duration);
    }
    else if(controlTick >= settling & controlTick <= loco_start){ // Start standing
    
        //================================================================================================
        //========================================== HIGH LEVEL ==========================================
        //================================================================================================
        updateData2(GET_DATA, HL_DATA, &HLData2);
        mpc_obj->updateState(HLData2.q, HLData2.dq, HLData2.ind, HLData2.toePos, HLData2.last_state1);
        // mpc_obj->plannedCycleIndex(TROT);

        if ((*runMPC == 1 || first_time0) && LLData2.domain < TOTALSTEPNUM)
        {
            mpc_obj->lipMPC_eventbase();
            *runMPC = 0;
            first_time0 = 0;

            HLData2.MPC_data_available = 1;
            HLData2.alpha_COM = mpc_obj->get_alphaCOM();
            HLData2.MPC_sol_ = mpc_obj->get_MPCsol();
            HLData2.domain = mpc_obj->getDomain();
            HLData2.last_state0 = mpc_obj->get_lastState();
            // std::cout << "TEST LAST_STATE 0: " << HLData0.last_state0.transpose(); std::cin.get();
        }
        updateData2(SET_DATA, HL_DATA, &HLData2);

        //================================================================================================
        //========================================== LOW LEVEL ===========================================
        //================================================================================================
        updateData2(GET_DATA, LL_DATA, &LLData2);
        loco_obj->set_MPC_DATA(LLData2.alpha_COM, LLData2.MPC_sol_, LLData2.MPC_data_available);
        // loco_obj->plannedCycleIndex(TROT);
        loco_obj->calcTau(jpos,jvel,rotMatrixDouble,force,STAND,controlTick, runMPC);
        tau = loco_obj->getTorque();

        LLData2.MPC_data_available = loco_obj->get_MPC_data_available();
        const int* contactMat = loco_obj->getConDes();
        Eigen::Matrix<double, 3, 4> toePosTmp = loco_obj->get_toePos();
        LLData2.toePos = toePosTmp;
        memcpy(LLData2.ind, contactMat,4*sizeof(int));
        updateData2(SET_DATA, LL_DATA, &LLData2);
    }
    else if(controlTick > loco_start){ // Start locomotion

        //================================================================================================
        //========================================== HIGH LEVEL ==========================================
        //================================================================================================
        size_t loco_kind = TROT;
        // loco_kind = (LLData.domain >= 40) ? STAND : TROT;
        // std::cout<<LLData.domain<<std::endl;

        // timer tset;
        // tic(&tset);cd 

        updateData2(GET_DATA, HL_DATA, &HLData2);
        mpc_obj->updateState(HLData2.q, HLData2.dq, HLData2.ind, HLData2.toePos, HLData2.last_state1);

        if ((*runMPC == 1 || sec_time) && LLData2.domain < TOTALSTEPNUM)
        {
            sec_time = 0;
            // mpc_obj->plannedCycleIndex(loco_kind);
            mpc_obj->lipMPC_eventbase();
            *runMPC = 0;

            HLData2.MPC_data_available = 1;
            HLData2.alpha_COM = mpc_obj->get_alphaCOM();
            HLData2.MPC_sol_ = mpc_obj->get_MPCsol();
            HLData2.domain = mpc_obj->getDomain();
            HLData2.last_state0 = mpc_obj->get_lastState();
            // std::cout << "HLData2.last_state0: " << HLData2.last_state0.transpose(); std::cin.get();
        }
        updateData2(SET_DATA, HL_DATA, &HLData2);

        // printf("%f\n",1000*toc(&tset));

        // cout << "MPC_sol: \n" << HLData.MPC_sol_;

        //================================================================================================
        //========================================== LOW LEVEL ==========================================
        //================================================================================================
        
        updateData2(GET_DATA, LL_DATA, &LLData2);

        loco_obj->set_MPC_DATA(LLData2.alpha_COM, LLData2.MPC_sol_, LLData2.MPC_data_available);
        loco_obj->calcTau(jpos,jvel,rotMatrixDouble,force,loco_kind,controlTick, runMPC);
        tau = loco_obj->getTorque();

        LLData2.MPC_data_available = loco_obj->get_MPC_data_available();
        const int* contactMat = loco_obj->getConDes();
        Eigen::Matrix<double, 3, 4> toePosTmp = loco_obj->get_toePos();
        LLData2.toePos = toePosTmp;
        memcpy(LLData2.ind, contactMat,4*sizeof(int));
        updateData2(SET_DATA, LL_DATA, &LLData2);
    }

    memcpy(data2.q,jpos,18*sizeof(double));
    memcpy(data2.dq,jvel,18*sizeof(double));

    const int* contactMat = loco_obj->getConDes();
    memcpy(data2.ind,contactMat,4*sizeof(int));
    memcpy(conIndDes,contactMat,4*sizeof(int));

    jointTorqueFF = Eigen::Map< Eigen::Matrix<double,18,1> >(tau,18);
    jointTorqueFF.block(0,0,6,1).setZero();

    // Set the desired torques
    A1.back()->setControlMode(raisim::ControlMode::FORCE_AND_TORQUE);

    A1.back()->setGeneralizedForce(jointTorqueFF);
};

void controller3(std::vector<raisim::ArticulatedSystem *> A1, LocoWrapper *loco_obj, MPC_dist* mpc_obj,size_t controlTick, bool* runMPC) {
    /////////////////////////////////////////////////////////////////////
    //////////////////////////// INITIALIZE
    /////////////////////////////////////////////////////////////////////
    size_t loco_kind = TROT;                        // Gait pattern to use
    size_t pose_kind = POSE_COMB;                   // Pose type to use (if loco_kind is set to POSE)
    size_t settling = 0.2*ctrlHz;                   // Settling down
    size_t duration = 0.8*ctrlHz;                   // Stand up
    size_t loco_start = settling + duration;        // Start the locomotion pattern

    double *tau;
    Eigen::VectorXd jointTorqueFF = Eigen::MatrixXd::Zero(TOTAL_DOF,1);
    Eigen::VectorXd jointPosTotal = Eigen::MatrixXd::Zero(TOTAL_DOF+1,1); // +1 is for 4th Component of Quaternion 
    Eigen::VectorXd jointVelTotal = Eigen::MatrixXd::Zero(TOTAL_DOF,1);
    static int conIndDes[4] = {1};
    
    /////////////////////////////////////////////////////////////////////
    //////////////////////////// UPDATE STATE
    /////////////////////////////////////////////////////////////////////

    A1.back()->getState(jointPosTotal, jointVelTotal);
    raisim::Mat<3,3> rotMat;
    Eigen::Matrix3d rotE;
    A1.back()->getBaseOrientation(rotMat);
    double rotMatrixDouble[9];
    for(size_t i=0;i<9;i++){
        rotMatrixDouble[i] = rotMat[i];
        rotE(i) = rotMat[i];
    }
    jointVelTotal.segment(3,3) = rotE.transpose()*jointVelTotal.segment(3,3); // convert to body frame, like robot measurements

    double jpos[18], jvel[18];
    Eigen::Matrix<double, 3, 1> eul;
    Eigen::Matrix<double, 4, 1> quat;
    quat = jointPosTotal.block(3,0,4,1);
    quat_to_XYZ(quat,eul);
    for(size_t i=0; i<3; ++i){
        jpos[i] = jointPosTotal(i);
        jvel[i] = jointVelTotal(i);
        jpos[i+3] = eul(i);
        jvel[i+3] = jointVelTotal(i+3);
    }
    for(size_t i=6; i<18; ++i){
        jpos[i] = jointPosTotal(i+1);
        jvel[i] = jointVelTotal(i);
    }

    int force[4] = {0};
    for(auto &con: A1.back()->getContacts()){
        int conInd = con.getlocalBodyIndex();
        force[conInd/3-1] = 0;
        // force[conInd/3-1] = con.getNormal().e().norm();
    }

    // kinEst3(force,conIndDes,jpos,jvel,rotE);

    /////////////////////////////////////////////////////////////////////
    //////////////////////////// CONTROL
    /////////////////////////////////////////////////////////////////////
    // Update the desired torques
    if(controlTick < settling){ // Settle down
        // loco_obj->posSetup(jointPosTotal.head(3));
        double temp[18] = {0};
        tau = temp;
        loco_obj->initStandVars(jointPosTotal.block(0,0,3,1),(int)duration);
    }
    else if(controlTick >= settling & controlTick <= loco_start){ // Start standing
    
        //================================================================================================
        //========================================== HIGH LEVEL ==========================================
        //================================================================================================
        updateData3(GET_DATA, HL_DATA, &HLData3);
        mpc_obj->updateState(HLData3.q, HLData3.dq, HLData3.ind, HLData3.toePos, HLData3.last_state0);
        // mpc_obj->plannedCycleIndex(TROT);

        if ((*runMPC == 1 || first_time1) && LLData3.domain < TOTALSTEPNUM)
        {   
            mpc_obj->lipMPC_eventbase();
            *runMPC = 0;
            first_time1 = 0;

            HLData3.MPC_data_available = 1;
            HLData3.alpha_COM = mpc_obj->get_alphaCOM();
            HLData3.MPC_sol_ = mpc_obj->get_MPCsol();
            HLData3.domain = mpc_obj->getDomain();
            HLData3.last_state1 = mpc_obj->get_lastState();
        }
        updateData3(SET_DATA, HL_DATA, &HLData3);

        //================================================================================================
        //========================================== LOW LEVEL ===========================================
        //================================================================================================
        updateData3(GET_DATA, LL_DATA, &LLData3);
        loco_obj->set_MPC_DATA(LLData3.alpha_COM, LLData3.MPC_sol_, LLData3.MPC_data_available);
        // loco_obj->plannedCycleIndex(TROT);
        loco_obj->calcTau(jpos,jvel,rotMatrixDouble,force,STAND,controlTick, runMPC);
        tau = loco_obj->getTorque();

        LLData3.MPC_data_available = loco_obj->get_MPC_data_available();
        const int* contactMat = loco_obj->getConDes();
        Eigen::Matrix<double, 3, 4> toePosTmp = loco_obj->get_toePos();
        LLData3.toePos = toePosTmp;
        memcpy(LLData3.ind, contactMat,4*sizeof(int));
        updateData3(SET_DATA, LL_DATA, &LLData3);
    }
    else if(controlTick > loco_start){ // Start locomotion

        //================================================================================================
        //========================================== HIGH LEVEL ==========================================
        //================================================================================================
        size_t loco_kind = TROT;
        // loco_kind = (LLData.domain >= 40) ? STAND : TROT;
        // std::cout<<LLData.domain<<std::endl;

        // timer tset;
        // tic(&tset);

        updateData3(GET_DATA, HL_DATA, &HLData3);
        mpc_obj->updateState(HLData3.q, HLData3.dq, HLData3.ind, HLData3.toePos, HLData3.last_state0);

        if ((*runMPC == 1 || sec_time) && LLData3.domain < TOTALSTEPNUM)
        {
            sec_time = 0;
            mpc_obj->plannedCycleIndex(loco_kind);
            mpc_obj->lipMPC_eventbase();
            *runMPC = 0;

            HLData3.MPC_data_available = 1;
            HLData3.alpha_COM = mpc_obj->get_alphaCOM();
            HLData3.MPC_sol_ = mpc_obj->get_MPCsol();
            HLData3.domain = mpc_obj->getDomain();
            HLData3.last_state1 = mpc_obj->get_lastState();
        }
        updateData3(SET_DATA, HL_DATA, &HLData3);
        // std::cout << "5   DEBUG POINT CONTROLLER\n";

        // printf("%f\n",1000*toc(&tset));

        // cout << "MPC_sol: \n" << HLData.MPC_sol_;

        //================================================================================================
        //========================================== LOW LEVEL ==========================================
        //================================================================================================
        
        updateData3(GET_DATA, LL_DATA, &LLData3);

        loco_obj->set_MPC_DATA(LLData3.alpha_COM, LLData3.MPC_sol_, LLData3.MPC_data_available);
        loco_obj->calcTau(jpos,jvel,rotMatrixDouble,force,loco_kind,controlTick, runMPC);
        tau = loco_obj->getTorque();

        LLData3.MPC_data_available = loco_obj->get_MPC_data_available();
        const int* contactMat = loco_obj->getConDes();
        Eigen::Matrix<double, 3, 4> toePosTmp = loco_obj->get_toePos();
        LLData3.toePos = toePosTmp;
        memcpy(LLData3.ind, contactMat,4*sizeof(int));
        updateData3(SET_DATA, LL_DATA, &LLData3);
        // std::cout << "6   DEBUG POINT CONTROLLER\n";
    }

    memcpy(data3.q,jpos,18*sizeof(double));
    memcpy(data3.dq,jvel,18*sizeof(double));

    const int* contactMat = loco_obj->getConDes();
    memcpy(data3.ind,contactMat,4*sizeof(int));
    memcpy(conIndDes,contactMat,4*sizeof(int));

    jointTorqueFF = Eigen::Map< Eigen::Matrix<double,18,1> >(tau,18);
    jointTorqueFF.block(0,0,6,1).setZero();

    // Set the desired torques
    A1.back()->setControlMode(raisim::ControlMode::FORCE_AND_TORQUE);

    A1.back()->setGeneralizedForce(jointTorqueFF);
    // std::cout << "7   DEBUG POINT CONTROLLER\n";
};

int main(int argc, char *argv[]) {

    std::fstream outFile("/home/basit/Desktop/A1_RaiSim_Outputs/failingDistances.txt", std::ios::out);

    const int NUMBER_OF_SIMS = 1;

    const float threshold = 0.4;

    bool shared_data_backed_up = 0;
    // ============================================================ //
    // =================== SETUP RAISIM/VISUALS =================== //
    // ============================================================ //
    /// create raisim world

    raisim::World::setActivationKey(raisim::loadResource("activation.raisim"));
    raisim::World world;
    world.setTimeStep(simfreq_raisim);

    raisim::OgreVis *vis = raisim::OgreVis::get();

    /// these method must be called before initApp
    vis->setWorld(&world);
    vis->setWindowSize(1792, 1200); // Should be evenly divisible by 16!!
    vis->setImguiSetupCallback(imguiSetupCallback); // These 2 lines make the interactable gui visible
    vis->setImguiRenderCallback(imguiRenderCallBack);
    vis->setKeyboardCallback(raisimKeyboardCallback);
    vis->setSetUpCallback(setupCallback);
    vis->setAntiAliasing(2);

    /// starts visualizer thread
    vis->initApp();
    /// create raisim objects
    raisim::TerrainProperties terrainProperties;
    terrainProperties.frequency = 0.0;
    terrainProperties.zScale = 0.0;
    terrainProperties.xSize = 300.0;
    terrainProperties.ySize = 300.0;
    terrainProperties.xSamples = 50;
    terrainProperties.ySamples = 50;
    terrainProperties.fractalOctaves = 0;
    terrainProperties.fractalLacunarity = 0.0;
    terrainProperties.fractalGain = 0.0;

    raisim::HeightMap *ground = world.addHeightMap(0.0, 0.0, terrainProperties);
    vis->createGraphicalObject(ground, "terrain", "checkerboard_blue");
    world.setDefaultMaterial(0.8, 0.0, 0.0); //surface friction could be 0.8 or 1.0
    vis->addVisualObject("extForceArrow", "arrowMesh", "red", {0.0, 0.0, 0.0}, false, raisim::OgreVis::RAISIM_OBJECT_GROUP);

    auto& list = vis->getVisualObjectList();

    float box_width = 0.25;
    float box_z = 0.15;
    auto boxd = world.addBox(box_width,box_width,0.3,0.1);
    vis->createGraphicalObject(boxd, "boxd", "green");

    // Mimic the rough terrain

    std::vector<std::string> colors = {"red", "green", "blue", "yellow", "orange", "purple", "pink", "brown", "grey", "white"};
    std::random_device rd1, rd2, rd3;  
    std::mt19937 gen1(rd1()), gen2(rd2()), gen3(rd3());
    std::uniform_real_distribution<> dis_height(0.0, 0.02); // Random heights between 0 and 1 cm
    std::uniform_real_distribution<> dis_width(0.03, 0.2); // Random width between 3 and 7 cm
    std::uniform_real_distribution<> dis_length(1, 5); // Random length between 3 and 7 cm
    std::uniform_int_distribution<> dis_color(0, colors.size() - 1); // Distribution for color indices

    int num_blocks = 24; // Number of blocks
    std::vector<raisim::Box*> wood_blocks;

    int num_rows = num_blocks/1; // Number of rows for block placement
    float spacing_factor = 1.5; // Adjust spacing between blocks

    if(rough_terrain_en) {
        for (size_t i = 0; i < num_blocks; i++) {
            float wood_height = dis_height(gen1); // Generate a random height for each box
            float wood_width = dis_width(gen2);   // Generate a random width for each box
            float wood_length = dis_length(gen3); // Generate a random length for each box
            int color_index = dis_color(gen1);    // Generate a random color index

            wood_blocks.push_back(world.addBox(wood_width, wood_length, wood_height, 0.1)); // Add the box to the world with random dimensions
            vis->createGraphicalObject(wood_blocks.back(), "wood_block" + std::to_string(i), colors[color_index]);

            // Set the position of the box in a grid pattern
            int row = i / num_rows;
            int col = i % num_rows;
            wood_blocks.back()->setPosition(col * wood_width * spacing_factor, row * wood_length * spacing_factor, wood_height / 2);
        }
    };


    // Populate the obstacles 
    // int number_of_obs = NUMBER_OF_OBS;

    std::vector<raisim::Box*> obstacles;
    if (obstacle_en)
    {
        for (size_t i = 0; i < NUMBER_OF_OBS; i++)
        {
            obstacles.push_back(world.addBox(box_width,box_width,0.3,0.1));
            vis->createGraphicalObject(obstacles.back(), "box"+std::to_string(i), "red");
        }
    }

    // ============================================================ //
    // ======================= SETUP Robot ======================== //
    // ============================================================ //
    std::vector<raisim::ArticulatedSystem*> A1;
    A1.push_back(world.addArticulatedSystem(raisim::loadResource("A1/A1_modified.urdf")));
    vis->createGraphicalObject(A1.back(), "A1");
    A1.back()->setName("A1_Robot");

    // ============================================================ //
    // ==================== SETUP MORE Robots ===================== //
    // ============================================================ //

    std::vector<raisim::ArticulatedSystem*> A2;
    A2.push_back(world.addArticulatedSystem(raisim::loadResource("A1/A1_modified.urdf")));
    vis->createGraphicalObject(A2.back(), "A2");
    A2.back()->setName("A1_Robot_2");

    for (size_t sim = 0; sim < NUMBER_OF_SIMS; ++sim)
    {
        boxd->setPosition(GOAL_X,GOAL_Y,box_z);
        Eigen::Matrix<double, 2*NUMBER_OF_AGENTS, 1> Pstart;
        // Define the matrices
        Eigen::Matrix<double, 2, NUMBER_OF_OBS> Pobs;
        Eigen::Matrix<double, 2, NUMBER_OF_OBS> Pobs_real;


        std::random_device rd;  // Will be used to obtain a seed for the random number engine
        std::mt19937 gen(rd()); // Standard mersenne_twister_engine seeded with rd()
        
        // Uniform distributions for the required ranges
        std::uniform_real_distribution<> dis_x(0, 9.0); // For x values 
        std::uniform_real_distribution<> dis_y(-2.0, 2.0); // For y values 
        std::uniform_real_distribution<> dis_uncertainty(-0.6, 0.6); // For uncertainties

        
        srand(static_cast<unsigned int>(time(nullptr)));

        // Pobs   <<       2.2,            1,         1,             1,              1,                3,             3,        3,       -100,
        //         0.9,            1,     -0.75,             2,          -1.75,              0.5,         -0.25,    -1.75,    -0.5 + 100; 


        for (int col = 0; col < Pobs.cols(); ++col) {
            // Generate random values in the specified ranges
            double randX = dis_x(gen);
            double randY = dis_y(gen);

            // Assign these values to Pobs
            Pobs(0, col) = randX;
            Pobs(1, col) = randY;

            if(!obstacle_en)
            {
                // Add 100 in y values to move obstacles out of the way 
                Pobs.row(1).array() += 100;
            }

            // Generate uncertainty
            double uncertaintyX = dis_uncertainty(gen);
            double uncertaintyY = dis_uncertainty(gen);

            // Add uncertainty to create Pobs_real
            Pobs_real(0, col) = Pobs(0, col) + uncertaintyX;
            Pobs_real(1, col) = Pobs(1, col) + uncertaintyY;

            // Ensure Pobs_real stays within bounds
            Pobs_real(0, col) = std::min(std::max(Pobs_real(0, col), 1.0), 9.0);
            Pobs_real(1, col) = std::min(std::max(Pobs_real(1, col), -3.0), 3.0);
        }
    
        for (size_t solver = 0; solver < 2; ++solver)
        {
            bool checkifrunMPC0 = 0;
            bool checkifrunMPC1 = 0;
            bool checkifrunMPC2 = 0;
            bool checkifrunMPC3 = 0;
            first_time0 = 1;
            first_time1 = 1;
            first_time2 = 1;
            first_time3 = 
            sec_time = 1;

            HLData0 = HLData0_Backup;
            updateData0(SET_DATA, HL_DATA, &HLData0);
            HLData1 = HLData1_Backup;
            updateData1(SET_DATA, HL_DATA, &HLData1);
            LLData0 = LLData0_Backup;
            updateData0(SET_DATA, LL_DATA, &LLData0);
            LLData1 = LLData1_Backup;
            updateData1(SET_DATA, LL_DATA, &LLData1);

            // ============================================================ //
            // =================== SETUP ENVIRONMENT ====================== //
            // ============================================================ //
            

            std::cout << "Pobs:\n" << Pobs << "\n\n";
            std::cout << "Pobs_real (with uncertainty):\n" << Pobs_real << "\n";

            Pstart << 0.0, 0.0, 0.0, -0.9, -1, 0, -1, -0.9;
        
            // Populate the obstacles 
            int obs_iter = 0;

            if (obstacle_en)
            {
                for (auto it = obstacles.begin(); it != obstacles.end(); ++it) {
                    raisim::Box* obstacle = *it;
                    // Use 'obstacle' as needed
                    obstacle->setPosition(Pobs_real(0,obs_iter),Pobs_real(1,obs_iter),box_z);
                    obstacle->setMass(0.2);
                    obstacle->setOrientation(0,0,0,1);
                    obs_iter++;
                }
            }

            // SET AGENT POSITIONS FOR THE NEW SIM
            A1.back()->setGeneralizedCoordinate({Pstart(0), Pstart(1), 0.12, 1, 0, 0, 0,
                                                0.0, Pi/3, -2.6, 0.0, Pi/3, -2.6, 0.0, Pi/3, -2.6, 0.0, Pi/3, -2.6});
            A1.back()->setControlMode(raisim::ControlMode::FORCE_AND_TORQUE);

            LocoWrapper* loco_obj = new LocoWrapper(argc,argv);
            loco_obj->setAgentID(0); //Agent ID = 0
            loco_obj->setPstart(Pstart);
            loco_obj->setPobs(Pobs);
            loco_obj->generateReferenceTrajectory();

            A2.back()->setGeneralizedCoordinate({Pstart(2), Pstart(3), 0.12, 1, 0, 0, 0,
                                                0.0, Pi/3, -2.6, 0.0, Pi/3, -2.6, 0.0, Pi/3, -2.6, 0.0, Pi/3, -2.6});
            A2.back()->setControlMode(raisim::ControlMode::FORCE_AND_TORQUE);

            LocoWrapper* loco_obj2 = new LocoWrapper(argc,argv);
            loco_obj2->setAgentID(1); //Agent ID = 1
            loco_obj2->setPstart(Pstart);
            loco_obj2->setPobs(Pobs);
            loco_obj2->generateReferenceTrajectory();

            // // Third Robot 
            // std::vector<raisim::ArticulatedSystem*> A3;
            // A3.push_back(world.addArticulatedSystem(raisim::loadResource("A1/A1_modified.urdf")));
            // vis->createGraphicalObject(A3.back(), "A3");
            // A3.back()->setGeneralizedCoordinate({Pstart(4), Pstart(5), 0.12, 1, 0, 0, 0,
            //                                     0.0, Pi/3, -2.6, 0.0, Pi/3, -2.6, 0.0, Pi/3, -2.6, 0.0, Pi/3, -2.6});
            // A2.back()->setControlMode(raisim::ControlMode::FORCE_AND_TORQUE);
            // A2.back()->setName("A1_Robot_3");
            // LocoWrapper* loco_obj3 = new LocoWrapper(argc,argv);
            // loco_obj3->setAgentID(2); //Agent ID = 1
            // loco_obj3->setPstart(Pstart);
            // loco_obj3->setPobs(Pobs);
            // loco_obj3->generateReferenceTrajectory();

            // // Fourth Robot 
            // std::vector<raisim::ArticulatedSystem*> A4;
            // A4.push_back(world.addArticulatedSystem(raisim::loadResource("A1/A1_modified.urdf")));
            // vis->createGraphicalObject(A4.back(), "A4");
            // A4.back()->setGeneralizedCoordinate({Pstart(6), Pstart(7), 0.12, 1, 0, 0, 0,
            //                                     0.0, Pi/3, -2.6, 0.0, Pi/3, -2.6, 0.0, Pi/3, -2.6, 0.0, Pi/3, -2.6});
            // A4.back()->setControlMode(raisim::ControlMode::FORCE_AND_TORQUE);
            // A4.back()->setName("A1_Robot_4");
            // LocoWrapper* loco_obj4 = new LocoWrapper(argc,argv);
            // loco_obj4->setAgentID(3); //Agent ID = 1
            // loco_obj4->setPstart(Pstart);
            // loco_obj4->setPobs(Pobs);
            // loco_obj4->generateReferenceTrajectory();

            // ============================================================ //
            // ================= VIEW AND RECORDING OPTIONS =============== //
            // ============================================================ //
            raisim::gui::showContacts = false;
            raisim::gui::showForces = false;
            raisim::gui::showCollision = false;
            raisim::gui::showBodies = true;

            std::string cameraview = "top";
            bool panX = true;                // Pan view with robot during walking (X direction)
            bool panY = false;                // Pan view with robot during walking (Y direction)
            bool record = false;             // Record?
            double startTime = 0*ctrlHz;    // RecoPstartrding start time
            double simlength = 60*ctrlHz;   // Sim end time
            double fps = 30;            
            std::string directory = "~/Desktop/Raisim_Simulations/";
            // std::string filename = "Payload_Inplace";
            std::string filename = "Success_Rate_Sim";
            // std::string filename = "inplace_sim";

            // ============================================================ //
            // ========================= VIEW SETUP ======================= //
            // ============================================================ //
            if(cameraview == "iso"){
                vis->getCameraMan()->getCamera()->setPosition(-1, -2, 0.5);
                vis->getCameraMan()->getCamera()->yaw(Ogre::Radian(4.5*Pi/6-Pi/2));
                vis->getCameraMan()->getCamera()->pitch(Ogre::Radian(Pi/2));
            }else if(cameraview == "isoside"){
                vis->getCameraMan()->getCamera()->setPosition(1.1, -2, 0.5);
                vis->getCameraMan()->getCamera()->yaw(Ogre::Radian(4*Pi/6-Pi/2));
                vis->getCameraMan()->getCamera()->pitch(Ogre::Radian(Pi/2));
            }else if(cameraview == "side"){
                vis->getCameraMan()->getCamera()->setPosition(0, -2, 0.5);
                vis->getCameraMan()->getCamera()->yaw(Ogre::Radian(0));
                vis->getCameraMan()->getCamera()->pitch(Ogre::Radian(Pi/2));
            }else if(cameraview == "front"){
                vis->getCameraMan()->getCamera()->setPosition(2, 0, 0.5);
                vis->getCameraMan()->getCamera()->yaw(Ogre::Radian(Pi/2));
                vis->getCameraMan()->getCamera()->pitch(Ogre::Radian(Pi/2));
            }else if(cameraview == "top"){
                vis->getCameraMan()->getCamera()->setPosition(2, -1, 6);
                vis->getCameraMan()->getCamera()->pitch(Ogre::Radian(0));
            }else{
                vis->getCameraMan()->getCamera()->setPosition(1, -3, 2.5);
                vis->getCameraMan()->getCamera()->pitch(Ogre::Radian(1.0));
            }
            unsigned long mask = 0;
            if(raisim::gui::showBodies) mask |= raisim::OgreVis::RAISIM_OBJECT_GROUP;
            if(raisim::gui::showCollision) mask |= raisim::OgreVis::RAISIM_COLLISION_BODY_GROUP;
            if(raisim::gui::showContacts) mask |= raisim::OgreVis::RAISIM_CONTACT_POINT_GROUP;
            if(raisim::gui::showForces) mask |= raisim::OgreVis::RAISIM_CONTACT_FORCE_GROUP;
            vis->setVisibilityMask(mask);

            if(panX) raisim::gui::panViewX = panX;
            if(panY) raisim::gui::panViewY = panY;
            
            // ============================================================ //
            // ========================== RUN SIM ========================= //
            // ============================================================ //
            const std::string name = directory+filename+"_"+cameraview+".mp4";
            vis->setDesiredFPS(fps);
            long simcounter = 0;
            static bool added = false;

            // ============================ MPC =========================== //
            MPC_dist* MPC_Agent0;
            MPC_dist* MPC_Agent1;
            // MPC_dist* MPC_Agent2;
            // MPC_dist* MPC_Agent3;

            MPC_Agent0 = new MPC_dist();
            MPC_Agent0->setAgentID(0);    //MPC_Agent1 = new MPC_dist();
            MPC_Agent0->setPstart(Pstart);
            MPC_Agent0->setPobs(Pobs);
            MPC_Agent0->setPobs_real(Pobs_real);
            // cout << "Can we reach here SIM 1"; cin.get();
            MPC_Agent0->generateReferenceTrajectory();
            // cout << "Can we reach here SIM 2"; cin.get();

            MPC_Agent1 = new MPC_dist();
            MPC_Agent1->setAgentID(1);    //MPC_Agent1 = new MPC_dist();
            MPC_Agent1->setPstart(Pstart);
            MPC_Agent1->setPobs(Pobs);
            MPC_Agent1->setPobs_real(Pobs_real);
            MPC_Agent1->generateReferenceTrajectory();

            if (solver == 0)
            {
                MPC_Agent0->use_snopt = false;
                MPC_Agent1->use_snopt = false;
            }
            else
            {
                MPC_Agent0->use_snopt = true;
                MPC_Agent1->use_snopt = true;
            }
            // MPC_Agent2 = new MPC_dist();
            // MPC_Agent2->setAgentID(2);    //MPC_Agent1 = new MPC_dist();
            // MPC_Agent2->setPstart(Pstart);
            // MPC_Agent2->setPobs(Pobs);
            // MPC_Agent2->setPobs_real(Pobs_real);
            // MPC_Agent2->generateReferenceTrajectory();

            // MPC_Agent3 = new MPC_dist();
            // MPC_Agent3->setAgentID(3);    //MPC_Agent1 = new MPC_dist();
            // MPC_Agent3->setPstart(Pstart);
            // MPC_Agent3->setPobs(Pobs);
            // MPC_Agent3->setPobs_real(Pobs_real);
            // MPC_Agent3->generateReferenceTrajectory();
            double failingDistance0 = 10;
            double failingDistance1 = 10;

            while (!vis->getRoot()->endRenderingQueued() && simcounter <= simlength){

                cout << "Sim: " << sim << "\n";
                size_t dist_start = 5*ctrlHz;               // Start the disturbance (if any)
                size_t dist_stop  = dist_start+5000;             // Stop the disturbance (if any)

                // disturbance(A1, list, dist_start, dist_stop, simcounter);

                controller0(A1,loco_obj, MPC_Agent0, simcounter, &checkifrunMPC0);
                // controller1(A2,loco_obj2, MPC_Agent1, simcounter, &checkifrunMPC1);

                MPC_Agent0->updateDistance_to_fail();
                MPC_Agent1->updateDistance_to_fail();

                failingDistance0 = MPC_Agent0->getDistance_to_fail();
                failingDistance1 = MPC_Agent1->getDistance_to_fail();
                // controller2(A3,loco_obj3, MPC_Agent2, simcounter, &checkifrunMPC2);
                // controller3(A4,loco_obj4, MPC_Agent3, simcounter, &checkifrunMPC3);
                // controller(A3,loco_obj3,simcounter);
                // controller(A4,loco_obj4,simcounter);

                world.integrate();        
                
                if (simcounter%30 == 0)
                    vis->renderOneFrame();
                
                if (!vis->isRecording() & record & simcounter>=startTime)
                    vis->startRecordingVideo(name);
                
                auto currentPos = vis->getCameraMan()->getCamera()->getPosition();
                if (raisim::gui::panViewX){
                    Eigen::VectorXd jointPosTotal(18 + 1);
                    Eigen::VectorXd jointVelTotal(18);
                    jointPosTotal.setZero();
                    jointVelTotal.setZero();
                    A1.back()->getState(jointPosTotal, jointVelTotal);
                    if (cameraview=="front"){
                        currentPos[0] = jointPosTotal(0)+2;
                        vis->getCameraMan()->getCamera()->setPosition(currentPos);
                    } else if(cameraview=="side"){
                        currentPos[0] = jointPosTotal(0);
                        vis->getCameraMan()->getCamera()->setPosition(currentPos);
                    } else if(cameraview=="iso"){
                        currentPos[0] = jointPosTotal(0)+2;
                        vis->getCameraMan()->getCamera()->setPosition(currentPos);
                    } else if(cameraview=="isoside"){
                        currentPos[0] = jointPosTotal(0)+1.1;
                        vis->getCameraMan()->getCamera()->setPosition(currentPos);
                    }
                }
                if (raisim::gui::panViewY){
                    Eigen::VectorXd jointPosTotal(18 + 1);
                    Eigen::VectorXd jointVelTotal(18);
                    jointPosTotal.setZero();
                    jointVelTotal.setZero();
                    A1.back()->getState(jointPosTotal, jointVelTotal);
                    if (cameraview=="front"){
                        currentPos[1] = jointPosTotal(1);
                        vis->getCameraMan()->getCamera()->setPosition(currentPos);
                    } else if(cameraview=="side"){
                        currentPos[1] = jointPosTotal(1)-2;
                        vis->getCameraMan()->getCamera()->setPosition(currentPos);
                    } else if(cameraview=="iso"){
                        currentPos[1] = jointPosTotal(1)-1;
                        vis->getCameraMan()->getCamera()->setPosition(currentPos);
                    } else if(cameraview=="isoside"){
                        currentPos[1] = jointPosTotal(1)-2;
                        vis->getCameraMan()->getCamera()->setPosition(currentPos);
                    }
                }
                simcounter++;
            }

            outFile << failingDistance0 << "," << failingDistance1 << std::endl;

            delete loco_obj;
            delete loco_obj2;
            delete MPC_Agent0;
            delete MPC_Agent1;
        }
    }
    outFile.close();

    // End recording if still recording
    if (vis->isRecording())
        vis->stopRecordingVideoAndSave();

    // // Writing to a text file
    // std::fstream outFile0("/home/basit/Desktop/A1_RaiSim_Outputs/failingDistances0.txt", std::ios::out);
    // if (outFile0.is_open()) {
    //     for (size_t i = 0; i < NUMBER_OF_SIMS; ++i) {
    //         outFile0 << failingDistances0[i];
    //         if (i < NUMBER_OF_SIMS - 1) {
    //             outFile0 << ",";
    //         }
    //     }
    //     outFile0.close();
    // } else {
    //     std::cerr << "Unable to open file for writing." << std::endl;
    // }

    // // Writing to a text file
    // std::fstream outFile1("/home/basit/Desktop/A1_RaiSim_Outputs/failingDistances1.txt", std::ios::out);
    // if (outFile1.is_open()) {
    //     for (size_t i = 0; i < NUMBER_OF_SIMS; ++i) {
    //         outFile1 << failingDistances1[i];
    //         if (i < NUMBER_OF_SIMS - 1) {
    //             outFile1 << ",";
    //         }
    //     }
    //     outFile1.close();
    // } else {
    //     std::cerr << "Unable to open file for writing." << std::endl;
    // }
    
    /// terminate the app
    vis->closeApp();
    return 0;
}