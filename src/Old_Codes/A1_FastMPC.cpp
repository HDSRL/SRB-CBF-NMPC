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

//HDSRL header
#include "fast_MPC.hpp"
#include "contactEst.hpp"
#include "utils.h"


void setupCallback() {
    auto vis = raisim::OgreVis::get();

    /// light
    vis->getLight()->setDiffuseColour(1, 1, 1);
    vis->getLight()->setCastShadows(false);
    Ogre::Vector3 lightdir(-3,-3,-0.5);
    lightdir.normalise();
    vis->getLightNode()->setDirection({lightdir});
    vis->setCameraSpeed(300);

    /// load  textures
    vis->addResourceDirectory("/home/kavehakbarihamed/raisim/workspace/A1_LL_Exp/rsc/material");
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

void controller(std::vector<raisim::ArticulatedSystem *> A1, FastMPC *loco_obj, ConEst *est_obj, size_t controlTick) {
    static auto& list = raisim::OgreVis::get()->getVisualObjectList();
    raisim::Vec<3> extForce;
    raisim::Mat<3,3> rot;
    raisim::Vec<3> dir;
    
    /////////////////////////////////////////////////////////////////////
    //////////////////////////// INITIALIZE
    /////////////////////////////////////////////////////////////////////
    size_t loco_kind = TROT;                         // Gait pattern to use
    size_t pose_kind = POSE_Z;                   // Pose type to use (if loco_kind is set to POSE)
    size_t settling = 0.2*ctrlHz;                   // Settling down
    size_t duration = 1*ctrlHz-settling;            // Stand up
    size_t loco_start = settling + duration;        // Start the locomotion pattern
    size_t dist_start = 40000*ctrlHz;  // Start the disturbance (if any)
    size_t dist_stop  = dist_start+200;             // Stop the disturbance (if any)


    loco_obj->timeSetup(settling, loco_start); // Basically sets the stand duration
    loco_obj->setPoseType(pose_kind);

    Eigen::VectorXd jointTorqueFF = Eigen::MatrixXd::Zero(TOTAL_DOF,1);
    Eigen::VectorXd jointPosTotal = Eigen::MatrixXd::Zero(TOTAL_DOF+1,1);
    Eigen::VectorXd jointVelTotal = Eigen::MatrixXd::Zero(TOTAL_DOF,1);

    /////////////////////////////////////////////////////////////////////
    //////////////////////////// UPDATE STATE
    /////////////////////////////////////////////////////////////////////
    A1.back()->getState(jointPosTotal, jointVelTotal);
    raisim::Mat<3,3> rotMat;
    A1.back()->getBaseOrientation(rotMat);
    double rotMatrixDouble[9];
    for(size_t i=0;i<9;i++){
        rotMatrixDouble[i] = rotMat[i];
    }
    Eigen::Map< Eigen::Matrix<double, 3, 3> > rotE(rotMatrixDouble, 3, 3);
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


    int* ctrlCon;
    int newDomain = loco_obj->impactDetected();
    Eigen::Matrix<int, 4, 1> curDomain = loco_obj->getDomainDes();
    float force[4] = {0};
    float fpos[4]  = {0};
    float phase    = 0;
    for(auto &con: A1.back()->getContacts()){
        int conInd = con.getlocalBodyIndex();
        force[conInd/3-1] = 50.0;
    }
    if (newDomain==1){
        est_obj->setDesDomain(curDomain.data());
    }
    est_obj->updateConState(fpos,phase,force);
    ctrlCon = est_obj->getCtrlCon();
    
    /////////////////////////////////////////////////////////////////////
    //////////////////////////// CONTROL
    /////////////////////////////////////////////////////////////////////
    double spd[3] = {0.0,0.0,0.0};
    // Update the desired torques
    if(controlTick < settling){ // Settle down
        loco_obj->posSetup(jointPosTotal.head(3));
        jointTorqueFF = Eigen::MatrixXd::Zero(TOTAL_DOF,1);
    }
    else if(controlTick >= settling & controlTick <= loco_start){ // Start standing
        loco_obj->compute(jpos,jvel,rotMatrixDouble,STAND,controlTick,spd,ctrlCon);
        jointTorqueFF = loco_obj->getJointTorqueCommand();
    }
    else if(controlTick > loco_start){ // Start locomotion
        loco_obj->compute(jpos,jvel,rotMatrixDouble,loco_kind,controlTick,spd,ctrlCon);
        jointTorqueFF = loco_obj->getJointTorqueCommand();
    }

    // Set the desired torques
    A1.back()->setControlMode(raisim::ControlMode::FORCE_AND_TORQUE);
    A1.back()->setGeneralizedForce(jointTorqueFF);
};

int main(int argc, char *argv[]) {
    // ============================================================ //
    // =================== SETUP RAISIM/VISUALS =================== //
    // ============================================================ //
    /// create raisim world
    raisim::World::setActivationKey("rsc/activation_Randall_Fawcett.raisim");
    raisim::World world;
    world.setTimeStep(simfreq_raisim);

    raisim::OgreVis *vis = raisim::OgreVis::get();

    /// these method must be called before initApp
    vis->setWorld(&world);
    vis->setWindowSize(1800, 1200); // Should be evenly divisible by 16!!
    vis->setImguiSetupCallback(imguiSetupCallback); // These 3 lines make the interactable gui visible
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
    vis->createGraphicalObject(ground, "terrain", "checkerboard_green");
    world.setDefaultMaterial(0.8, 0.0, 0.0); //surface friction could be 0.8 or 1.0
    vis->addVisualObject("extForceArrow", "arrowMesh", "red", {0.0, 0.0, 0.0}, false, raisim::OgreVis::RAISIM_OBJECT_GROUP);
    // vis->getSceneManager()->setSkyBox(true, "skybox/violentdays", 5, 8, Ogre::Quaternion(Ogre::Radian(45),{1,0,0}));

    auto& list = vis->getVisualObjectList();

    // ============================================================ //
    // ======================= SETUP Robot ======================== //
    // ============================================================ //
    std::vector<raisim::ArticulatedSystem*> A1;
    Eigen::VectorXd jointPgain = Eigen::MatrixXd::Zero(TOTAL_DOF,1);
    Eigen::VectorXd jointDgain = Eigen::MatrixXd::Zero(TOTAL_DOF,1);
    jointPgain.tail(TOTAL_IN).setConstant(900.0);
    jointDgain.tail(TOTAL_IN).setConstant(10.0);

    A1.push_back(world.addArticulatedSystem(raisim::loadResource("A1/A1_modified.urdf")));
    vis->createGraphicalObject(A1.back(), "A1");
    A1.back()->setGeneralizedCoordinate({0, 0, 0.12, 1, 0, 0, 0,
                                        0.0, Pi/3, -2.6, 0.0, Pi/3, -2.6, 0.0, Pi/3, -2.6, 0.0, Pi/3, -2.6});

    // A1.back()->setGeneralizedCoordinate({0, 0, 0.12, 0.9239, 0, 0, 0.3827,
    //                                     0.0, Pi/3, -2.6, 0.0, Pi/3, -2.6, 0.0, Pi/3, -2.6, 0.0, Pi/3, -2.6});

    A1.back()->setControlMode(raisim::ControlMode::FORCE_AND_TORQUE);
    A1.back()->setGeneralizedForce(Eigen::VectorXd::Zero(A1.back()->getDOF()));
    A1.back()->setPdGains(jointPgain, jointDgain);
    A1.back()->setName("A1_Robot");

    FastMPC* loco_obj = new FastMPC(argc,argv);
    ConEst* est_obj = new ConEst();

    // ============================================================ //
    // ================= VIEW AND RECORDING OPTIONS =============== //
    // ============================================================ //
    raisim::gui::showContacts = false;
    raisim::gui::showForces = false;
    raisim::gui::showCollision = false;
    raisim::gui::showBodies = true;

    std::string cameraview = "side";
    bool panX = true;                // Pan view with robot during walking (X direction)
    bool panY = false;                // Pan view with robot during walking (Y direction)
    bool record = false;             // Record?
    double startTime = 0*ctrlHz;    // Recording start time
    double simlength = 60*ctrlHz;   // Sim end time
    double fps = 30;            
    std::string directory = "/media/kavehakbarihamed/Data/A1_RaiSim_Outputs/";
    std::string filename = "LateralTrot_0_5ms";

    // ============================================================ //
    // ========================= VIEW SETUP ======================= //
    // ============================================================ //
    // NOTE: Pi is defined in /dynamics/dynamicsSupportFunctions.h
    // NOTE: This section still needs some work. 
    int TOTALSTEPNUM = 1;
    int STEPLENGTHX = 0.1;  
    int STEPLENGTHY = 0;
    if(cameraview == "iso"){
        // vis->getCameraMan()->getCamera()->setPosition(3, 3, 2);
        vis->getCameraMan()->getCamera()->setPosition(TOTALSTEPNUM*STEPLENGTHX-2, 4, 2);
        vis->getCameraMan()->getCamera()->yaw(Ogre::Radian(5*Pi/6));
        vis->getCameraMan()->getCamera()->pitch(Ogre::Radian(Pi/3));
    }else if(cameraview == "side"){
        vis->getCameraMan()->getCamera()->setPosition(0, 2, 0.5);
        vis->getCameraMan()->getCamera()->yaw(Ogre::Radian(Pi));
        vis->getCameraMan()->getCamera()->pitch(Ogre::Radian(Pi/2));
    }else if(cameraview == "front"){
        vis->getCameraMan()->getCamera()->setPosition(2, 0, 0.5);
        // vis->getCameraMan()->getCamera()->setPosition(TOTALSTEPNUM*STEPLENGTHX*2-1, TOTALSTEPNUM*STEPLENGTHY/4, 2);
        vis->getCameraMan()->getCamera()->yaw(Ogre::Radian(Pi/2));
        vis->getCameraMan()->getCamera()->pitch(Ogre::Radian(Pi/2));
    }else if(cameraview == "top"){
        vis->getCameraMan()->getCamera()->setPosition(TOTALSTEPNUM*STEPLENGTHX/4, TOTALSTEPNUM*STEPLENGTHY/4, 4.5);
        vis->getCameraMan()->getCamera()->yaw(Ogre::Radian(Pi/2));
        vis->getCameraMan()->getCamera()->roll(Ogre::Radian(Pi/2));
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
    while (!vis->getRoot()->endRenderingQueued() && simcounter <= simlength){

        controller(A1,loco_obj,est_obj,simcounter);
        world.integrate();        
        
        if (simcounter%30 == 0)
            vis->renderOneFrame();
        
        if (!vis->isRecording() & record & simcounter>=startTime)
            vis->startRecordingVideo(name);
        
        if (raisim::gui::panViewX){
            Eigen::VectorXd jointPosTotal(18 + 1);
            Eigen::VectorXd jointVelTotal(18);
            jointPosTotal.setZero();
            jointVelTotal.setZero();
            A1.back()->getState(jointPosTotal, jointVelTotal);
            if (cameraview=="front"){
                auto currentPos = vis->getCameraMan()->getCamera()->getPosition();
                currentPos[0] = jointPosTotal(0)+2;
                vis->getCameraMan()->getCamera()->setPosition(currentPos);
            } else if(cameraview=="side"){
                auto currentPos = vis->getCameraMan()->getCamera()->getPosition();
                currentPos[0] = jointPosTotal(0);
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
                auto currentPos = vis->getCameraMan()->getCamera()->getPosition();
                currentPos[1] = jointPosTotal(1);
                vis->getCameraMan()->getCamera()->setPosition(currentPos);
            } else if(cameraview=="side"){
                auto currentPos = vis->getCameraMan()->getCamera()->getPosition();
                currentPos[1] = jointPosTotal(1)+2;
                vis->getCameraMan()->getCamera()->setPosition(currentPos);
            }
        }
        simcounter++;
    }

    // End recording if still recording
    if (vis->isRecording())
        vis->stopRecordingVideoAndSave();

    /// terminate the app
    vis->closeApp();

    delete loco_obj;

    return 0;
}
