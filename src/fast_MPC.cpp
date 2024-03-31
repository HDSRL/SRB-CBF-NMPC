//
// Authror: Randy Fawcett on 06/2021.
//
// Copyright (c) Hybrid Dynamic Systems and Robot Locomotion Lab, Virginia Tech
//

#include "fast_MPC.hpp" 

FastMPC::FastMPC(int argc, char *argv[]): FILE_RW(true){
    if (FILE_RW){
        for (size_t i=0; i<FILE_CNT; i++){
            file[i].open(FILE_NAMES[i].c_str(), std::ios_base::out);
        }
    }

    params = new Parameters(argc,argv);

    // ======================================================= //
    // ================ Initialize Properties ================ //
    // ======================================================= //
    // General Parameters
    standStart_ = 0;
    locoStart_ = 3*ctrlHz;
    locoTick_ = 0.0;
    tickScale_ = 1.0;
    phaseVar_ = 0.0;
    impact_ = false;
    domainLength_ = 1*ctrlHz;

    mu_ = params->mu;
    mu_MPC_ = params->mu_MPC;
    Ts_opt_ = 1.0/MPC_Hz;
    standHeight_ = params->standHeight;
    startHeight_ = 0.12;
    startx_ = 0.0;
    finalX_ = -0.06*0;
    finalY_ = -0.04*0;
    h_sw_ =  params->swingHeight;
    stepLength_.setZero();
    mass_ = 12.4530;
    inertia_ << 0.01683993,   8.3902e-5, 0.000597679,
                 8.3902e-5, 0.056579028,   2.5134e-5,
               0.000597679,   2.5134e-5, 0.064713601;

    // Contact variables
    contactIndex_ << 1, 1, 1, 1;
    numContact_ = 4;

    // Quad Parameters
    D_.setZero();
    B_.setZero();
    H_.setZero();
    jointPosE_.setZero();
    jointVel_.setZero();
    R_.setZero();
    jointPosE_prev_.setZero();
    jointVel_prev_.setZero();
    toePos_.setZero();
    toeJaco_.setZero();
    toeJacoDot_.setZero();
    sat_.setZero(); 
    toeOffset_.setZero();
    hipOffset_.setZero();
    jointTorque_FF_.setZero();
    initToePos_.setZero();
    footTraj_.setZero();
    comDes_.setZero();
    desLock_.setZero();

    forceDes_.setZero(TOTAL_IN,1);
    QP_force_.setZero(TOTAL_IN,1);
    q_.setZero();
    dq_.setZero();
    ddq_.setZero();

    sat_ << 8, 20, 25;
//    sat_ << 22, 50, 50;
    toeOffset_ << 0.0, 0.0, 0.019;

    hipOffset_.block(0,FR_LEG,3,1) <<  0.183, -0.1321, 0;
    hipOffset_.block(0,FL_LEG,3,1) <<  0.183,  0.1321, 0;
    hipOffset_.block(0,RR_LEG,3,1) << -0.183, -0.1321, 0;
    hipOffset_.block(0,RL_LEG,3,1) << -0.183,  0.1321, 0;

    comFiltered_.setZero();
    comHist_.setZero();

    // Virtual Constraints
    y_ = Eigen::MatrixXd::Zero(6,1);
    dy_ = Eigen::MatrixXd::Zero(6,1);
    hd_ = Eigen::MatrixXd::Zero(6,1);
    dhd_ = Eigen::MatrixXd::Zero(6,1);
    ddhd_ = Eigen::MatrixXd::Zero(6,1);

    kpGain_ = params->kp;
    kdGain_ = params->kd;
};

void FastMPC::timeSetup(size_t stand_start, size_t loco_start){
    standStart_ = stand_start;
    locoStart_ = loco_start;
};

void FastMPC::posSetup(Eigen::Matrix<double,3,1> posCOM){
    startx_ = posCOM(0);
    startHeight_ = posCOM(2);
};

void FastMPC::setPoseType(size_t type){
    poseType_ = type;
};


// Robot functions (Kin, Dyn, Jaco)
void FastMPC::updateRobotState(const double q[18], const double  dq[18], const double R[9]){
    updateState(q, dq, R);
    updateDynamics();
    updateJacobian();
    updateJacobianDot();
    updateFwdKinematics();
};

void FastMPC::updateState(const double q[18], const double  dq[18], const double R[9]){
    for(int i=0; i<18; i++){
        jointPosE_(i) = q[i];
        jointVel_(i) = dq[i];
    }
    for(int i=0; i<9; i++){
        R_(i) = R[i];
    }
    jointVel_.block(3,0,3,1) = QuadModel::toWorld(jointVel_.block(3,0,3,1),R_);

    Eigen::Matrix<double, 3, 299> comTemp;
    comTemp = comHist_.block(0,0,3,299);
    comHist_.block(0,0,3,1) = jointVel_.block(0,0,3,1);
    comHist_.block(0,1,3,299) = comTemp;

    comFiltered_ = comHist_.rowwise().mean();
};

void FastMPC::updateDynamics(){
    // Update D_ matrix
    D_mat(D_.data(),jointPosE_.data());
    Dinv_ = D_.inverse();

    // Update nonlinear vector
    Eigen::Matrix<double, 18, 18> C_temp;
    G_vec(H_.data(),jointPosE_.data());
    // C_vec(C_temp.data(),jointPosE_.data(),jointVel_.data());
    // H_ += C_temp*jointVel_;

    // Update B matrix
    B_.setZero();
    B_.block(6,0,TOTAL_IN,TOTAL_IN)  = Eigen::MatrixXd::Identity(TOTAL_IN,TOTAL_IN);
};

void FastMPC::updateJacobian(){
    Eigen::Matrix<double, 3, TOTAL_DOF> J1,J2,J3,J4;

    // Toe jacobians
    J_FR_toe(J1.data(), jointPosE_.data());
    J_FL_toe(J2.data(), jointPosE_.data());
    J_RR_toe(J3.data(), jointPosE_.data());
    J_RL_toe(J4.data(), jointPosE_.data());
    toeJaco_.block<3,TOTAL_DOF>(0,0) = J1;
    toeJaco_.block<3,TOTAL_DOF>(3,0) = J2;
    toeJaco_.block<3,TOTAL_DOF>(6,0) = J3;
    toeJaco_.block<3,TOTAL_DOF>(9,0) = J4;

    // Hip jacobians
    J_FR_hip(J1.data(), jointPosE_.data());
    J_FL_hip(J2.data(), jointPosE_.data());
    J_RR_hip(J3.data(), jointPosE_.data());
    J_RL_hip(J4.data(), jointPosE_.data());
    hipJaco_.block<3,TOTAL_DOF>(0,0) = J1;
    hipJaco_.block<3,TOTAL_DOF>(3,0) = J2;
    hipJaco_.block<3,TOTAL_DOF>(6,0) = J3;
    hipJaco_.block<3,TOTAL_DOF>(9,0) = J4;
};

void FastMPC::updateJacobianDot(){
    Eigen::Matrix<double,3,1>dJ1,dJ2,dJ3,dJ4;

    // Toe jaco dot
    dJ_FR_toe(dJ1.data(), jointPosE_.data(), jointVel_.data());
    dJ_FL_toe(dJ2.data(), jointPosE_.data(), jointVel_.data());
    dJ_RR_toe(dJ3.data(), jointPosE_.data(), jointVel_.data());
    dJ_RL_toe(dJ4.data(), jointPosE_.data(), jointVel_.data());
    toeJacoDot_.block<3,1>(0,0) = dJ1;
    toeJacoDot_.block<3,1>(3,0) = dJ2;
    toeJacoDot_.block<3,1>(6,0) = dJ3;
    toeJacoDot_.block<3,1>(9,0) = dJ4;

    // Hip jaco dot
    dJ_FR_hip(dJ1.data(), jointPosE_.data(), jointVel_.data());
    dJ_FL_hip(dJ2.data(), jointPosE_.data(), jointVel_.data());
    dJ_RR_hip(dJ3.data(), jointPosE_.data(), jointVel_.data());
    dJ_RL_hip(dJ4.data(), jointPosE_.data(), jointVel_.data());
    hipJacoDot_.block<3,1>(0,0) = dJ1;
    hipJacoDot_.block<3,1>(3,0) = dJ2;
    hipJacoDot_.block<3,1>(6,0) = dJ3;
    hipJacoDot_.block<3,1>(9,0) = dJ4;
};

void FastMPC::updateFwdKinematics(){
    Eigen::Matrix<double,3,1>p1,p2,p3,p4;

    // Toe forward kinematics
    FK_FR_toe(p1.data(), jointPosE_.data());
    FK_FL_toe(p2.data(), jointPosE_.data());
    FK_RR_toe(p3.data(), jointPosE_.data());
    FK_RL_toe(p4.data(), jointPosE_.data());
    toePos_.block<3,1>(0,FR_LEG) = p1;
    toePos_.block<3,1>(0,FL_LEG) = p2;
    toePos_.block<3,1>(0,RR_LEG) = p3;
    toePos_.block<3,1>(0,RL_LEG) = p4;

    // Hip forward kinematics
    FK_FR_hip(p1.data(), jointPosE_.data());
    FK_FL_hip(p2.data(), jointPosE_.data());
    FK_RR_hip(p3.data(), jointPosE_.data());
    FK_RL_hip(p4.data(), jointPosE_.data());
    hipPos_.block<3,1>(0,FR_LEG) = p1;
    hipPos_.block<3,1>(0,FL_LEG) = p2;
    hipPos_.block<3,1>(0,RR_LEG) = p3;
    hipPos_.block<3,1>(0,RL_LEG) = p4;
};

void FastMPC::updateSwingState(){
    Js_.setZero(3*(4-numContact_),TOTAL_DOF);
    dJs_.setZero(3*(4-numContact_),1);
    Jc_.setZero(3*(numContact_),TOTAL_DOF);
    dJc_.setZero(3*(numContact_),1);

    size_t cnts = 0, cntc = 0;
    for (size_t i=0; i<4; i++){
        if(contactIndex_(i)==0){
            Js_.block(cnts,0,3,TOTAL_DOF) = toeJaco_.block(3*i,0,3,TOTAL_DOF);
            dJs_.block(cnts,0,3,1) = toeJacoDot_.block(3*i,0,3,1);
            cnts+=3;
        }else if(contactIndex_(i)==1){
            Jc_.block(cntc,0,3,TOTAL_DOF) = toeJaco_.block(3*i,0,3,TOTAL_DOF);
            dJc_.block(cntc,0,3,1) = toeJacoDot_.block(3*i,0,3,1);
            cntc+=3;
        }
    }
}

// Trajectory planning
void FastMPC::planTraj(size_t gait, size_t ctrlTick){
    if(gait==STAND){
        domainLength_ = locoStart_-standStart_;
        initToePos_ = toePos_;
        contactIndex_ << 1,1,1,1;
        numContact_ = 4;

        Eigen::Map< Eigen::Matrix<double, 12,1> > toeTemp(toePos_.data(),12,1);
        footTraj_.block(0,0,12,1) = toeTemp;
        footTraj_.block(0,1,12,1) = toeTemp;

        forceDes_ << 0, 0, mass_*9.81, 0, 0, mass_*9.81, 0, 0, mass_*9.81, 0, 0, mass_*9.81;
        forceDes_ *= 0.25;    
        redDes_ = comDes_;
    } 
    else if(gait==INPLACE_WALK){
    	domainLength_ = 0.16*ctrlHz; 
        if(impact_){
            Eigen::Map< Eigen::Matrix<double, 12,1> > toeTemp(toePos_.data(),12,1);
            footTraj_.block(0,0,12,1) = toeTemp;
            footTraj_.block(0,1,12,1) = toeTemp;
            
            static int n = 0;
            Eigen::Matrix<int, 4, 4> contactDomains;
            contactDomains << 0,1,1,1,
                              1,1,1,0,
                              1,0,1,1,
                              1,1,0,1;
            n += (n>3) ? -4 : 1;
            contactDes_ = contactDomains.block(n,0,1,4).transpose();
            contactIndex_ = contactDes_;
            numContact_ = 3;
        }
        redDes_(0) = 0.0;
    }
    else if(gait==TROT){
        double mpc_dt = (1.0/MPC_Hz)*CTRL_HORIZ;
        double dt = (1.0/LL_Hz);
        static bool standTrigger = false;
        static bool lockX = false, lockY = true, lockYaw = true;
        static double xlock = 0, ylock = 0, yawLock = 0;
        double normMax = 0.04;
        double domLength_sec = 0.16;
        double maxStepRear   = 0.16;
        double maxStepFront  = 0.16;
        domainLength_ = domLength_sec*ctrlHz;
        double maxVel[3] = {params->fwdSpeed,params->latSpeed,params->yawSpeed};

        static Eigen::Matrix<double, 3, 1> desVel = Eigen::Matrix<double, 3, 1>::Zero(3,1);
        static Eigen::Matrix<double, 3, 1> desOmega = Eigen::Matrix<double, 3, 1>::Zero(3,1);
        Eigen::Matrix<double, 3, 1> delPos;

        // ================================================================================= //
        // If impact, change the contact index, the step length/foot traj, and desired force //
        // ================================================================================= //
        int dir = 0;
        if(impact_){      	
            if(maxVel[0]<0){
            	lockX = false;
            	desVel(0) += desVel(0)>maxVel[0] ? -0.1 : 0;
            } else if(maxVel[0]>0){
            	lockX = false;
            	desVel(0) += desVel(0)<maxVel[0] ? 0.1 : 0;
			} else if(maxVel[0]==0){
				if (!lockX){
					xlock = comDes_(0);
					lockX = true;
				}
				desVel(0) = 0;
			}
            if(maxVel[1]<0){
            	lockY = false;
            	desVel(1) += desVel(1)>maxVel[1] ? -0.01 : 0;
            } else if(maxVel[1]>0){
            	lockY = false;
            	desVel(1) += desVel(1)<maxVel[1] ? 0.01 : 0;
			} else if(maxVel[1]==0){
				if (!lockY){
					ylock = comDes_(1);
					lockY = true;
				}
				desVel(1) = 0;
			}
            if(maxVel[2]<0){
            	lockYaw = false;
            	desOmega(2) += desOmega(2)>maxVel[2] ? -0.01 : 0;
            } else if(maxVel[2]>0){
            	lockYaw = false;
            	desOmega(2) += desOmega(2)<maxVel[2] ? 0.01 : 0;
			} else if(maxVel[2]==0){
				if (!lockY){
					yawLock = comDes_(8);
					lockYaw = true;
				}
				desOmega(2) = 0;
			}
			
			
            if(desVel(0)==0 && desVel(1)==0 && desOmega(2)==0 && comFiltered_.block(0,0,2,1).norm()<normMax){
                standTrigger = true;
            } else {
                standTrigger = false;
            }

            // Update the contact matrix
            numContact_ = 2; 
            if(contactDes_(0)==1){
                contactDes_ << 0, 1, 1, 0;}
            else{
           	    contactDes_ << 1, 0, 0, 1;}
            contactIndex_ = contactDes_;

            // ================================================ //
            // Marc Raibert foothold selection (similar)
            // ================================================ //
            Eigen::Matrix<double, 3, 1> velErr, kk;
            kk << 0.04, 0.001, 0.0;
            stepLength_.setZero();
            velErr = QuadModel::toBody(comFiltered_,R_)-desVel;
            velErr(0) *= kk(0);
            velErr(1) *= kk(1);
            stepLength_ = domLength_sec*QuadModel::toBody(comFiltered_,R_)/2+velErr;
            stepLength_ = QuadModel::toWorld(stepLength_,R_);

            Eigen::Map<Eigen::MatrixXd> toeTemp(toePos_.data(),12,1);
            footTraj_.block(0,0,12,1) = toeTemp;
        }

        if(standTrigger & !params->neverStopTrot){
        	comDes_.block(0,0,2,1) = jointPosE_.block(0,0,2,1);
            planTraj(STAND,ctrlTick);
            domainLength_ = domLength_sec*ctrlHz;
            return;
        }

        // ================================================ //
        // Update the desired position/velocity
        // ================================================ //
        comDes_.block(0,0,3,1) = jointPosE_.block(0,0,3,1) + desVel*dt; // CHANGE BACK WHEN USING MPC
        // comDes_(1) /= 1.5;
        if (desVel(0) == 0){
            comDes_(0) = (2*desLock_(0)+xlock)/3;
        }
        if (desVel(1) == 0){
            comDes_(1) = (2*jointPosE_(1)+ylock)/3;
        }
        comDes_(2) = standHeight_;
        comDes_.block(3,0,3,1) = desVel;
        comDes_.block(6,0,3,1) << 0,-2*Pi/180,jointPosE_(5) + desOmega(2)*dt;
        // comDes_.block(6,0,3,1) << 0,0,0;
        comDes_.block(9,0,3,1) << desOmega;
        if (desOmega(2) == 0){
            comDes_(8) = yawLock;
        }

        redDes_.block(0,0,3,1) = jointPosE_.block(0,0,3,1) + desVel*mpc_dt;
        // redDes_(1) = 0;
        redDes_(2) = standHeight_;
        redDes_.block(3,0,3,1) = desVel;
        redDes_.block(6,0,3,1) << 0,0, jointPosE_(5) + desOmega(2)*mpc_dt;
        // redDes_.block(6,0,3,1) << 0,0,0;
        redDes_.block(9,0,3,1) << desOmega;
    } 
    else if(gait==INPLACE){
    	double domLength_sec = 0.16;
        domainLength_ = domLength_sec*ctrlHz;
        if(impact_){
            Eigen::Map< Eigen::Matrix<double, 12,1> > toeTemp(toePos_.data(),12,1);
            footTraj_.block(0,0,12,1) = toeTemp;
            footTraj_.block(0,1,12,1) = toeTemp;
            if(contactDes_(0)==1){
                contactDes_ << 0, 1, 1, 0;}
            else{
                contactDes_ << 1, 0, 0, 1;}
            contactIndex_ = contactDes_;
            numContact_ = 2;
        }
        // ================================================ //
        // Marc Raibert foothold selection (similar)
        // ================================================ //
        double velErr, kk;
        kk = 0.04; 
        stepLength_.setZero();
        velErr = kk*comFiltered_(0);
        stepLength_(0) = domLength_sec*comFiltered_(0)/2+velErr;
        redDes_(0) = 0.0;
    }
    else if(gait==TAP){
        contactIndex_ << 1, 0, 1, 1;
        domainLength_ = 0.25*ctrlHz;
        stepLength_ << 0.0, 0.0, 0.0;
        if(impact_){
            numContact_ = 3; 
            Eigen::Map< Eigen::Matrix<double, 12,1> > toeTemp(toePos_.data(),12,1);
            footTraj_.block(0,0,12,1) = toeTemp;
            footTraj_.block(0,1,12,1) = toeTemp;
        }
    }
    else if(gait==POSE){
        redDes_ = desLock_;
        redDes_.block(3,0,3,1) << 0,0,0;
        redDes_.block(9,0,3,1) << 0,0,0;
        double t = 1.0*ctrlTick/(1.0*ctrlHz);
        static double t_init = t;
        if(poseType_==POSE_X){
			double freq = 0.8*Pi;
            double mag = 0.04;
            redDes_(0) += mag*sin(freq*(t-t_init));
            redDes_(3) += mag*freq*cos(freq*(t-t_init));
        }
        else if(poseType_==POSE_Y){
            double freq = 0.8*Pi;
            double mag = 0.04;
            redDes_(1) += mag*sin(freq*(t-t_init));
            redDes_(4) += mag*freq*cos(freq*(t-t_init));
        }
        else if(poseType_==POSE_Z){
            double freq = 0.8*Pi;
            double mag = 0.05;
            redDes_(2) += mag*cos(freq*(t-t_init))-mag;
            redDes_(5) += -1.0*mag*freq*sin(freq*(t-t_init));
        }
        else if(poseType_==POSE_ROLL){
            double freq = 0.8*Pi;
			double mag = 0.3491;
            redDes_(6) += mag*sin(freq*(t-t_init));
            redDes_(9) += mag*freq*cos(freq*(t-t_init));
        }
        else if(poseType_==POSE_PITCH){
            double freq = 0.8*Pi;
            double mag = 0.17453;
            redDes_(7) += mag*sin(freq*(t-t_init));
            redDes_(10) += mag*freq*cos(freq*(t-t_init));
        }
        else if(poseType_==POSE_YAW){
            double freq = 0.8*Pi;
            double mag = 0.13963;
            redDes_(8) += mag*sin(freq*(t-t_init));
            redDes_(11) += mag*freq*cos(freq*(t-t_init));
        }
        else if(poseType_==POSE_COMB){
        	double freq = 0.6*Pi;
        	double mag = 0.3491;
        	redDes_(7) += mag*sin(freq*(t-t_init));
        	redDes_(10) += mag*freq*cos(freq*(t-t_init));
        	
        	redDes_(8) += mag*cos(freq*(t-t_init));
        	redDes_(11) += -mag*freq*sin(freq*(t-t_init));
        }
        // comDes_ = redDes_;
    }
    desLock_ = comDes_;
    impact_ = false;
};

// CHANGE FOR MPC, USE redDes_
void FastMPC::updateDesiredForce(Eigen::Matrix<double, 6,1> &desAcc, Eigen::MatrixXd &desForce){
    // Parse desired vector
    Eigen::Vector3d pd = comDes_.block(0,0,3,1);
    Eigen::Vector3d wd = comDes_.block(9,0,3,1);

    // Setup
    Eigen::Vector3d rd;
    Eigen::Matrix3d rd_hat_temp;
    Eigen::MatrixXd rd_hat(3,12);
    for(int i=0;i<4;i++){
        rd = toePos_.block(0,i,3,1)-pd;
        hatmap(rd,rd_hat_temp);
        rd_hat.block(0,3*i,3,3) = rd_hat_temp;
    }

    // Calculate wd_hat
    Eigen::Matrix3d wd_hat;
    hatmap(wd, wd_hat);

    Eigen::MatrixXd H, b;
    Eigen::Vector3d g;
    Eigen::MatrixXd force(3,12);
    Eigen::MatrixXd torque(3,12);
    g << 0,0,9.81;
    H.setZero(6,12);
    b.setZero(6,1);


    // NOTE: all columns of non-contacting legs are set to zero
    force.setZero();
    torque.setZero();
    for(int i=0; i<4; i++){
        if(contactIndex_(i)==1){
            force.block(0,3*i,3,3) = Eigen::MatrixXd::Identity(3,3);
            torque.block(0,3*i,3,3) = rd_hat.block(0,3*i,3,3);
        }
    }
    H << force, torque;
    b.block(0,0,3,1) = mass_*(desAcc.block(0,0,3,1) + g);
    b.block(3,0,3,1) = inertia_*desAcc.block(3,0,3,1) + wd_hat*inertia_*wd;

    double* optimVec = new double[12];
    Eigen::Matrix<double, 12, 12> P_QP;
    Eigen::Matrix<double, 12,  1> c_QP;
    Eigen::Matrix<double,  1, 12> A_QP;
    Eigen::Matrix<double,  1,  1> b_QP;
    Eigen::MatrixXd G_QP(5*numContact_,12);
    Eigen::MatrixXd h_QP(5*numContact_,1);
    P_QP = H.transpose()*H;
    c_QP = -H.transpose()*b;
    A_QP.setZero();
    b_QP.setZero();

    // Eigen::MatrixXd Gc(20,12); // contact constraints
    Eigen::MatrixXd gc(5,3);
    G_QP.setZero();
    double mu = mu_MPC_;
    gc << 1,  0, -mu/sqrt(2),
         -1,  0, -mu/sqrt(2),
          0,  1, -mu/sqrt(2),
          0, -1, -mu/sqrt(2),
          0,  0,          -1;

    size_t cnt = 0;
    for(size_t i=0; i<4; i++){
        if (contactIndex_(i)==1){
            G_QP.block(5*cnt,3*i,5,3) = gc;
            cnt++;
        }
    }
    h_QP.setZero();

    iswiftQp_e(P_QP, c_QP, A_QP, b_QP, G_QP, h_QP, optimVec);
    desForce.setZero(12,1);
    for(size_t i=0; i<12; i++){
        desForce(i) = optimVec[i];
    }
    delete[] optimVec;
}


// Low Level Controller
void FastMPC::impactDetection(size_t gait, size_t ctrlTick, int ctrlCon[4]){
    double tickTemp = locoTick_;
    phaseVar_ = calcPhaseVar(tickTemp,0,1.0*domainLength_);
    int totalCon = 0;
    for(int i=0; i<4; ++i){totalCon+=ctrlCon[i];}
    if(phaseVar_>1.0 & phaseVar_<1.0 & gait!=STAND){
        Eigen::Matrix<double, 18, 1> velDiff;
        velDiff = jointVel_-jointVel_prev_;
        if(velDiff.norm()>0.02*jointVel_.norm()){
            impact_ = true;
            locoTick_ = 0;
            phaseVar_ = 0;
            std::cout<<"Impact detected"<<std::endl;
        }
    }else if(phaseVar_>=1.05 & gait!=STAND){
        impact_ = true;
        locoTick_ = 0;
        phaseVar_ = 0;
        std::cout<<"Phase var >= 1, impact assumed"<<std::endl;
    }else if (totalCon==4 & gait!=STAND){
        impact_ = true;
        locoTick_ = 0;
        if (phaseVar_<1){
            std::cout<<"Early Impact detected. Phase: "<<phaseVar_<<std::endl;
        }
        phaseVar_ = 0;
    }

    if(ctrlTick<standStart_){
        locoTick_ = 0;
    }
};

void FastMPC::computeVirtualConstraints(size_t gait, size_t ctrlTick){
    size_t outDim = 6+3*(4-numContact_);
    h0_.setZero(outDim,1);
    dh0_.setZero(outDim,1);
    H0_.setZero(outDim,TOTAL_DOF);
    dH0_.setZero(outDim,1);
    hd_.setZero(outDim,1);
    dhd_.setZero(outDim,1);
    ddhd_.setZero(outDim,1);
    y_.setZero(outDim,1);
    dy_.setZero(outDim,1);

    H0_.block(0,0,6,6) = Eigen::MatrixXd::Identity(6,6);
    h0_ = H0_*jointPosE_;

    if (gait==STAND){
        double phase = calcPhaseVar(1.0*ctrlTick,1.0*standStart_,1.0*locoStart_);
        double s = (phase>1) ? 1 : ((phase<0) ? 0 : phase);

        Eigen::Vector3d traj_x, traj_y, traj_z;
        Eigen::VectorXd alpha_x, alpha_y, alpha_z;
        alpha_x.setZero(8);
        alpha_y.setZero(8);
        alpha_z.setZero(8);

        double xFinal = startx_+finalX_;
        double starty_ = 0;
        double yFinal = finalY_;
        alpha_x << startx_,startx_,startx_,
                 startx_+(xFinal-startx_)/4,
                 startx_+3*(xFinal-startx_)/4,
                 xFinal,xFinal,xFinal;
        alpha_y << starty_,starty_,starty_,
                 starty_+(yFinal-starty_)/4,
                 starty_+3*(yFinal-starty_)/4,
                 yFinal,yFinal,yFinal;
        alpha_z << startHeight_,startHeight_,startHeight_,
                 startHeight_+(standHeight_-startHeight_)/4,
                 startHeight_+3*(standHeight_-startHeight_)/4,
                 standHeight_,standHeight_,standHeight_;

        calcBezierAll(alpha_x, s, traj_x);
        calcBezierAll(alpha_y, s, traj_y);
        calcBezierAll(alpha_z, s, traj_z);

        // comDes_ -> pos, vel, theta, omega
        comDes_.block(0,0,3,1) << traj_x(0), traj_y(0), traj_z(0);
        comDes_.block(3,0,3,1) << traj_x(1), traj_y(1), traj_z(1);
        comDes_.block(6,0,3,1) << 0, 0, 0;
        comDes_.block(9,0,3,1) << 0, 0, 0;

        hd_.block(0,0,3,1)   << traj_x(0), traj_y(0), traj_z(0);
        dhd_.block(0,0,3,1)  << traj_x(1), traj_y(1), traj_z(1);
        ddhd_.block(0,0,3,1) << traj_x(2), traj_y(2), traj_z(2);
        hd_.block(3,0,3,1)   << 0, 0, 0;
        dhd_.block(3,0,3,1)  << 0, 0, 0;
        ddhd_.block(3,0,3,1) << 0, 0, 0;
    }
    else {
        hd_.block(0,0,3,1) = redDes_.block(0,0,3,1);
        hd_.block(3,0,3,1) = redDes_.block(6,0,3,1);
        dhd_.block(0,0,3,1) = redDes_.block(3,0,3,1);
        dhd_.block(3,0,3,1) = redDes_.block(9,0,3,1);
        ddhd_.block(0,0,3,1) << 0,0,0;
        ddhd_.block(3,0,3,1) << 0,0,0;
        
        if(gait==TROT){
        	hd_.block(0,0,3,1) = comDes_.block(0,0,3,1);
        	hd_.block(3,0,3,1) = comDes_.block(6,0,3,1);
            dhd_.block(0,0,3,1) = comDes_.block(3,0,3,1);
            dhd_.block(3,0,3,1) = comDes_.block(9,0,3,1);
        }

        Eigen::VectorXd ddq(18);
        Eigen::VectorXd hipAcc(3);
        Eigen::VectorXd hipVel(3);
        Eigen::VectorXd ax(4);
        Eigen::VectorXd dax(4);
        Eigen::VectorXd ddax(4);
        Eigen::VectorXd ay(4);
        Eigen::VectorXd day(4);
        Eigen::VectorXd dday(4);
        Eigen::VectorXd az(8);
        Eigen::Vector3d tx, ty, tz;

        size_t cnts = 0;
        double to = toeOffset_(2);
        double ds = (1.0*ctrlHz)/domainLength_;
        for(size_t i=0; i<4; i++){
            if(contactIndex_(i)==0){
                h0_.block(6+cnts,0,3,1) = toePos_.block(0,i,3,1);
                H0_.block(6+cnts,0,3,TOTAL_DOF) = toeJaco_.block(3*i,0,3,TOTAL_DOF);
                dH0_.block(6+cnts,0,3,1) = toeJacoDot_.block(3*i,0,3,1);

                // Swing leg to follow time varying bezier
                hipVel = hipJaco_.block(3*i,0,3,18)*jointVel_;
                hipAcc = ( hipJaco_.block(3*i,0,3,18)*ddq_ + hipJacoDot_.block(3*i,0,3,1) );
                
                double DL = domainLength_/(1.0*ctrlHz);
                double tune = (2*(i%2==0)-1)*0.02*0;
                
                ax << footTraj_(3*i,0), footTraj_(3*i,0), hipPos_(0,i)+stepLength_(0), hipPos_(0,i)+stepLength_(0);
                dax << 0, 0, hipVel(0), hipVel(0);
                ddax << 0, 0, hipAcc(0), hipAcc(0);
                calcVaryingBezierAll(ax,dax,ddax,DL,phaseVar_,tx);
                
                ay << footTraj_(3*i+1,0), footTraj_(3*i+1,0), hipPos_(1,i)+stepLength_(1)+tune, hipPos_(1,i)+stepLength_(1)+tune;
                day << 0, 0, hipVel(1), hipVel(1);
                dday << 0, 0, hipAcc(1), hipAcc(1);
                calcVaryingBezierAll(ay,day,dday,DL,phaseVar_,ty);
                
                az << footTraj_(3*i+2,0), footTraj_(3*i+2,0), h_sw_, h_sw_, h_sw_, h_sw_, to, to;
                calcBezierAll(az, phaseVar_, tz);

                // Save foot traj
                hd_.block(6+cnts,0,3,1)   << tx(0), ty(0), tz(0);
                dhd_.block(6+cnts,0,3,1)  << tx(1), ty(1), tz(1);
                ddhd_.block(6+cnts,0,3,1) << tx(2), ty(2), tz(2);

                // Scale foot traj using domain length
                dhd_.block(6+cnts, 0, 3, 1) *= ds;
                ddhd_.block(6+cnts, 0, 3, 1) *= ds*ds;

                cnts+=3;
            }
        }
    }
    dh0_ = H0_*jointVel_;

    y_ = h0_-hd_;
    dy_ = dh0_-dhd_;
};
        
void FastMPC::computeTorque(){
    // ====================================================================== //
    // =============================== Setup ================================ //
    // ====================================================================== //
    size_t useCLF = (params->useCLF==1) ? 1 : 0;
    size_t conDim = 3*numContact_;
    size_t outDim = 6+3*(4-numContact_);
    size_t numDec = conDim+TOTAL_IN+outDim+useCLF;
    Eigen::MatrixXd P_QP = Eigen::MatrixXd::Zero(numDec,numDec);
    Eigen::MatrixXd c_QP = Eigen::MatrixXd::Zero(numDec,1);
    Eigen::MatrixXd A_QP = Eigen::MatrixXd::Zero(conDim+outDim,numDec);
    Eigen::MatrixXd b_QP = Eigen::MatrixXd::Zero(conDim+outDim,1);
    Eigen::MatrixXd G_QP = Eigen::MatrixXd::Zero(2*TOTAL_IN+2*outDim+5*numContact_+useCLF,numDec);
    Eigen::MatrixXd h_QP = Eigen::MatrixXd::Zero(2*TOTAL_IN+2*outDim+5*numContact_+useCLF,1);

    LL_Cost(P_QP,c_QP,outDim,conDim,numDec,useCLF);
    LL_Constraints(A_QP,b_QP,G_QP,h_QP,outDim,conDim,numDec,useCLF);

    // ====================================================================== //
    // ============================ Solve the QP ============================ //
    // ====================================================================== //
    optimOut_ = new double[numDec];
    iswiftQp_e(P_QP,c_QP,A_QP,b_QP,G_QP,h_QP,optimOut_);
    Eigen::MatrixXd sol(numDec,1);
    for(int i=0; i<numDec; ++i){
        sol(i,0) = optimOut_[i];
    }

    // ====================================================================== //
    // ======================== Parse the QP solution ======================= //
    // ====================================================================== //
    Eigen::MatrixXd force  = sol.block(0,0,conDim,1);
    Eigen::MatrixXd torque = sol.block(conDim,0,TOTAL_IN,1);
    Eigen::MatrixXd delta  = sol.block(conDim+TOTAL_IN,0,outDim,1);

    jointTorque_FF_.setZero();
    jointTorque_FF_.block(6,0,TOTAL_IN,1) = torque;
    QP_force_.setZero(12,1);
    size_t cnt = 0;
    for(size_t i=0; i<4; i++){
        if(contactIndex_(i)==1){
            for(size_t j=0; j<3; j++){
                QP_force_(3*i+j) = force(3*cnt+j);
            }
            cnt++;
        }
    }
    
    if(useCLF==1){
    	Eigen::MatrixXd dV_temp = LgV_*delta;
    	dV_ = LfV_ + dV_temp(0);
    }

    delete[] optimOut_;

    // ====================================================================== //
    // ============================ Swing Leg PD ============================ //
    // ====================================================================== //
    if (conDim<12){
        Eigen::MatrixXd p_d = Eigen::MatrixXd::Zero(12-conDim,1);
        Eigen::MatrixXd v_d = Eigen::MatrixXd::Zero(12-conDim,1);
        size_t cnts = 0;
        for (size_t i=0; i<4; i++){
            if(contactIndex_(i)==0){
                p_d.block(cnts,0,3,1) = (hd_.block(6+cnts,0,3,1)-toePos_.block(0,i,3,1));
                v_d.block(cnts,0,3,1) = (dhd_.block(6+cnts,0,3,1)-toeJaco_.block(3*i,0,3,TOTAL_DOF)*jointVel_);
                cnts+=3;
            }
        }
        double Kp = 400;
        double Kd = 40;
        jointTorque_FF_ += Js_.transpose()*(Kp*p_d + Kd*v_d);
    }
    
    
    // ====================================================================== //
    // ======================= Calculate Joint Angles ======================= //
    // ====================================================================== //
    ddq_ = Dinv_*(B_*jointTorque_FF_.block(6,0,12,1)+toeJaco_.transpose()*QP_force_-H_);
    dq_ = jointVel_+ddq_/LL_Hz;
    q_  = jointPosE_+dq_/LL_Hz+0.5/(LL_Hz*LL_Hz)*ddq_;

};

void FastMPC::LL_Cost(Eigen::MatrixXd &P_QP, Eigen::MatrixXd &c_QP, size_t outDim, size_t conDim, size_t numDec, size_t useCLF){
    double dfPen  = params->dfPen;
    double tauPen = params->tauPen;
    double auxPen = params->auxPen;
    double clfPen = params->clfPen;

    // Eigen::MatrixXd Fdes = Eigen::MatrixXd::Zero(12,1);
    // Eigen::Matrix<double, 6, 1> acc = Eigen::Matrix<double, 6, 1>::Zero();
    // updateDesiredForce(acc,Fdes);

    // Eigen::MatrixXd Fd = Eigen::MatrixXd::Zero(3*numContact_,1);
    // size_t cnt = 0;
    // for(size_t i=0; i<4; i++){
    //     if(contactIndex_(i)==1){
    //         Fd.block(3*cnt,0,3,1) = Fdes.block(3*i,0,3,1);
    //         cnt++;
    //     }
    // }

    // ====================================================================== //
    // ============================ Cost Function =========================== //
    // ====================================================================== //
    P_QP.block(0,0,conDim,conDim) = dfPen*Eigen::MatrixXd::Identity(conDim, conDim);
    P_QP.block(conDim,conDim,TOTAL_IN,TOTAL_IN) = tauPen*Eigen::MatrixXd::Identity(TOTAL_IN,TOTAL_IN);
    P_QP.block(conDim+TOTAL_IN,conDim+TOTAL_IN,outDim,outDim) = auxPen*Eigen::MatrixXd::Identity(outDim,outDim);
    if (useCLF){
        P_QP(numDec-1, numDec-1) = clfPen;
    }

    c_QP.setZero();
    // c_QP.block(0,0,conDim,1) = -Fd*dfPen;
};

void FastMPC::LL_Constraints(Eigen::MatrixXd &A_QP, Eigen::MatrixXd &b_QP, Eigen::MatrixXd &G_QP, Eigen::MatrixXd &h_QP, size_t outDim, size_t conDim, size_t numDec, size_t useCLF){
    // ====================================================================== //
    // ======================== Equality Constraints ======================== //
    // ====================================================================== //
    A_QP.block(0,0,conDim+outDim,numDec-useCLF) <<  
            Jc_*Dinv_*Jc_.transpose(), Jc_*Dinv_*B_, Eigen::MatrixXd::Zero(conDim,outDim),
            H0_*Dinv_*Jc_.transpose(), H0_*Dinv_*B_, Eigen::MatrixXd::Identity(outDim,outDim);
    b_QP.block(0,0,conDim+outDim,1) << Jc_*Dinv_*H_ - dJc_,
                                     (-kpGain_*y_-kdGain_*dy_) + H0_*Dinv_*H_ - dH0_;

    // ====================================================================== //
    // ======================= Inequality Constraints ======================= //
    // ====================================================================== //
    // Friction Cone
    Eigen::MatrixXd gc(5,3);
    Eigen::MatrixXd Gc = Eigen::MatrixXd::Zero(5*numContact_,conDim);
    gc <<  1,  0, -mu_/sqrt(2),
          -1,  0, -mu_/sqrt(2),
           0,  1, -mu_/sqrt(2),
           0, -1, -mu_/sqrt(2),
           0,  0,           -1;
    repdiag(gc,Gc,numContact_);
    G_QP.block(0,0,5*numContact_,conDim) = Gc;

    // Bounds
    G_QP.block(5*numContact_,conDim,TOTAL_IN+outDim,TOTAL_IN+outDim) <<  Eigen::MatrixXd::Identity(TOTAL_IN+outDim,TOTAL_IN+outDim);
    G_QP.block(5*numContact_+TOTAL_IN+outDim,conDim,TOTAL_IN+outDim,TOTAL_IN+outDim) << -Eigen::MatrixXd::Identity(TOTAL_IN+outDim,TOTAL_IN+outDim);
    
    h_QP.block(5*numContact_,0,TOTAL_IN+outDim,1) << sat_,sat_,sat_,sat_,params->auxMax*Eigen::MatrixXd::Ones(outDim,1);
    h_QP.block(5*numContact_+TOTAL_IN+outDim,0,TOTAL_IN+outDim,1) << sat_,sat_,sat_,sat_,params->auxMax*Eigen::MatrixXd::Ones(outDim,1);

    if(useCLF == 1){
        Eigen::MatrixXd PP = Eigen::MatrixXd::Zero(2*outDim,2*outDim);
        Eigen::MatrixXd P_temp(2*outDim,2*outDim);
        Eigen::MatrixXd tuneMat(2*outDim,2*outDim);
        double P1, Pd, P2;
        double cc;
        double eps;
        Eigen::MatrixXd FF(2*outDim, 2*outDim);
        Eigen::MatrixXd GG(2*outDim, outDim);
        Eigen::MatrixXd eta(2*outDim,1);
        Eigen::Matrix<double, 1, 1> V;
        Eigen::Matrix<double, 1, 1> LfV;
        Eigen::MatrixXd LgV;


        // Solve Algebraic Lyapunov Equation (MATLAB Check: lyap(FF',I) )
        // NOTE THE TRANSPOSE IN THE MATLAB CHECK!!!
        // This is a "special" solution given the particular of the matrices form used in this code
        P1 = (kdGain_*kdGain_+kpGain_*kpGain_+kpGain_)/(2*kpGain_*kdGain_);
        Pd = 1/(2*kpGain_);
        P2 = (kpGain_+1)/(2*kdGain_*kpGain_);
        PP.block(0,0,outDim,outDim) = P1*Eigen::MatrixXd::Identity(outDim,outDim);
        PP.block(0,outDim,outDim,outDim) = Pd*Eigen::MatrixXd::Identity(outDim,outDim);
        PP.block(outDim,0,outDim,outDim) = Pd*Eigen::MatrixXd::Identity(outDim,outDim);
        PP.block(outDim,outDim,outDim,outDim) = P2*Eigen::MatrixXd::Identity(outDim,outDim);
        cc = 1.0/( 0.5*( P1 + P2 + sqrt(P1*P1-2*P1*P2+P2*P2+4*Pd*Pd) ) );

        eps = params->clfEps;

        tuneMat.setIdentity();
        tuneMat.block(0,0,outDim,outDim) *= (1.0/eps);
        P_temp = tuneMat*PP*tuneMat;
        PP = P_temp;

        FF.setZero();
        GG.setZero();

        FF.block(0,outDim,outDim,outDim) = Eigen::MatrixXd::Identity(outDim,outDim);
        FF.block(outDim,0,outDim,outDim) = -kpGain_*Eigen::MatrixXd::Identity(outDim,outDim);
        FF.block(outDim,outDim,outDim,outDim) = -kdGain_*Eigen::MatrixXd::Identity(outDim,outDim);

        GG.block(outDim,0,outDim,outDim) = Eigen::MatrixXd::Identity(outDim,outDim);
        eta << y_, dy_;

        V   = (eta.transpose()*PP*eta);
        LfV = eta.transpose()*(FF.transpose()*PP+PP*FF)*eta;
        LgV = 2*eta.transpose()*PP*GG;

        G_QP.block(2*TOTAL_IN+2*outDim+5*numContact_, conDim+TOTAL_IN, 1, outDim) = LgV;
        G_QP(2*TOTAL_IN+5*numContact_+2*outDim, numDec-1) = -1.0; // defect variable
        h_QP(2*TOTAL_IN+5*numContact_+2*outDim, 0) = -LfV(0)-cc/eps*V(0);

        V_ = V(0);
        Veps_ = cc/eps*V(0);
        LfV_ = LfV(0);
        LgV_ = LgV;
    }
};

void FastMPC::torqueSat(){
    Eigen::Matrix<double, TOTAL_DOF,1> tau = jointTorque_FF_;
    // Joint Torque Saturation
    for(size_t i=0; i<4; i++){
        tau(6+3*i)   =   tau(6+3*i)>sat_(0) ? sat_(0) : ( (  tau(6+3*i)<-1*sat_(0)) ? -1*sat_(0) :   tau(6+3*i) );
        tau(6+3*i+1) = tau(6+3*i+1)>sat_(1) ? sat_(1) : ( (tau(6+3*i+1)<-1*sat_(1)) ? -1*sat_(1) : tau(6+3*i+1) );
        tau(6+3*i+2) = tau(6+3*i+2)>sat_(2) ? sat_(2) : ( (tau(6+3*i+2)<-1*sat_(2)) ? -1*sat_(2) : tau(6+3*i+2) );
    }
    jointTorque_FF_ = tau;
};

void FastMPC::dataLog(size_t ctrlTick){
    if (FILE_RW){
        double startTime = standStart_/1000;
        if(ctrlTick>=startTime*ctrlHz){
            size_t n = y_.rows();
            Eigen::Matrix<double,12,1> new_y = Eigen::MatrixXd::Zero(12,1);
            Eigen::Matrix<double,12,1> new_dy = Eigen::MatrixXd::Zero(12,1);
            Eigen::Matrix<double,12,1> new_hd = Eigen::MatrixXd::Zero(12,1);
            Eigen::Matrix<double,12,1> new_dhd = Eigen::MatrixXd::Zero(12,1);
            Eigen::Matrix<double,12,1> new_ddhd = Eigen::MatrixXd::Zero(12,1);
            new_y.block(0,0,n,1) = y_;
            new_dy.block(0,0,n,1) = dy_;
            new_hd.block(0,0,n,1) = hd_;
            new_dhd.block(0,0,n,1) = dhd_;
            new_ddhd.block(0,0,n,1) = ddhd_;

            file[0] << (ctrlTick-startTime*ctrlHz)*0.001 << ","
                    << new_y(0) << "," << new_y(1) << "," << new_y(2) << ","
                    << new_y(3) << "," << new_y(4) << "," << new_y(5) << ","
                    << new_y(6) << "," << new_y(7) << "," << new_y(8) << ","
                    << new_y(9) << "," << new_y(10) << "," << new_y(11) << ","
                    << jointTorque_FF_(6) << "," << jointTorque_FF_(7) << "," << jointTorque_FF_(8) << ","
                    << jointTorque_FF_(9) << "," << jointTorque_FF_(10) << "," << jointTorque_FF_(11) << ","
                    << jointTorque_FF_(12) << "," << jointTorque_FF_(13) << "," << jointTorque_FF_(14) << ","
                    << jointTorque_FF_(15) << "," << jointTorque_FF_(16) << "," << jointTorque_FF_(17) << ","
                    << jointPosE_(0) << "," << jointPosE_(1) << "," << jointPosE_(2) << ","
                    << jointPosE_(3) << "," << jointPosE_(4) << "," << jointPosE_(5) << ","
                    << jointPosE_(6) << "," << jointPosE_(7) << "," << jointPosE_(8) << ","
                    << jointPosE_(9) << "," << jointPosE_(10) << "," << jointPosE_(11) << ","
                    << jointPosE_(12) << "," << jointPosE_(13) << "," << jointPosE_(14) << ","
                    << jointPosE_(15) << "," << jointPosE_(16) << "," << jointPosE_(17) << ","
                    << jointVel_(0) << "," << jointVel_(1) << "," << jointVel_(2) << ","
                    << jointVel_(3) << "," << jointVel_(4) << "," << jointVel_(5) << ","
                    << jointVel_(6) << "," << jointVel_(7) << "," << jointVel_(8) << ","
                    << jointVel_(9) << "," << jointVel_(10) << "," << jointVel_(11) << ","
                    << jointVel_(12) << "," << jointVel_(13) << "," << jointVel_(14) << ","
                    << jointVel_(15) << "," << jointVel_(16) << "," << jointVel_(17) << ","
                    << new_hd(0) << "," << new_hd(1) << "," << new_hd(2) << ","
                    << new_hd(3) << "," << new_hd(4) << "," << new_hd(5) << ","
                    << new_hd(6) << "," << new_hd(7) << "," << new_hd(8) << ","
                    << new_hd(9) << "," << new_hd(10) << "," << new_hd(11) << ","
                    << new_dhd(0) << "," << new_dhd(1) << "," << new_dhd(2) << ","
                    << new_dhd(3) << "," << new_dhd(4) << "," << new_dhd(5) << ","
                    << new_dhd(6) << "," << new_dhd(7) << "," << new_dhd(8) << ","
                    << new_dhd(9) << "," << new_dhd(10) << "," << new_dhd(11) << ","
                    << new_ddhd(0) << "," << new_ddhd(1) << "," << new_ddhd(2) << ","
                    << new_ddhd(3) << "," << new_ddhd(4) << "," << new_ddhd(5) << ","
                    << new_ddhd(6) << "," << new_ddhd(7) << "," << new_ddhd(8) << ","
                    << new_ddhd(9) << "," << new_ddhd(10) << "," << new_ddhd(11) << ","
                    << (0)<< "," << (1) << "," << (2) << ","
                    << (3)<< "," << (4) << "," << (5) << ","
                    << (6)<< "," << (7) << "," << (8) << ","
                    << (9)<< "," << (10) << "," << (11) << ","
                    << (0)<< "," << (1) << "," << (2) << ","
                    << (3)<< "," << (4) << "," << (5) << ","
                    << (6)<< "," << (7) << "," << (8) << ","
                    << (9)<< "," << (10) << "," << (11) << ","
                    << new_dy(0) << "," << new_dy(1) << "," << new_dy(2) << ","
                    << new_dy(3) << "," << new_dy(4) << "," << new_dy(5) << ","
                    << new_dy(6) << "," << new_dy(7) << "," << new_dy(8) << ","
                    << new_dy(9) << "," << new_dy(10) << "," << new_dy(11) << "," 
                    << V_ << "," << dV_ << "," << stepLength_(0) << std::endl;
        }
    }
};

void FastMPC::compute(const double q[18], const double  dq[18], const double R[9], size_t gait, size_t ctrlTick,double speed[3],int ctrlCon[4]){
//	params->fwdSpeed = speed[0];
//	params->latSpeed = speed[1];

    static size_t gaitTemp = STAND;
    if(gait!=gaitTemp){
        impact_ = true;
    }

    if (gait!=STAND){
        numContact_ = 0;
        for(int i=0; i<4; ++i){
            contactIndex_(i) = ctrlCon[i];
            numContact_ += ctrlCon[i];
        }
        updateSwingState();
    }

    updateRobotState(q,dq,R);
    impactDetection(gait, ctrlTick, ctrlCon);
    newDom_ = impact_;
    planTraj(gait, ctrlTick);
    updateSwingState();
    computeVirtualConstraints(gait,ctrlTick);
    computeTorque();
    torqueSat();
    dataLog(ctrlTick);
    
    jointPosE_prev_ = jointPosE_;
    jointVel_prev_ = jointVel_;
    tau_prev_ = jointTorque_FF_.block(6,0,12,1);
    locoTick_ += (ctrlHz)/LL_Hz;
    gaitTemp = gait;
};

















