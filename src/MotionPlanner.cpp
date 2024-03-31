#include "MotionPlanner.hpp"

MotionPlanner::MotionPlanner(){
    x0 = 0;
    y0 = 0;
    z0 = 0.05;
    
    traj.domLen = 1*ctrlHz;
    standTime = 1*ctrlHz;

    traj.comDes.setZero();
    traj.redDes.setZero();
    traj.toeInit.setZero();
    traj.toeFinal.setZero();
    traj.toeOffset[2] = 0.01;
}

void MotionPlanner::printSize(const Eigen::MatrixXd& mat)
{
    std::cout << "\nRows(): " << mat.rows() << "   cols(): " << mat.cols() << std::endl;
    std::cout << mat << std::endl;
} 

void MotionPlanner::setComDes(const Eigen::Vector4d& comTraj) // comTraj -> x, y, dx, dy  
{
    traj.comDes(0) = comTraj(0);
    traj.comDes(1) = comTraj(1);
    traj.comDes(2) = 0.28; //z traj is defined in plan_traj for the standing, otherwise it is constant 0.26 and all the derivatives are zero

    // if (comTraj(3) > 0.1 || comTraj(4) > 0.1)
    // {
    //     traj.comDes(3) = 0.5*comTraj(3);
    //     traj.comDes(4) = 0.5*comTraj(4); 
    // }

    traj.comDes(3) = comTraj(2);    
    traj.comDes(4) = comTraj(3);
    // traj.comDes(0) = 0;
    // traj.comDes(1) = 0;
    // traj.comDes(2) = 0.26; //z traj is defined in plan_traj for the standing, otherwise it is constant 0.26 and all the derivatives are zero
    // traj.comDes(3) = 0;    
    // traj.comDes(4) = 0;
}

void MotionPlanner::planTraj(const StateInfo *state, const KinematicsInfo *kin, ContactEst *con_obj, size_t gait, double phase, size_t ctrlTick, MP * params, const Eigen::MatrixXd & Pr_refined_, size_t agent_id_, size_t gaitDomain_, Eigen::MatrixXd footPrintTruncated_){
    const ContactInfo* con = con_obj->getConInfoPointer();  
    Eigen::Matrix<double, 8, 1> newFoothold;    
    Eigen::Matrix<double, 4, 8> qref; int gap = 7;
    qref.setZero();
    qref.block(0,0,1,(gap+1)) = Pr_refined_.block(2*agent_id_,(gap+1)*(gaitDomain_),1,(gap+1));
    qref.block(2,0,1,(gap+1)) = Pr_refined_.block(2*agent_id_+1,(gap+1)*(gaitDomain_),1,(gap+1));
    newFoothold.setZero();
    newFoothold << qref(0,gap)+0.1830, qref(2,gap)+0.1320, qref(0,gap)-0.1830,qref(2,gap)+0.1320,qref(0,gap)+0.1830,qref(2,gap)-0.1320,qref(0,gap)-0.1830,qref(2,gap)-0.1320;

    static Eigen::Matrix<double, 3, 1> desVel = {0,0,0};                // <<<<<--------------------------------------------------------------------
    static Eigen::Matrix<double, 3, 1> desOmega = {0,0,0};
      
    static bool standTrigger = false;
    double normMax = 0.08;
    if(gait==STAND){
        double s = (phase>1) ? 1 : ((phase<0) ? 0 : phase);

        double xFinal = x0-0.04*0;
        double yFinal = y0-0.04*0;
        double zFinal = params->standHeight;
        double alpha_x[8] = { x0,x0,x0,
                 x0+(xFinal-x0)/4,
                 x0+3*(xFinal-x0)/4,
                 xFinal,xFinal,xFinal};
        double alpha_y[8] = {y0,y0,y0,
                 y0+(yFinal-y0)/4,
                 y0+3*(yFinal-y0)/4,
                 yFinal,yFinal,yFinal};
        double alpha_z[8] = {z0,z0,z0,
                 z0+(zFinal-z0)/4,
                 z0+3*(zFinal-z0)/4,
                 zFinal,zFinal,zFinal};

        double traj_x[3], traj_y[3], traj_z[3];
        calcBezierAll((int)8, alpha_x, s, traj_x);
        calcBezierAll((int)8, alpha_y, s, traj_y);
        calcBezierAll((int)8, alpha_z, s, traj_z);

        // traj.comDes -> pos, vel, theta, omega
        traj.comDes.block(0,0,3,1) << traj_x[0], traj_y[0], traj_z[0];
        // traj.comDes(2) = traj_z[0];
        traj.comDes.block(3,0,3,1) << traj_x[1], traj_y[1], traj_z[1];
        // traj.comDes(5) = traj_z[1];
        traj.comDes.block(6,0,3,1) << 0, 0, 0;
        traj.comDes.block(9,0,3,1) << 0, 0, 0;

        traj.domLen = standTime;
        con_obj->setDesDomain({1,1,1,1});

        traj.toeInit = kin->toePos; 
        traj.toeFinal = kin->toePos;
        traj.redDes = traj.comDes;
        
    }else if(gait==POSE) {
        traj.comDes.block(3,0,3,1) << 0,0,0;
        traj.comDes.block(9,0,3,1) << 0,0,0;
        double t = 1.0*ctrlTick/(1.0*ctrlHz);
        static double t_init = t;
        static Eigen::Matrix<double, 12, 1> lock = traj.comDes;
        traj.comDes = lock;
        if(poseType==POSE_X){
			double freq = 0.8*MY_PI;
            double mag = 0.04;
            traj.comDes(0) += mag*sin(freq*(t-t_init));
            traj.comDes(3) += mag*freq*cos(freq*(t-t_init));
        }
        else if(poseType==POSE_Y){
            double freq = 0.8*MY_PI;
            double mag = 0.04;
            traj.comDes(1) += mag*sin(freq*(t-t_init));
            traj.comDes(4) += mag*freq*cos(freq*(t-t_init));
        }
        else if(poseType==POSE_Z){
            double freq = 0.8*MY_PI;
            double mag = 0.05;
            traj.comDes(2) += mag*cos(freq*(t-t_init))-mag;
            traj.comDes(5) += -1.0*mag*freq*sin(freq*(t-t_init));
        }
        else if(poseType==POSE_ROLL){
            double freq = 0.8*MY_PI;
			double mag = 0.3491;
            traj.comDes(6) += mag*sin(freq*(t-t_init));
            traj.comDes(9) += mag*freq*cos(freq*(t-t_init));
        }
        else if(poseType==POSE_PITCH){
            double freq = 0.8*MY_PI;
            double mag = 0.17453;
            traj.comDes(7) += mag*sin(freq*(t-t_init));
            traj.comDes(10) += mag*freq*cos(freq*(t-t_init));
        }
        else if(poseType==POSE_YAW){
            double freq = 0.8*MY_PI;
            double mag = 0.13963;
            traj.comDes(8) += mag*sin(freq*(t-t_init));
            traj.comDes(11) += mag*freq*cos(freq*(t-t_init));
        }
        else if(poseType==POSE_COMB){
        	double freq = 0.6*MY_PI;
        	double mag = 0.3491;
        	traj.comDes(7) += mag*sin(freq*(t-t_init));
        	traj.comDes(10) += mag*freq*cos(freq*(t-t_init));
        	
            static int triggerStart = 0;
            if (triggerStart || cos(freq*(t-t_init))<0){
                traj.comDes(8) += mag*cos(freq*(t-t_init));
                traj.comDes(11) += -mag*freq*sin(freq*(t-t_init));
                triggerStart = 1;
            }
        }
    }else if(gait==TAP){
        setStepLen(0.0,0.0,0.0);
        if(con->changeDomain){ 
        	static double domLenSec = 1.5;
            con_obj->setDesDomain({1, 0, 1, 1});
            traj.domLen = domLenSec*ctrlHz;
            traj.toeInit = kin->toePos;
//            domLenSec -= (domLenSec>1.0) ? 0.25 : 0;
        }
    }else if(gait==INPLACE_WALK){
        if(con->changeDomain==1){
            traj.toeInit = kin->toePos;
            
            static int n = 0;
            Eigen::Matrix<int, 4, 4> doms;
            doms << 0,1,1,1,
                    1,0,1,1,
                    1,1,1,0,
                    1,1,0,1;
            n = (++n) % 4;
            con_obj->setDesDomain({doms(n,0),doms(n,1),doms(n,2),doms(n,3)});
            traj.domLen = 0.3*ctrlHz;
        }
    }else if(gait==INPLACE_TROT){
        if(con->changeDomain){
            if(con->des[0]==1){
                con_obj->setDesDomain({0, 1, 1, 0});}
            else{
                con_obj->setDesDomain({1, 0, 0, 1});}
            traj.domLen = 0.16*ctrlHz;
            traj.toeInit.block(0,0,2,4) = kin->hipPos.block(0,0,2,4);
            traj.toeInit.block(2,0,1,4) = kin->toePos.block(2,0,1,4);
        }
    }else if(gait==WALK){
        double dt = (1.0/LL_Hz);
        double domLenSec = 0.2;

        if(con->changeDomain==1){
            // desVel(0) += (desVel(0)<0.2) ? 0.05 : 0; 
            updateVel(desVel,desOmega,params);
            
            // ================================================ //
            // Update contact matrix and initial toe position(s)
            // ================================================ //
            static int n = 2;
            Eigen::Matrix<int, 4, 4> doms;
            doms << 0,1,1,1,
                    1,1,0,1,
                    1,1,1,0,
                    1,0,1,1;
            n = (++n) % 4;
            con_obj->setDesDomain({doms(n,0),doms(n,1),doms(n,2),doms(n,3)});
            traj.domLen = domLenSec*ctrlHz;
            traj.toeInit = kin->toePos;

            // ================================================ //
            // Marc Raibert foothold selection (similar)
            // ================================================ //
            Eigen::Matrix<double, 3, 1> stepLenTemp;
            std::vector<double> KP = {0.04,0.01,0.0};
            setStepLen(0.0,0.0,0.0);
            stepLenTemp = toBody(state->comFiltered,state->R)-desVel;
            stepLenTemp(0) *= KP[0]; stepLenTemp(1) *= KP[1]; stepLenTemp(2) *= KP[2];
            stepLenTemp += (domLenSec*toBody(desVel,state->R)/2);
            toWorld(traj.stepLen,stepLenTemp,state->R);
        }

        // Eigen::Matrix<double, 3, 1> desVelWorld = toWorld(desVel,state->R);

        // traj.comDes.block(0,0,3,1) = state->q.block(0,0,3,1) + desVelWorld*dt;
        // traj.comDes(2) = params->standHeight; // fixed standing height
        // traj.comDes.block(3,0,3,1) = desVel;
    }else if(gait==TROT){
        // std::cout << "DBG planTraj 2\n";
        double dt = (1.0/LL_Hz);
        double domLenSec = TSOPTTICK*0.001*4;

        // static int startTrot = 0;
        if(con->changeDomain==1){
            if(traj.comDes(3)==0 && traj.comDes(4)==0 && state->comFiltered.block(0,0,2,1).norm()<normMax){
                standTrigger = true;
            } else {
                standTrigger = false;
            }

            // ================================================ //
            // Update contact matrix and initial toe position(s)
            // ================================================ //
            if(con->des[0]==1){
                con_obj->setDesDomain({0, 1, 1, 0});}
            else{
                con_obj->setDesDomain({1, 0, 0, 1});}
            traj.domLen = domLenSec*ctrlHz;
            traj.toeInit = kin->toePos;

            // ================================================ //
            // Marc Raibert foothold selection (similar)
            // ================================================ //
            double stepTmp[3] = {0};
            float des_vel_x = ( footPrintTruncated_(1) + footPrintTruncated_(1+4) + footPrintTruncated_(1+2*4) + footPrintTruncated_(1+3*4) )/4;
            float des_vel_y = (footPrintTruncated_(3) + footPrintTruncated_(3+4) + footPrintTruncated_(3+2*4) + footPrintTruncated_(3+3*4))/4;
            // stepTmp[0] = (( footPrintTruncated_(1) + footPrintTruncated_(1+4) + footPrintTruncated_(1+2*4) + footPrintTruncated_(1+3*4) )/8 )*domLenSec;
            // stepTmp[1] = (( footPrintTruncated_(3) + footPrintTruncated_(3+4) + footPrintTruncated_(3+2*4) + footPrintTruncated_(3+3*4) )/8 )*domLenSec;
            float Kx = 1*0.1*sqrt(0.28/9.81); float des_vel = 0.1;
            float Ky = 1*0.05*sqrt(0.28/9.81); 
            // stepTmp[0] = ((state->dq[0]*domLenSec)/2);// + K*(state->dq[0]-des_vel);
            // stepTmp[1] = ((state->dq[1]*domLenSec)/2);// + K*(state->dq[1]-0);
            //stepTmp[0] = ((state->dq[0]*domLenSec)/2) + Kx*(state->dq[0]-des_vel);
            stepTmp[0] = ((des_vel_x*domLenSec)/2) + Kx*(state->dq[0]-des_vel_x);
            //stepTmp[1] = ((state->dq[1]*domLenSec)/2) + Ky*(state->dq[1]-0);
            stepTmp[1] = ((des_vel_y*domLenSec)/2) + Ky*(state->dq[1]-des_vel_y);
            
            setStepLen(stepTmp[0],stepTmp[1],0.0);
        }
    } else if ( gait==PACE ){
        double dt = (1.0/LL_Hz);
        double domLenSec = 0.1;

        if(con->changeDomain==1){
            updateVel(desVel, desOmega, params);
            
            // ================================================ //
            // Update contact matrix and initial toe position(s)
            // ================================================ //
            if(con->des[0]==1){
                con_obj->setDesDomain({0, 1, 0, 1});}
            else{
                con_obj->setDesDomain({1, 0, 1, 0});}
            traj.domLen = domLenSec*ctrlHz;
            traj.toeInit = kin->toePos;

            // ================================================ //
            // Marc Raibert foothold selection (similar)
            // ================================================ //
            Eigen::Matrix<double, 3, 1> stepLenTemp;
            std::vector<double> KP = {0.04,0.02,0.0};
            setStepLen(0.0,0.0,0.0);
            stepLenTemp = toBody(state->comFiltered,state->R)-desVel;
            stepLenTemp(0) *= KP[0]; stepLenTemp(1) *= KP[1]; stepLenTemp(2) *= KP[2];
            stepLenTemp += (domLenSec*toBody(desVel,state->R)/2);
            toWorld(traj.stepLen,stepLenTemp,state->R);
        }

        Eigen::Matrix<double, 3, 1> desVelWorld = toWorld(desVel,state->R);

        // traj.comDes.block(0,0,3,1) = state->q.block(0,0,3,1) + desVelWorld*dt;
        // traj.comDes(0) = (desVel(0)==0) ? traj.comDes(0)/3 : traj.comDes(0);
        // traj.comDes(1) = (desVel(1)==0) ? traj.comDes(1)/3 : traj.comDes(1);
        // traj.comDes(2) = params->standHeight; // fixed standing height
        // traj.comDes.block(3,0,3,1) = desVelWorld;
    }

    if(con->changeDomain==1){
        traj.toeFinal = kin->toePos;
    }else {
        for(int i=0; i<4; ++i){
            if(con->ind[i]!=con->ind_prev[i]){
                traj.toeFinal.block(0,i,3,1) = kin->toePos.block(0,i,3,1);
            }
        }
    }
}

void MotionPlanner::updateStandVars(const Eigen::Matrix<double,3,1> &com, double timeToStand){
    x0 = com(0);
    y0 = com(1);
    z0 = com(2);
    traj.domLen = timeToStand;
    standTime = timeToStand;

    // Why do you do this? - Basit 
    if(timeToStand<100){
        traj.domLen *=ctrlHz;
        standTime *= ctrlHz;
    }
};

void MotionPlanner::updateVel(Eigen::Matrix<double,3,1> &desVel, Eigen::Matrix<double, 3, 1> &desOmega, MP *params){
    float rate = 0.01;
    int fwd_sgn = (params->fwdSpeed>0) ? 1 : (params->fwdSpeed<0) ? -1 : 0;
    int lat_sgn = (params->latSpeed>0) ? 1 : (params->latSpeed<0) ? -1 : 0;
    desVel(0) += (fwd_sgn*desVel(0)<fwd_sgn*params->fwdSpeed) ? rate*fwd_sgn : 0;
    desVel(1) += (lat_sgn*desVel(1)<lat_sgn*params->latSpeed) ? rate*lat_sgn : 0;
};
