//
// Authror: Randy Fawcett on 12/2021.
//
// Copyright (c) Hybrid Dynamic Systems and Robot Locomotion Lab, Virginia Tech
//

#include "VirtualConstraints.hpp"

using VirtCon = VirtualConstraints;

VirtCon::VirtualConstraints(){
    VC.fDes.setZero();
}

void VirtCon::updateVirtualConstraints(const StateInfo *state, const KinInf *kin, const TrajInfo *traj, const ConInf *con, size_t gait, double phaseVar, MP *params, const LLInfo *ll){
    size_t outDim = 6+3*(4-con->cnt);
    size_t conDim = 3*con->cnt;

    // Only initialize size if necessary
    if (outDim!=h0.rows()){
        h0.setZero(outDim,1);
        dh0.setZero(outDim,1);
        VC.H0.setZero(outDim,TOTAL_DOF);
        VC.dH0.setZero(outDim,1);
        VC.hd.setZero(outDim,1);
        VC.dhd.setZero(outDim,1);
        VC.ddhd.setZero(outDim,1);
        VC.y.setZero(outDim,1);
        VC.dy.setZero(outDim,1);
        VC.y_ST.setZero(conDim,1);
        VC.dy_ST.setZero(conDim,1);
        VC.hd_ST.setZero(conDim,1);
        VC.dhd_ST.setZero(conDim,1);
        VC.H0.block(0,0,6,6) = Eigen::MatrixXd::Identity(6,6);
    }

    h0 = VC.H0*state->q;

    if (gait==STAND){
        VC.hd.block(0,0,3,1)   << traj->comDes.block(0,0,3,1);
        VC.dhd.block(0,0,3,1)  << traj->comDes.block(3,0,3,1);
        VC.ddhd.block(0,0,3,1) << 0, 0, 0;
        VC.hd.block(3,0,3,1)   << 0, 0, 0;
        VC.dhd.block(3,0,3,1)  << 0, 0, 0;
        VC.ddhd.block(3,0,3,1) << 0, 0, 0;
    }
    else {
        //std::cout << "comTraj_: " << traj->comDes(0) << "   " << traj->comDes(1) << "   Actual: " << state->q[0] << "    " << state->q[1] << std::endl; //std::cin.get(); 
        VC.hd.block(0,0,3,1) = traj->comDes.block(0,0,3,1);
        VC.hd.block(3,0,3,1) = traj->comDes.block(6,0,3,1);
        VC.dhd.block(0,0,3,1) = traj->comDes.block(3,0,3,1);
        VC.dhd.block(3,0,3,1) = traj->comDes.block(9,0,3,1);
        VC.ddhd.block(0,0,3,1) << 0,0,0;
        VC.ddhd.block(3,0,3,1) << 0,0,0;

        Eigen::VectorXd hipAcc(3);
        Eigen::VectorXd hipVel(3);

        size_t cnts = 0;
        size_t cntc = 0;
        double to = traj->toeOffset[2];
        double ds = (1.0*ctrlHz)/traj->domLen;
//        double phase = (phaseVar>1.0) ? 1.0 : phaseVar;
		double phase = phaseVar;    
        double dt = traj->domLen/(1.0*ctrlHz);
        for(size_t i=0; i<4; i++){
            if(con->ind[i]==0){
                h0.block(6+cnts,0,3,1) = kin->toePos.block(0,i,3,1);
                VC.H0.block(6+cnts,0,3,TOTAL_DOF) = kin->Jtoe.block(3*i,0,3,TOTAL_DOF);
                VC.dH0.block(6+cnts,0,3,1) = kin->dJtoe.block(3*i,0,3,1);
                

                // Swing leg to follow time varying bezier
                hipVel = kin->Jhip.block(3*i,0,3,18)*state->dq;
                hipAcc = ( kin->Jhip.block(3*i,0,3,18)*ll->ddq + kin->dJhip.block(3*i,0,3,1));
                // hipAcc.setZero();
                
                double tune=0;
                if (gait==PACE){
                    tune = (2*(i%2==0)-1)*0.04; // eventually this needs to be rotated by R (body to world)
                }

                int n = 4;
                double tx[3] = {0};
                double ax[4]{traj->toeInit(0,i), traj->toeInit(0,i), 
                            kin->hipPos(0,i)+traj->stepLen[0], kin->hipPos(0,i)+traj->stepLen[0]};          // NEED TO CHANGE , ToeFinal, toeInit 
                double dax[4] {0, 0, hipVel(0), hipVel(0)};
                double ddax[4] {0, 0, hipAcc(0), hipAcc(0)};
                calcVaryingBezierAll(n,dt,ax,dax,ddax,phase,tx);
                
                double ty[3] = {0};
                double ay[4] = {traj->toeInit(1,i), traj->toeInit(1,i), 
                                kin->hipPos(1,i)+traj->stepLen[1]+tune, kin->hipPos(1,i)+traj->stepLen[1]+tune};
                double day[4] = {0, 0, hipVel(1), hipVel(1)};
                double dday[4] = {0, 0, hipAcc(1), hipAcc(1)};
                calcVaryingBezierAll(n,dt,ay,day,dday,phase,ty);

                n = 8;
                double tz[3] = {0};
                double az[8]= {traj->toeInit(2,i), traj->toeInit(2,i), params->swingHeight, params->swingHeight, 
                               params->swingHeight, to+0.005, to+0.005, to};
                calcBezierAll(n, az, phaseVar, tz);

                // Save foot traj
                VC.hd.block(6+cnts,0,3,1)   << tx[0], ty[0], tz[0];
                VC.dhd.block(6+cnts,0,3,1)  << tx[1], ty[1], tz[1]*ds;
                VC.ddhd.block(6+cnts,0,3,1) << tx[2], ty[2], tz[2]*ds*ds; // z scaling necessary!!

                cnts+=3;
            }else {
                VC.hd_ST.block(cntc,0,3,1) = traj->toeFinal.block(0,i,3,1);
                cntc+=3;
            }
        }
    }
    dh0 = VC.H0*state->dq;

    VC.y = h0-VC.hd; 
    VC.y(0) = VC.y(0)*1; 
    VC.y(1) = VC.y(1)*1;
    
    VC.dy = dh0-VC.dhd; 
    VC.dy(0) = VC.dy(0)*1;
    VC.dy(1) = VC.dy(1)*1;
}



