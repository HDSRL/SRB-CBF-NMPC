#include "RobotModel.hpp"

RobotModel::RobotModel(){
    // Initialize some of the variables
    state.q.setZero();
    state.dq.setZero();
    comHist.setZero();
    state.comFiltered.setZero();

    //Make the B matrix (constant matrix)
    dyn.B.setZero();
    dyn.B.block(6,0,TOTAL_IN,TOTAL_IN) = Eigen::Matrix<double, TOTAL_IN, TOTAL_IN>::Identity();
}

void RobotModel::updateState(const double q_in[18], const double dq_in[18], const double R[9]){
    // Update previous before changing current
    state.q_prev = state.q;
    state.dq_prev = state.dq;
    
    // Save state to eigen vectors
    for(int i=0; i<18; i++){
        state.q(i) = q_in[i];
        state.dq(i) = dq_in[i];
    }
    for(int i=0; i<9; i++){
        state.R(i) = R[i];
    }

    // Transform angular velocities
    state.dq.block(3,0,3,1) = toWorld(state.dq.block(3,0,3,1),state.R);

    // Get body frame pos and vel
    state.bodyPos.block(0,0,3,1) = toBody(state.dq.block(0,0,3,1),state.R);
    state.bodyPos.block(3,0,3,1) = toBody(state.dq.block(3,0,3,1),state.R);
    state.bodyVel.block(0,0,3,1) = toBody(state.dq.block(0,0,3,1),state.R);
    state.bodyVel.block(3,0,3,1) = toBody(state.dq.block(3,0,3,1),state.R);

    // 
    // Eigen::Matrix<double, 3, HIST_LEN-1> comTemp;
    // comTemp = comHist.block(0,0,3,HIST_LEN-1);
    // comHist.block(0,0,3,1) = state.dq.block(0,0,3,1);
    // comHist.block(0,1,3,HIST_LEN-1) = comTemp;

    comHist.block(0,histInd,3,1) = state.dq.block(0,0,3,1);
    histInd = (++histInd) % HIST_LEN;
    state.comFiltered = comHist.rowwise().mean();

    updateDynamics();
    updateJacobian();
    updateJacobianDot();
    updateFwdKinematics();
}

void RobotModel::updateDynamics(){
    // Update the D matrix and its inverse
    D_mat(dyn.D.data(),state.q.data());
    dyn.Dinv = dyn.D.inverse();

    // Update the nonlinear vector (currently neglects coriolis)
    G_vec(dyn.H.data(),state.q.data());
}

void RobotModel::updateJacobian(){
    Eigen::Matrix<double, 3, TOTAL_DOF> J1,J2,J3,J4;

    // Toe jacobians
    J_FR_toe(J1.data(), state.q.data());
    J_FL_toe(J2.data(), state.q.data());
    J_RR_toe(J3.data(), state.q.data());
    J_RL_toe(J4.data(), state.q.data());
    kin.Jtoe.block<3,TOTAL_DOF>(0,0) = J1;
    kin.Jtoe.block<3,TOTAL_DOF>(3,0) = J2;
    kin.Jtoe.block<3,TOTAL_DOF>(6,0) = J3;
    kin.Jtoe.block<3,TOTAL_DOF>(9,0) = J4;

    // Hip jacobians
    J_FR_hip(J1.data(), state.q.data());
    J_FL_hip(J2.data(), state.q.data());
    J_RR_hip(J3.data(), state.q.data());
    J_RL_hip(J4.data(), state.q.data());
    kin.Jhip.block<3,TOTAL_DOF>(0,0) = J1;
    kin.Jhip.block<3,TOTAL_DOF>(3,0) = J2;
    kin.Jhip.block<3,TOTAL_DOF>(6,0) = J3;
    kin.Jhip.block<3,TOTAL_DOF>(9,0) = J4;
}

void RobotModel::updateJacobianDot(){
    Eigen::Matrix<double,3,1>dJ1,dJ2,dJ3,dJ4;

    // Toe jaco dot
    dJ_FR_toe(dJ1.data(), state.q.data(), state.dq.data());
    dJ_FL_toe(dJ2.data(), state.q.data(), state.dq.data());
    dJ_RR_toe(dJ3.data(), state.q.data(), state.dq.data());
    dJ_RL_toe(dJ4.data(), state.q.data(), state.dq.data());
    kin.dJtoe.block<3,1>(0,0) = dJ1;
    kin.dJtoe.block<3,1>(3,0) = dJ2;
    kin.dJtoe.block<3,1>(6,0) = dJ3;
    kin.dJtoe.block<3,1>(9,0) = dJ4;

    // Hip jaco dot
    dJ_FR_hip(dJ1.data(), state.q.data(), state.dq.data());
    dJ_FL_hip(dJ2.data(), state.q.data(), state.dq.data());
    dJ_RR_hip(dJ3.data(), state.q.data(), state.dq.data());
    dJ_RL_hip(dJ4.data(), state.q.data(), state.dq.data());
    kin.dJhip.block<3,1>(0,0) = dJ1;
    kin.dJhip.block<3,1>(3,0) = dJ2;
    kin.dJhip.block<3,1>(6,0) = dJ3;
    kin.dJhip.block<3,1>(9,0) = dJ4;
}

void RobotModel::updateFwdKinematics(){
    Eigen::Matrix<double,3,1>p1,p2,p3,p4;

    // Toe forward kinematics
    FK_FR_toe(p1.data(), state.q.data());
    FK_FL_toe(p2.data(), state.q.data());
    FK_RR_toe(p3.data(), state.q.data());
    FK_RL_toe(p4.data(), state.q.data());
    kin.toePos.block<3,1>(0,FR_LEG) = p1;
    kin.toePos.block<3,1>(0,FL_LEG) = p2;
    kin.toePos.block<3,1>(0,RR_LEG) = p3;
    kin.toePos.block<3,1>(0,RL_LEG) = p4;

    // Hip forward kinematics
    FK_FR_hip(p1.data(), state.q.data());
    FK_FL_hip(p2.data(), state.q.data());
    FK_RR_hip(p3.data(), state.q.data());
    FK_RL_hip(p4.data(), state.q.data());
    kin.hipPos.block<3,1>(0,FR_LEG) = p1;
    kin.hipPos.block<3,1>(0,FL_LEG) = p2;
    kin.hipPos.block<3,1>(0,RR_LEG) = p3;
    kin.hipPos.block<3,1>(0,RL_LEG) = p4;
}

void RobotModel::updateSwingMatrices(const int conInd[4], const int &numCon){
    // Only change the size if the current size is not sufficient
    if ( (3*numCon) != kin.Jc.rows() ){
        kin.Js.setZero(3*(4-numCon),TOTAL_DOF);
        kin.dJs.setZero(3*(4-numCon),1);
        kin.Jc.setZero(3*(numCon),TOTAL_DOF);
        kin.dJc.setZero(3*(numCon),1);
    }

    size_t cnts = 0, cntc = 0;
    for (int i=0; i<4; ++i){
        if(conInd[i]==0){
            kin.Js.block(cnts,0,3,TOTAL_DOF) = kin.Jtoe.block(3*i,0,3,TOTAL_DOF);
            kin.dJs.block(cnts,0,3,1) = kin.dJtoe.block(3*i,0,3,1);
            cnts+=3;
        }else if(conInd[i]==1){
            kin.Jc.block(cntc,0,3,TOTAL_DOF) = kin.Jtoe.block(3*i,0,3,TOTAL_DOF);
            kin.dJc.block(cntc,0,3,1) = kin.dJtoe.block(3*i,0,3,1);
            cntc+=3;
        }
    }
}


