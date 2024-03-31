#include "kalman.hpp"

KF::KF(float dt_in){
    dt = dt_in;
    sax = 2000.0;
    say = 2000.0;
    saz = 2000.0;

    A_.setIdentity();
    B_.setZero();
    C_.setZero();
    Q_.setZero();
    R_.setIdentity();
    xk_.setZero();
    Pk_.setIdentity();
    Pk_*=1e4;
    
    xk_(2) = 0.09;
    
    Rot_.setIdentity();

    A_(0, 3) = dt;
    A_(1, 4) = dt;
    A_(2, 5) = dt;

    B_(0, 0) = dt * dt / 2.0;
    B_(3, 0) = dt;
    B_(1, 1) = dt * dt / 2.0;
    B_(4, 1) = dt;
    B_(2, 2) = dt * dt / 2.0;
    B_(5, 2) = dt;

    C_(0, 0) = 1.0; C_(1, 1) = 1.0;  C_(2, 2) = 1.0;  C_(0, 6) = -1.0;  C_(3, 9) = -1.0;  C_(6, 12) = -1.0;   C_(9, 15) = -1.0;
    C_(3, 0) = 1.0; C_(4, 1) = 1.0;  C_(5, 2) = 1.0;  C_(1, 7) = -1.0;  C_(4, 10) = -1.0;  C_(7, 13) = -1.0;  C_(10, 16) = -1.0;
    C_(6, 0) = 1.0; C_(7, 1) = 1.0;  C_(8, 2) = 1.0;  C_(2, 8) = -1.0;  C_(5, 11) = -1.0;  C_(8, 14) = -1.0;  C_(11, 17) = -1.0;
    C_(9, 0) = 1.0; C_(10, 1) = 1.0; C_(11, 2) = 1.0; C_(12, 8) = 1.0;  C_(13, 11) = 1.0;  C_(14, 14) =  1.0; C_(15, 17) = 1.0;
}

void KF::updateKalman(const int contactIndex[4], float IMU[3], const double eul[3], const double relVec[12]){
    // ============================================================== //
    // ====================== Rotate IMU Data ======================= // 
    // ============================================================== // 
    Eigen::Vector3d imu,orig_imu;
    orig_imu(0) = IMU[0];
    orig_imu(1) = IMU[1];
    orig_imu(2) = IMU[2];
	R_XYZ(eul[0],eul[1],eul[2],Rot_);
    imu = Rot_*orig_imu;
    imu(2) -= 2.0*9.81;
    
    // ============================================================== //
    // ====================== Update Q_ Matrix ====================== // 
    // ============================================================== // 
    double spdfl = contactIndex[0]==1 ? 1 : 1e6;
    double spdfr = contactIndex[1]==1 ? 1 : 1e6;
    double spdhl = contactIndex[2]==1 ? 1 : 1e6;
    double spdhr = contactIndex[3]==1 ? 1 : 1e6;
    
    double dtdt = dt*dt;
    double SAX = sax*sax*dtdt;
    double SAY = say*say*dtdt;
    double SAZ = saz*saz*dtdt;    
    double FL = spdfl*spdfl*dtdt;
    double FR = spdfr*spdfr*dtdt;
    double RL = spdhl*spdhl*dtdt;
    double RR = spdhr*spdhr*dtdt;
    
    Q_(3, 3) = SAX;
    Q_(6, 6) = FL;
    Q_(9, 9) = FR;
    Q_(12, 12) = RL;
    Q_(15, 15) = RR;

    Q_(4, 4) = SAY;
    Q_(7, 7) = FL;
    Q_(10, 10) = FR;
    Q_(13, 13) = RL;
    Q_(16, 16) = RR;

    Q_(5,5) = SAZ;
    Q_(8, 8) = FL;
    Q_(11, 11) = FR;
    Q_(14, 14) = RL;
    Q_(17, 17) = RR;

    // ============================================================== //
    // ===================== Measurement Vector ===================== // 
    // ============================================================== //
    double toeOffset = 0.0;
    //double yk[16] = {relVec[0], relVec[1], relVec[2],  relVec[3], 
    //      relVec[4], relVec[5], relVec[6],  relVec[7], 
    //      relVec[8], relVec[9], relVec[10], relVec[11], 
    //      toeOffset, toeOffset, toeOffset, toeOffset};
    
    // yk should be an Eigen variable when using the regular kalman
    Eigen::Matrix<double, 16, 1> yk;
    yk << relVec[0], relVec[1], relVec[2],  relVec[3], 
          relVec[4], relVec[5], relVec[6],  relVec[7], 
          relVec[8], relVec[9], relVec[10], relVec[11], 
          toeOffset, toeOffset, toeOffset, toeOffset;

    // ============================================================== //
    // ============== Sequential (inverse free) Kalman ============== // 
    // ============================================================== //
    // Eigen::Matrix<double, 18,  1> K;
    // Eigen::Matrix<double, 18, 18> I = Eigen::Matrix<double, 18, 18>::Identity();
    // Pk_ = A_*Pk_*A_.transpose()+Q_;
    // xk_ = A_*xk_+B_*imu;
    // for(int i=0; i<16; i++){
    //     K   = Pk_ * C_.row(i).transpose() / ( C_.row(i)*Pk_*C_.row(i).transpose() + R_(i,i) );
    //     xk_ = xk_ + K*( yk[i] - C_.row(i)*xk_ );
    //     Pk_ = ( I - K*C_.row(i) ) * Pk_;
    // } 

    // ============================================================== //
    // ======================= Regular Kalman ======================= // 
    // ============================================================== //
    Eigen::Matrix<double, 18, 16> K;
    Eigen::Matrix<double, 16, 16> S;
    Eigen::Matrix<double, 18,  1> x_temp;
    Pk_ = A_*Pk_*A_.transpose()+Q_;
    S   = C_*Pk_*C_.transpose()+R_;
    K   = Pk_*C_.transpose()*S.inverse();
    xk_ = A_*xk_+B_*imu;
    xk_ = xk_ + K*(yk - C_*xk_);
    Pk_ = (Eigen::Matrix<double, 18, 18>::Identity() - K*C_)*Pk_;
}

void KF::updateKalman(const int contactIndex[4], float IMU[3], const float quat[4], const double relVec[12]){
    // ============================================================== //
    // ====================== Rotate IMU Data ======================= // 
    // ============================================================== // 
    Eigen::Vector3d imu,orig_imu;
    orig_imu(0) = IMU[0];
    orig_imu(1) = IMU[1];
    orig_imu(2) = IMU[2];
    quat2rot(quat,Rot_);
    imu = Rot_*orig_imu;
    imu(2) -= 2.0*9.81;
    
    // ============================================================== //
    // ====================== Update Q_ Matrix ====================== // 
    // ============================================================== // 
    double spdfl = contactIndex[0]==1 ? 1 : 1e6;
    double spdfr = contactIndex[1]==1 ? 1 : 1e6;
    double spdhl = contactIndex[2]==1 ? 1 : 1e6;
    double spdhr = contactIndex[3]==1 ? 1 : 1e6;
    
    double dtdt = dt*dt;
    double SAX = sax*sax*dtdt;
    double SAY = say*say*dtdt;
    double SAZ = saz*saz*dtdt;    
    double FL = spdfl*spdfl*dtdt;
    double FR = spdfr*spdfr*dtdt;
    double RL = spdhl*spdhl*dtdt;
    double RR = spdhr*spdhr*dtdt;
    
    Q_(3, 3) = SAX;
    Q_(6, 6) = FL;
    Q_(9, 9) = FR;
    Q_(12, 12) = RL;
    Q_(15, 15) = RR;

    Q_(4, 4) = SAY;
    Q_(7, 7) = FL;
    Q_(10, 10) = FR;
    Q_(13, 13) = RL;
    Q_(16, 16) = RR;

    Q_(5,5) = SAZ;
    Q_(8, 8) = FL;
    Q_(11, 11) = FR;
    Q_(14, 14) = RL;
    Q_(17, 17) = RR;

    // ============================================================== //
    // ===================== Measurement Vector ===================== // 
    // ============================================================== //
    double toeOffset = 0.0;
    //double yk[16] = {relVec[0], relVec[1], relVec[2],  relVec[3], 
    //      relVec[4], relVec[5], relVec[6],  relVec[7], 
    //      relVec[8], relVec[9], relVec[10], relVec[11], 
    //      toeOffset, toeOffset, toeOffset, toeOffset};
    
    // yk should be an Eigen variable when using the regular kalman
    Eigen::Matrix<double, 16, 1> yk;
    yk << relVec[0], relVec[1], relVec[2],  relVec[3], 
          relVec[4], relVec[5], relVec[6],  relVec[7], 
          relVec[8], relVec[9], relVec[10], relVec[11], 
          toeOffset, toeOffset, toeOffset, toeOffset;

    // ============================================================== //
    // ============== Sequential (inverse free) Kalman ============== // 
    // ============================================================== //
    // Eigen::Matrix<double, 18,  1> K;
    // Eigen::Matrix<double, 18, 18> I = Eigen::Matrix<double, 18, 18>::Identity();
    // Pk_ = A_*Pk_*A_.transpose()+Q_;
    // xk_ = A_*xk_+B_*imu;
    // for(int i=0; i<16; i++){
    //     K   = Pk_ * C_.row(i).transpose() / ( C_.row(i)*Pk_*C_.row(i).transpose() + R_(i,i) );
    //     xk_ = xk_ + K*( yk[i] - C_.row(i)*xk_ );
    //     Pk_ = ( I - K*C_.row(i) ) * Pk_;
    // } 

    // ============================================================== //
    // ======================= Regular Kalman ======================= // 
    // ============================================================== //
    Eigen::Matrix<double, 18, 16> K;
    Eigen::Matrix<double, 16, 16> S;
    Eigen::Matrix<double, 18,  1> x_temp;
    Pk_ = A_*Pk_*A_.transpose()+Q_;
    S   = C_*Pk_*C_.transpose()+R_;
    K   = Pk_*C_.transpose()*S.inverse();
    xk_ = A_*xk_+B_*imu;
    xk_ = xk_ + K*(yk - C_*xk_);
    Pk_ = (Eigen::Matrix<double, 18, 18>::Identity() - K*C_)*Pk_;
}


void KF::quat2rot(const float q[4], Eigen::Matrix3d &R){
    R(0,0) = 2*(q[0]*q[0]+q[1]*q[1])-1;
    R(0,1) = 2*(q[1]*q[2]-q[0]*q[3]);
    R(0,2) = 2*(q[1]*q[3]+q[0]*q[2]);

    R(1,0) = 2*(q[1]*q[2]+q[0]*q[3]);
    R(1,1) = 2*(q[0]*q[0]+q[2]*q[2])-1;
    R(1,2) = 2*(q[2]*q[3]-q[0]*q[1]);

    R(2,0) = 2*(q[1]*q[3]-q[0]*q[2]);
    R(2,1) = 2*(q[2]*q[3]+q[0]*q[1]);
    R(2,2) = 2*(q[0]*q[0]+q[3]*q[3])-1;
}
