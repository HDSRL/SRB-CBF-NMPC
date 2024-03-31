#include "DataLog.hpp"


DataLog::DataLog(std::string filename){
    if(!filename.empty()){
        fid = std::fstream(filename, std::ios::out);
    }
    success = (fid.is_open()) ? 1 : 0;
};

DataLog::~DataLog(){
    if(fid.is_open()){
        fid.close();
    }
};

void DataLog::writeData(const StateInfo *state, const VCInfo *vc, const TrajInfo *traj, const LLInfo *LL, const size_t ctrlTick, const int force[4], double phase){
    if (success==1){
        size_t n = vc->y.rows();
        y_.setZero(); dy_.setZero();
        hd_.setZero(); dhd_.setZero(); ddhd_.setZero();
        y_.block(0,0,n,1) = vc->y;
        dy_.block(0,0,n,1) = vc->dy;
        hd_.block(0,0,n,1) = vc->hd;
        dhd_.block(0,0,n,1) = vc->dhd;
        ddhd_.block(0,0,n,1) = vc->ddhd;

        fid << ctrlTick/(1.0*ctrlHz) << "," 
            << y_(0) << "," << y_(1)  << "," << y_(2)  << ","
            << y_(3) << "," << y_(4)  << "," << y_(5)  << ","
            << y_(6) << "," << y_(7)  << "," << y_(8)  << ","
            << y_(9) << "," << y_(10) << "," << y_(11) << ","
            << LL->tau[6]  << "," << LL->tau[7]  << "," << LL->tau[8]  << "," 
            << LL->tau[9]  << "," << LL->tau[10] << "," << LL->tau[11] << "," 
            << LL->tau[12] << "," << LL->tau[13] << "," << LL->tau[14] << ","
            << LL->tau[15] << "," << LL->tau[16] << "," << LL->tau[17] << "," 
            << state->q(0)  << "," << state->q(1)  << "," << state->q(2)  << "," 
            << state->q(3)  << "," << state->q(4)  << "," << state->q(5)  << "," 
            << state->q(6)  << "," << state->q(7)  << "," << state->q(8)  << "," 
            << state->q(9)  << "," << state->q(10) << "," << state->q(11) << "," 
            << state->q(12) << "," << state->q(13) << "," << state->q(14) << "," 
            << state->q(15) << "," << state->q(16) << "," << state->q(17) << "," 
            << state->dq(0) << ","  << state->dq(1)  << "," << state->dq(2)  << "," 
            << state->dq(3) << ","  << state->dq(4)  << "," << state->dq(5)  << "," 
            << state->dq(6) << ","  << state->dq(7)  << "," << state->dq(8)  << "," 
            << state->dq(9) << ","  << state->dq(10) << "," << state->dq(11) << "," 
            << state->dq(12) << "," << state->dq(13) << "," << state->dq(14) << "," 
            << state->dq(15) << "," << state->dq(16) << "," << state->dq(17) << ","
            << hd_(0) << "," << hd_(1)  << "," << hd_(2)  << "," 
            << hd_(3) << "," << hd_(4)  << "," << hd_(5)  << "," 
            << hd_(6) << "," << hd_(7)  << "," << hd_(8)  << ","  
            << hd_(9) << "," << hd_(10) << "," << hd_(11) << "," 
            << dhd_(0) << "," << dhd_(1)  << "," << dhd_(2)  << "," 
            << dhd_(3) << "," << dhd_(4)  << "," << dhd_(5)  << "," 
            << dhd_(6) << "," << dhd_(7)  << "," << dhd_(8)  << ","  
            << dhd_(9) << "," << dhd_(10) << "," << dhd_(11) << "," 
            << ddhd_(0) << "," << ddhd_(1)  << "," << ddhd_(2)  << "," 
            << ddhd_(3) << "," << ddhd_(4)  << "," << ddhd_(5)  << "," 
            << ddhd_(6) << "," << ddhd_(7)  << "," << ddhd_(8)  << ","  
            << ddhd_(9) << "," << ddhd_(10) << "," << ddhd_(11) << "," 
            << (0)<< "," << (1)  << "," << (2)  << ","
            << (3)<< "," << (4)  << "," << (5)  << ","
            << (6)<< "," << (7)  << "," << (8)  << ","
            << (9)<< "," << (10) << "," << (11) << ","
            << (0)<< "," << (1)  << "," << (2)  << ","
            << (3)<< "," << (4)  << "," << (5)  << ","
            << (6)<< "," << (7)  << "," << (8)  << ","
            << (9)<< "," << (10) << "," << (11) << ","
            << dy_(0) << "," << dy_(1)  << "," << dy_(2)  << ","
            << dy_(3) << "," << dy_(4)  << "," << dy_(5)  << ","
            << dy_(6) << "," << dy_(7)  << "," << dy_(8)  << ","
            << dy_(9) << "," << dy_(10) << "," << dy_(11) << ","
            << LL->V << "," << LL->dV << "," 
            << force[0] << ","<< force[1] << ","<< force[2] << ","<< force[3] << ","
            << phase << "," << std::endl;
            
    }
};
