#ifndef DATALOG_HPP
#define DATALOG_HPP

#include "global_loco_structs.hpp"
#include "iostream"
#include "fstream"


class DataLog{
public:
    DataLog(std::string filename);
    virtual ~DataLog();
    void openFile();
    void writeData(const StateInfo *state, const VCInfo *vc, const TrajInfo *traj, const LLInfo *LL, const size_t ctrlTick, const int force[4], double phase);

private:
    std::fstream fid;
    Eigen::Matrix<double, 12, 1> y_, dy_, hd_, dhd_, ddhd_;

    int success = 0;

};



#endif
