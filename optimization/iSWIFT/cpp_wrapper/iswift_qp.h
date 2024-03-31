/* Author - Jeeseop Kim */

#ifndef ISWIFT_QP_H
#define ISWIFT_QP_H

extern "C" {
    #include "Prime.h"
}
#include "Prime.h"

#include "utils.h"

void iswiftQp_test();
void iswiftQp_e(const Eigen::MatrixXd P, Eigen::MatrixXd c_e, 
                const Eigen::MatrixXd A, Eigen::MatrixXd b_e, 
                const Eigen::MatrixXd G, Eigen::MatrixXd h_e, double* sol);
void iswiftQp(const Eigen::MatrixXd P, double* c, 
                const Eigen::MatrixXd A, double* b, 
                const Eigen::MatrixXd G, double* h, double* sol);
void ccstorage(const Eigen::MatrixXd PAG, int* PAGir, int* PAGjc, double* PAGpr);
void permutation(const Eigen::MatrixXd P, const int n, 
                const Eigen::MatrixXd A, const int p, 
                const Eigen::MatrixXd G, const int m, int* permut);

#endif //ISWIFT_QP_H