#ifndef EIGENUTIL_FUNCS
#define EIGENUTIL_FUNCS

#include "math.h"
#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/Sparse"

template <typename inputdiag, typename outputdiag>
inline void repdiag(const Eigen::DenseBase<inputdiag> &A, Eigen::DenseBase<outputdiag> &diagMat, const int rep){
	int A_rows = A.rows();
	int A_cols = A.cols();
	diagMat.setZero();
	for(int i=0; i<rep; i++){
		diagMat.block(i*A_rows,i*A_cols,A_rows,A_cols)=A;
	}
}

template <typename inputmat, typename outputmat>
inline void repmat(const Eigen::DenseBase<inputmat> &A, Eigen::DenseBase<outputmat> &B, const int repx, const int repy){
	B.setZero();
	int r = A.rows();
	int c = A.cols();
	B.block(0,0,r,c) = A;
	for (int x=1; x<repx; ++x){
		B.block(r*x,0,r,c) = A; 
	}
	for (int y=1; y<repy; ++y){
		B.block(0,c*y,repx*r,c) = B.block(0,0,repx*r,c);
	}
}

inline void hatmap(const Eigen::Vector3d &w, Eigen::Matrix3d &skew){
	skew.setZero();
	skew(1,0) = w(2);
	skew(0,1) = -w(2);
	skew(2,0) = -w(1);
	skew(0,2) = w(1);
	skew(1,2) = -w(0);
	skew(2,1) = w(0);
}

inline void veemap(const Eigen::Matrix3d &skew, Eigen::Vector3d &w){
	w(0) = skew(2,1);
	w(1) = skew(0,2);
	w(2) = skew(1,0);
}

inline void skewmat(const Eigen::VectorXd &X, Eigen::MatrixXd &skewmatrix){
	skewmatrix.setZero();
	Eigen::Matrix3d skewtemp;
	Eigen::Vector3d wtemp;
	int c = X.cols();
	for(int i=0;i<c;i++){
		wtemp.segment(0,3) = X.block(0,i,3,1);
		hatmap(wtemp,skewtemp);
		skewmatrix.block(0,3*i,3,3) = skewtemp.block(0,0,3,3);
	}
}

template <typename q1, typename q2, typename output>
inline void quatMult(const Eigen::DenseBase<q1> &a, const Eigen::DenseBase<q2> &b, Eigen::DenseBase<output> &res){
	res(0) = a(0)*b(0) - a(1)*b(1) - a(2)*b(2) - a(3)*b(3);
	res(1) = a(1)*b(0) + a(0)*b(1) - a(3)*b(2) + a(2)*b(3);
	res(2) = a(2)*b(0) + a(3)*b(1) + a(0)*b(2) - a(1)*b(3);
	res(3) = a(3)*b(0) - a(2)*b(1) + a(1)*b(2) + a(0)*b(3);
}

template <typename T>
inline void spy(const Eigen::DenseBase<T> &X){
	// This function is used to display the structure of a matrix. A "1" is
	// placed everywhere that the matrix is non-zero. The final result is
	// displayed to the command window.
    const int r = X.rows();
    const int c = X.cols();
    T Y;
    Y.setZero(r,c);

    for(size_t i=0; i<r; i++){
        for(size_t j=0; j<c; j++){
            Y(i,j) = (X(i,j)!=0) ? 1 : 0;
        }
    }
    std::cout<<"Spying:"<<std::endl;
    std::cout<<Y<<std::endl;
}

#endif