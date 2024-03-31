#ifndef __PRIME_H__
#define __PRIME_H__

#include "timer.h"

// Sparse Matrix Storage Format
// Use Sparse Matrix Setup to Initialise a Sparse Matrix
typedef struct smat{
	idxint* jc;				// Vector to store column count ; Dim [n+1]
	idxint* ir;				// Vector to store row indices in column major format ; Dim[nnz]
	realqp* pr;				// Vector to store matrix values in column major format ; Dim[nnz]
	idxint n;				// Number of Rows of the Sparse Matrix
	idxint m;				// Number of Columns of the Sparse Matrix
	idxint nnz;				// Number of nonzeros entries ; nnz = jc [n]
}smat;



typedef struct kkt{
	smat *kktmatrix;		// Sparse kkt matrix
	realqp *b;				// b vector
	idxint *Parent;			// LDL - workspace Vectors
	idxint *Flag;			// LDL - workspace Vectors
	idxint *Lnz;			// LDL - workspace Vectors
	idxint *Li;				// ir vector of LDL Sparse Matrix in column compressed format
	idxint *Lp;				// jc vector of LDL Sparse Matrix in column compressed format
	idxint *Lti;            // ir vector of the transpose of LDL Sparse Matrix in column compressed format
	idxint *Ltp;            // jc vector of the transpose of LDL Sparse Matrix in column compressed format
	idxint *Pattern;		// LDL - workspace Vectors
	idxint *UPattern;		// Nodes to be updated during every iteration
	realqp *Y;				// LDL - workspace Vectors
	realqp *Lx;				// pr vector of LDL Sparse Matrix in column compressed format
	realqp *D;				// LDL - workspace Vectors
	idxint *P;				// Permutation Vector ; Input
	idxint *Pinv;			// Permutation Vector Invers

}kkt;

typedef struct stats{
	realqp tsetup;				// Setup Time ; Includes Initialisation Problem as well
	realqp tsolve;				// Solve Time
	realqp kkt_time;				// kkt Solve Time
	realqp ldl_numeric;			// ldl_numeric time
	idxint IterationCount;		// Iteration Count
	idxint Flag;				// Solver FLAG

} stats;

typedef struct settings{

	idxint maxit;		// Maximum Number of Iterations
	realqp reltol;		// Residual Tolerances
	realqp abstol;		// s and z Tolerances
	realqp sigma;			// Sigma Desired

}settings;

typedef struct QP{


	idxint n;					// Dimension of P matrix
	idxint m;					// First Dimension of G matrix
	idxint p;					// First Dimension of A matrix

	realqp sigma_d;				// Parameter
	realqp mu;					// Barrier Function Parameter
	realqp rho;					// Some Parameter


	realqp *x;					//	Primal Variables ;  Dimensions [n,1]
	realqp *y;				    //  Dual   Variables ;  Dimensions [p,1]
	realqp *z;					//	Dual Variables	 ;	Dimensions [m,1]
	realqp *s;					//	Primal Variables ;	Dimensions [m,1] 


	realqp *rx;					//	Residuals	;	Dimensions [n,1]
	realqp *ry;					//  Residuals	;	Dimensions [p,1]
	realqp *rz;					//	Residuals	;	Dimensions [m,1]

	realqp *delta;				// [delta_x;delta_y;delta_z]	;	Dimensions [n + p + m,1]
	realqp *delta_x;				// delta_x	;	Dimensions [n,1]
	realqp *delta_y;				// delta_y	;	Dimensions [p,1]
	realqp *delta_z;				// delta_z	;	Dimensions [m,1]
	realqp *delta_s;				// delta_s	;	Dimensions [m,1]

	realqp *ds;					// ds	;	Dimensions [m,1]
	realqp *lambda;				// lambda	;	Dimensions[m,1]

	smat *P;					// Cost Function	:	Quadratic Part	:	Dimensions	[n,n]
	realqp *c;					// Cost Function	:	linear term	:	Dimensions	[n,1]
	smat *G;					// Inequality Matrix	:	Gx<=h	:	Dimension	[m,n]
	realqp *h;					// Inequality Matrix	:	Gx<=h	:	Dimension	[m,1]
	smat *A;					// Equality Matrix		:	Ax=b	:	Dimension	[p,n]
	realqp *b;					// Equality Matrix		:	Ax=b	:	Dimension	[p,1]


	kkt *kkt;		// kkt Matrix
	settings *options;	// Solver Settings
	stats *stats;	// Solver Stats

} QP;

// Main Solver Functions
// Written in Prime.c
// QP Setup Function
QP* QP_SETUP(idxint n, idxint m, idxint p, idxint *Pjc, idxint *Pir, realqp *Ppr, idxint*Ajc, idxint *Air, realqp *Apr, idxint *Gjc, idxint *Gir, realqp *Gpr, realqp *c, realqp *h, realqp *b, realqp sigma_d, idxint *Permut);

// QP Solve Function
idxint QP_SOLVE(QP *myQP);

// QP Memory Clean Function
void QP_CLEANUP(QP *myQP);



// Auxillary Functions
// Written in Auxilary.c

idxint kktsolve_1(QP* myQP);

void kktsolve_2(QP* myQP);

smat* SparseMatrixSetup(idxint m, idxint n, idxint nnz, idxint* jc, idxint* ir, realqp* pr);

void Transpose_Row_Count(idxint m, idxint n, idxint *Li, idxint *Lp, idxint *Lti, idxint *Ltp);

void computeresiduals(QP* myQP);

idxint kkt_initialize(QP* myQP);

void SparseMatrixMultiply(smat *A, realqp *x, realqp *y, idxint start);

void SparseMatrixTransMultiply(smat *A, realqp* x, realqp* y, idxint start);

realqp norm(realqp*x, idxint n);

void form_ds(realqp* ds, realqp *lambda, realqp *delta_s, realqp *delta_z, realqp sigma, realqp mu, idxint m, idxint selector);

smat* formkktmatrix_U(smat* P, smat* G);

smat* formkktmatrix_full(smat *P, smat *G, smat *A);

smat* SparseMatrixTranspose(smat*A);

void updatekktmatrix(smat *kkt, realqp *s, realqp*z, realqp*delta_s, realqp* delta_z, realqp alpha_p, realqp alpha_d, idxint m, idxint n, idxint p, idxint indicator);

void updatekktmatrix_b(realqp *b, realqp *rx, realqp *ry, realqp *rz, realqp *ds, realqp *z, idxint n, idxint m, idxint p);

idxint checksign(realqp*s, realqp *delta_s, realqp alpha, idxint count);

void updatevariables(realqp *x, realqp *delta_x, realqp alpha, idxint count);

void formlambda(realqp *lambda, realqp *s, realqp*z, idxint n);

realqp formrho(realqp *s, realqp*delta_s, realqp*z, realqp*delta_z, realqp alpha_p, realqp alpha_d, idxint n);

idxint ldlinitialsolve(kkt* mykkt, realqp*delta);

idxint ldlitersolve(kkt* mykkt, realqp*delta);

realqp innerproduct(realqp *x, realqp *y, idxint n);

void findminmax(realqp*z, long n, realqp*min, realqp*max);

void findsteplength(realqp*s, realqp*delta_s, realqp*z, realqp*delta_z, idxint m, realqp*alpha_p, realqp*alpha_d);

void test_reach(idxint *Parent, idxint *Pinv, idxint *UPattern, idxint n, idxint m, idxint p);

#endif