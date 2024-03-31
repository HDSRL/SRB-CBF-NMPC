/* Author - Jeeseop Kim */

#include "iswift_qp.h"
#include "utils.h"
#include "iostream"

#include "Matrices_small.h"

/* very simple code to check if iswiftQp runs correctly. Put this block anywhere in the main function and run it*/
/*Eigen::Matrix<double, 5,5> p;
p = Eigen::MatrixXd::Identity(5,5);
double c[5] ={10, 15, 10, 5, 1};
Eigen::Matrix<double, 2,5> AAA;
AAA << 1,10,3,4,0,5,2,5,1,1;
double b[2]={1,1};
Eigen::Matrix<double, 1,5> G;
G<< 1,1,1,1,1;
double h[1]={3};
iswiftQp(p,c,AAA,b,G,h);*/

void iswiftQp_test(){
    QP* myQP;
	myQP = QP_SETUP(n, m, p, Pjc, Pir, Ppr, Ajc, Air, Apr, Gjc, Gir, Gpr, c, h, b, sigma_d, P);
	idxint ExitCode = QP_SOLVE(myQP);
	if (myQP != NULL)
		//nothing
	if (ExitCode == QP_OPTIMAL){
        //std::cout << "solve done with Optimal" <<std::endl;
	}
	if (ExitCode == QP_MAXIT){
        //std::cout << "solve done with Max iteration " << std::endl;
	}
	if (ExitCode == QP_FATAL){
        std::cout << "Unknown Error Dected during QP solving" << std::endl;
	}
	if (ExitCode == QP_KKTFAIL){
        std::cout << "LDL Factorization failed" <<std::endl;
	}
    printf("Solution Time in ms: %f\n",(myQP->stats->tsetup + myQP->stats->tsolve)*1000);

	QP_CLEANUP(myQP);
}

void iswiftQp_e(const Eigen::MatrixXd P, Eigen::MatrixXd c_e, const Eigen::MatrixXd A, Eigen::MatrixXd b_e, const Eigen::MatrixXd G, Eigen::MatrixXd h_e, double* sol){
    size_t size_c = c_e.rows();
    size_t size_b = b_e.rows();
    size_t size_h = h_e.rows();

    double* c;
    double* b;
    double* h;

    c = c_e.data();
    b = b_e.data();
    h = h_e.data();

    /*c = new double[size_c];
    b = new double[size_b];
    h = new double[size_h];

    for(size_t i=0; i< size_c; i++){
        c[i] = c_e(i);
    }
    for(size_t i=0; i< size_b; i++){
        b[i] = b_e(i);
    }
    for(size_t i=0; i< size_h; i++){
        h[i] = h_e(i);
    }*/

    iswiftQp(P,(double*) c, A,b, G,h, sol);

    //delete[] c;
    //delete[] b;
    //delete[] h;
}

void iswiftQp(const Eigen::MatrixXd P, double* c, const Eigen::MatrixXd A, double* b, const Eigen::MatrixXd G, double* h,double* sol){
    int n = P.innerSize();
    int m = G.innerSize();
    int p = A.innerSize();

    Eigen::SparseMatrix<double> P_sparse;
    Eigen::SparseMatrix<double> G_sparse;
    Eigen::SparseMatrix<double> A_sparse;

    P_sparse = P.sparseView();
    G_sparse = G.sparseView();
    A_sparse = A.sparseView();

    int nnz_P = P_sparse.nonZeros();
    int nnz_G = G_sparse.nonZeros();
    int nnz_A = A_sparse.nonZeros();

    int* permut;
    double sigma_d = 0.0;

    int *Pir, *Air, *Gir;
    int *Pjc, *Ajc, *Gjc;
    double *Ppr, *Apr, *Gpr;
    double *c_internal, *b_internal, *h_internal;

    Pir = (int*)malloc(nnz_P*sizeof(int));
    Pjc = (int*)malloc((n+1)*sizeof(int));

    Gir = (int*)malloc(nnz_G*sizeof(int));
    Gjc = (int*)malloc((n+1)*sizeof(int));

    Air = (int*)malloc(nnz_A*sizeof(int));
    Ajc = (int*)malloc((n+1)*sizeof(int));

    Ppr = (double*)malloc(nnz_P*sizeof(double));
    Gpr = (double*)malloc(nnz_G*sizeof(double));
    Apr = (double*)malloc(nnz_A*sizeof(double));

    permut = (int*)malloc((n+m+p)*sizeof(int));

    ccstorage(P, Pir, Pjc, Ppr);
    ccstorage(A, Air, Ajc, Apr);
    ccstorage(G, Gir, Gjc, Gpr);
    permutation(P, n, A, p, G, m, permut);

    QP* myQP;
	myQP = QP_SETUP(n, m, p, Pjc, Pir, Ppr, Ajc, Air, Apr, Gjc, Gir, Gpr, c, h, b, sigma_d, permut);

	idxint ExitCode = QP_SOLVE(myQP);
	if (myQP != NULL)
		//std::cout << "start QP solving using iSWIFT" << std::endl;
	if (ExitCode == QP_OPTIMAL){
        // nothing
	}
	if (ExitCode == QP_MAXIT){
//        std::cout << "Max Iterations reached" << std::endl;
	}
	if (ExitCode == QP_FATAL){
        std::cout << "Unknown Error Dected during QP solving" << std::endl;
	}
	if (ExitCode == QP_KKTFAIL){
        std::cout << "LDL Factorization failed" <<std::endl;
	}

    //printf("Number of Iterations : %d\n",myQP->stats->IterationCount);
    // printf("Solution Time in ms: %f\n",s(myQP->stats->tsetup + myQP->stats->tsolve)*1000);

    memcpy(sol, myQP->x, n*sizeof(double));

	QP_CLEANUP(myQP);

    free(Pir);
    free(Ppr);
    free(Pjc);

    free(Air);
    free(Apr);
    free(Ajc);

    free(Gir);
    free(Gpr);
    free(Gjc);

    free(permut);
}

void ccstorage(const Eigen::MatrixXd PAG, int* PAGir, int* PAGjc, double* PAGpr){
    Eigen::SparseMatrix<double> PAG_sparse;
    PAG_sparse = PAG.sparseView();
    PAG_sparse.makeCompressed();

    int m = PAG.outerSize();
    int nnz_PAG = PAG_sparse.nonZeros();

    for(size_t i=0; i< nnz_PAG; i++){
        PAGpr[i]=PAG_sparse.valuePtr()[i];
        PAGir[i]=PAG_sparse.innerIndexPtr()[i];
    }
    for(size_t i=0; i< m+1; i++){
        PAGjc[i]=PAG_sparse.outerIndexPtr()[i];
    }
    //PAGpr = PAG_sparse.valuePtr();
    //PAGir = PAG_sparse.innerIndexPtr();
    //PAGjc = PAG_sparse.outerIndexPtr();
}

void permutation(const Eigen::MatrixXd P, const int n, const Eigen::MatrixXd A, const int p, const Eigen::MatrixXd G, const int m, int* permut){
    //Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, 0, 1000, 1000> KKT_matrix;
    //Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, 0, 5, 5> KKT_matrix;
    Eigen::MatrixXd KKT_matrix(n+m+p, n+m+p);
    KKT_matrix.setZero(n+m+p, n+m+p);

    Eigen::SparseMatrix<double> KKT_matrix_sparse;

    KKT_matrix.block(0, 0, n, n)     = P;
    KKT_matrix.block(0, n, n, p)     = A.transpose();
    KKT_matrix.block(0, n+p, n, m)   = G.transpose();
    KKT_matrix.block(n, 0, p, n)     = A;
    KKT_matrix.block(n+p, 0, m, n)   = G;
    KKT_matrix.block(n+p, n+p, m, m) = -Eigen::MatrixXd::Identity(m, m);

    KKT_matrix_sparse = KKT_matrix.sparseView();

    Eigen::AMDOrdering<int> ordering;
    Eigen::PermutationMatrix<Eigen::Dynamic, Eigen::Dynamic, int> perm;
    Eigen::VectorXi indices;
    ordering(KKT_matrix_sparse, perm);
    indices = perm.indices();

    for(size_t i=0; i< n+m+p; i++){
        permut[i]=indices(i);
    }
}
