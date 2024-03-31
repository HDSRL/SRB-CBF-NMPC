#include "mex.h"
#include "Prime.h"
#define NINFOFIELDS 4
void mexFunction( int nlhs, mxArray *plhs[],
                  int nrhs, const mxArray *prhs[])
{
    const mwSize ZERO[2] = {0, 0};
    if (nrhs != 8 && nrhs != 6)
        mexErrMsgTxt("Please enter the correct number of input arguments");
    
    /* Initialize Output Structure Fields */
    const char *infofields[NINFOFIELDS] = {"Exit_Flag","Iterations","Setup_Time","Solve_Time"};//,"KKT_Solve_Time"};
    
    /* Declare Variables for size of Input arguments*/
    const mwSize *size_P;
    const mwSize *size_c;
    const mwSize *size_A = NULL;
    const mwSize *size_b = NULL;
    const mwSize *size_G;
    const mwSize *size_h;
    const mwSize *size_sigma_d;
    const mwSize *size_Permut;
    
    /* Declare Variables for Output Arguments*/
    
    /* Declare Variables for Input Arguments*/
    const mxArray *P;
    const mxArray *c;
    const mxArray *A = NULL;
    const mxArray *b = NULL;
    const mxArray *G;
    const mxArray *h;
    const mxArray *sigma_d;
    const mxArray *Permut;
    
    /* Declare Input Arguments for Swift Functions*/
    real *Ppr;
    real *cpr;
    real *Apr = NULL;
    real *bpr = NULL;
    real *Gpr;
    real *hpr;
    real *sigma_dpr;
    idxint *Permutpr;
    idxint n;
    idxint m;
    idxint p;
    
    idxint *Pir;
    idxint *Pjc;
    idxint *Air = NULL;
    idxint *Ajc = NULL;
    idxint *Gir;
    idxint *Gjc;
    
    /* Variable Assignment */
    if (nrhs ==8){
        P = prhs[0]; size_P = P ? mxGetDimensions(P) : (const mwSize *) &ZERO;
        c = prhs[1]; size_c = c ? mxGetDimensions(c) : (const mwSize *) &ZERO;
        A = prhs[2]; size_A = A ? mxGetDimensions(A) : (const mwSize *) &ZERO;
        b = prhs[3]; size_b = b ? mxGetDimensions(b) : (const mwSize *) &ZERO;
        G = prhs[4]; size_G = G ? mxGetDimensions(G) : (const mwSize *) &ZERO;
        h = prhs[5]; size_h = h ? mxGetDimensions(h) : (const mwSize *) &ZERO;
        sigma_d = prhs[6]; size_sigma_d = sigma_d ? mxGetDimensions(sigma_d) : (const mwSize *) &ZERO;
        Permut = prhs[7]; size_Permut = Permut ? mxGetDimensions(Permut) : (const mwSize *) &ZERO;
    }
    else{
           P = prhs[0]; size_P = P ? mxGetDimensions(P) : (const mwSize *) &ZERO;
           c = prhs[1]; size_c = c ? mxGetDimensions(c) : (const mwSize *) &ZERO;
           G = prhs[2]; size_G = G ? mxGetDimensions(G) : (const mwSize *) &ZERO;
           h = prhs[3]; size_h = h ? mxGetDimensions(h) : (const mwSize *) &ZERO;
           sigma_d = prhs[4]; size_sigma_d = sigma_d ? mxGetDimensions(sigma_d) : (const mwSize *) &ZERO;
           Permut = prhs[5]; size_Permut = Permut ? mxGetDimensions(Permut) : (const mwSize *) &ZERO;
    }
    
    n = (idxint)size_c[0];
    m = (idxint)size_h[0];
    if (nrhs == 8){
        p = (idxint)size_b[0];
    }
    else{
        p = 0;
    }
    /* Error Checking*/
    
    if (!mxIsDouble(P) || !mxIsSparse(P))
     mexErrMsgTxt("P should be a real sparse matrix");

    if (size_P[0] != size_P[1] )
     mexErrMsgTxt("P should be a square matrix");

    if (size_P[0] != n)
     mexErrMsgTxt("Dimensions of P and c do not match");

    if (!mxIsDouble(G) || !mxIsSparse(G))
     mexErrMsgTxt("G should be a real sparse matrix");

    if (size_G[0] != m)
     mexErrMsgTxt("Dimensions of G and h don’t match");

    if (size_G[1] != n)
     mexErrMsgTxt("Dimensions of G and c do not match");

    if(nrhs==8){
    if (!mxIsDouble(A) || !mxIsSparse(A))
     mexErrMsgTxt("A should be a real sparse matrix");

    if (size_A[0] != p)
     mexErrMsgTxt("Dimensions of A and b don’t match");

    if (size_A[1] != n)
     mexErrMsgTxt("Dimensions of A and c do not match");
    }
    
    if (!mxIsDouble(c) || mxIsSparse(c))
     mexErrMsgTxt("c should be a real dense vector");

    if (size_c[1] != 1)
     mexErrMsgTxt("c should be a column vector");

    if (!mxIsDouble(h) || mxIsSparse(h))
     mexErrMsgTxt("h should be a real dense vector");

    if (size_h[1] != 1)
     mexErrMsgTxt("h should be a column vector");
    
    if(nrhs == 8){
    if (!mxIsDouble(b) || mxIsSparse(b))
     mexErrMsgTxt("b should be a real dense vector");

    if (size_b[1] != 1)
     mexErrMsgTxt("b should be a column vector");
    }
    
    if (size_Permut[1] != 1)
     mexErrMsgTxt("Permutation Matrix should be a column vector");
    
    if (size_Permut[0] != n + m + p)
     mexErrMsgTxt("Permutation Matrix is of incorrect dimensions");
    
    if (mxIsSparse(Permut))
     mexErrMsgTxt("Permutation should be a real dense vector");
    
    if ( !mxIsDouble(sigma_d) || mxIsSparse(sigma_d) || size_sigma_d[0] != 1 || size_sigma_d[1] != 1)
        mexErrMsgTxt("sigma_d should be a scalar value");
    
    /* Copy Data from Imput arguments to local variables*/
    if(P)
    {
        Ppr = (real*)mxGetPr(P);
        Pir = (idxint*)mxGetIr(P);
        Pjc = (idxint*)mxGetJc(P);
    }
    
    if(c)
    {
        cpr = (real*)mxGetPr(c);
    }
    
    if(A)
    {
        Apr = (real*)mxGetPr(A);
        Air = (idxint*)mxGetIr(A);
        Ajc = (idxint*)mxGetJc(A);
    }
    
    if(b)
    {
        bpr = (real*)mxGetPr(b);
    }
    
    if(G)
    {
        Gpr = (real*)mxGetPr(G);
        Gir = (idxint*)mxGetIr(G);
        Gjc = (idxint*)mxGetJc(G);
    }
    
    if(h)
    {
        hpr = (real*)mxGetPr(h);
    }
    
    if(Permut){
        Permutpr = (idxint*)mxGetPr(Permut);
    }
    
    if(sigma_d){
        sigma_dpr = (real*)mxGetPr(sigma_d);
    }
    
     QP* myQP;
    
     
    if (myQP != NULL && nrhs == 8){
        myQP = QP_SETUP(n, m, p, Pjc, Pir, Ppr, Ajc, Air, Apr, Gjc, Gir, Gpr, cpr, hpr, bpr,0.0,Permutpr);
    }
    if(myQP != NULL && nrhs == 6){
        myQP = QP_SETUP(n, m, 0, Pjc, Pir, Ppr, NULL, NULL, NULL, Gjc, Gir, Gpr, cpr, hpr, NULL,0.0,Permutpr);
    }
    
    idxint EXITCODE;
    if (myQP != NULL){
         EXITCODE = QP_SOLVE(myQP);
    }
    
     if( nlhs > 0 ){
        plhs[0] = mxCreateDoubleMatrix(n, 1, mxREAL);
        memcpy(mxGetPr(plhs[0]), myQP->x, n * sizeof(double));
    }
    
    if( nlhs > 1 ){
        plhs[1] = mxCreateStructMatrix(1, 1, NINFOFIELDS, infofields);
        
        /* 0. exitflag */
        mxArray *outvar;
        outvar = mxCreateDoubleMatrix(1, 1, mxREAL);
		*mxGetPr(outvar) = (double)EXITCODE ;
		mxSetField(plhs[1], 0, "Exit_Flag", outvar);
        
        /* 1. Iteration Count */
//         mxArray *outvar1;
		outvar = mxCreateDoubleMatrix(1, 1, mxREAL);
		*mxGetPr(outvar) = (double)myQP->stats->IterationCount;
		mxSetField(plhs[1], 0, "Iterations", outvar);
        
        /* 2. Setup Time */
        
		outvar = mxCreateDoubleMatrix(1, 1, mxREAL);
		*mxGetPr(outvar) = (double)myQP->stats->tsetup;
		mxSetField(plhs[1], 0, "Setup_Time", outvar);
        
//         /* 3. Solve Time */
//         mxArray *outvar3;
		outvar = mxCreateDoubleMatrix(1, 1, mxREAL);
		*mxGetPr(outvar) = (double)myQP->stats->tsolve;
		mxSetField(plhs[1], 0, "Solve_Time", outvar);
        
        /* 4. KKT Solve Time */
//         mxArray *outvar2;
// 		outvar2 = mxCreateDoubleMatrix(1, 1, mxREAL);
// 		*mxGetPr(outvar2) = (double)myQP->stats->kkt_time;
// 		mxSetField(plhs[1], 0, "KKT_Solve_Time", outvar2);

    }
    
    QP_CLEANUP(myQP);
    
}