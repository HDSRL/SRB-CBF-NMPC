
// iSwift - Light Weight QP solver
// Copyright (C) Abhishek Pandala [pandala2@illinois.edu], Hae Won Park [haewon@illinois.edu], Yanran Ding [yding35@illinois.edu]
// Dynamic Robotics Lab, Department of Mechanical Science and Engineering, University of Illinois at Urbana - Champaign, USA

// This program is free software: you can redistribute it and/or modify
// it under the terms and consent of the authors



// Main Program Module
// Solves a QP of the form 
// Cost : x'Px + c'x
// s.t. Gx <= h
// and Ax = b
// P, G and A are to be supplied in Column Compressed Storage Format
// No error checking is performed; It is assumed that the matrices supplied are consistent in size and data type
// Permutation Matrix is supplied as a vector equal to the dimension of the kkt matrix


/* Permutation Matrix matlab Syntax

kkt = [P	A'	G']
[A	0	0]
[G	0	-I]
Permut = symamd(kkt) - 1;

*/
#include "Prime.h"

// Setup Function
// Allocates memory for all the variables
// Solves for the initial condition [x0,y0,z0,s0]
// Solves the result of LDL_symbolic and uses it for subsequent iterations
QP* QP_SETUP(idxint n, idxint m, idxint p, idxint *Pjc, idxint *Pir, realqp *Ppr, idxint*Ajc, idxint *Air, realqp *Apr, idxint *Gjc, idxint *Gir, realqp *Gpr, realqp *c, realqp *h, realqp *b, realqp sigma_d, idxint *Permut){

	timer tsetup;
	tic(&tsetup);
	QP *myQP;
	myQP = (QP*)malloc(sizeof(QP));
	// Initalize Matrices

	myQP->n = n;
	myQP->m = m;

	if (Apr && Ajc && Air && b && p != 0) {
		myQP->p = p;
		myQP->A = SparseMatrixSetup(myQP->p, myQP->n, Ajc[myQP->n], Ajc, Air, Apr);
	}
	else{
		myQP->p = 0;
		myQP->A = NULL;
	}

	myQP->P = SparseMatrixSetup(myQP->n, myQP->n, Pjc[myQP->n], Pjc, Pir, Ppr);
	myQP->G = SparseMatrixSetup(myQP->m, myQP->n, Gjc[myQP->n], Gjc, Gir, Gpr);
	myQP->c = c;
	myQP->h = h;
	myQP->b = b;
	myQP->sigma_d = sigma_d;

	myQP->x = (realqp*)malloc(myQP->n*sizeof(realqp));
	myQP->y = (realqp*)malloc(myQP->p*sizeof(realqp));
	myQP->z = (realqp*)malloc(myQP->m*sizeof(realqp));
	myQP->s = (realqp*)malloc(myQP->m*sizeof(realqp));


	myQP->rx = (realqp*)malloc(myQP->n*sizeof(realqp));
	myQP->ry = (realqp*)malloc(myQP->p*sizeof(realqp));
	myQP->rz = (realqp*)malloc(myQP->m*sizeof(realqp));

	myQP->delta = (realqp*)malloc((myQP->n + myQP->m + myQP->p)*sizeof(realqp));
	myQP->delta_x = (realqp*)malloc(myQP->n*sizeof(realqp));
	myQP->delta_y = (realqp*)malloc(myQP->p*sizeof(realqp));
	myQP->delta_z = (realqp*)malloc(myQP->m*sizeof(realqp));
	myQP->delta_s = (realqp*)malloc(myQP->m*sizeof(realqp));

	myQP->ds = (realqp*)malloc(myQP->m*sizeof(realqp));
	myQP->lambda = (realqp*)malloc(myQP->m*sizeof(realqp));



	// Initialise KKT Matrix
	myQP->kkt = (kkt*)malloc(sizeof(kkt));
	//myQP->kkt->kktmatrix = (smat*)malloc(sizeof(smat));
	// Forms the full kkt Matrix 
	//	Kkt =  [P   A'	G']
	//  	   [A   0	0 ]		with equality constraints included
	//		   [G	0   -I]

	//  kkt = [P	G']
	//		  [G	-I]			with inequality constraints
	myQP->kkt->kktmatrix = formkktmatrix_full(myQP->P, myQP->G, myQP->A);

	myQP->kkt->b = (realqp*)malloc((myQP->m + myQP->n + myQP->p)*sizeof(realqp));
	myQP->kkt->P = Permut;
	myQP->kkt->Pinv = (idxint*)malloc((myQP->m + myQP->n + myQP->p)*sizeof(idxint));

	// Initialise Settings
	myQP->options = (settings*)malloc(sizeof(settings));
	myQP->options->abstol = ABSTOl;
	myQP->options->reltol = RELTOL;
	myQP->options->maxit = MAXIT;
	myQP->options->sigma = SIGMA;
	// Initialise Settings


	// Initialise Stats
	myQP->stats = (stats*)malloc(sizeof(stats));
	myQP->stats->Flag = QP_FATAL;
	myQP->stats->IterationCount = 0;
	// Initialise Stats
	// Solves for Initial Condition
	if (!kkt_initialize(myQP)){
		myQP->stats->Flag = QP_KKTFAIL;
		printf("Status : %d", QP_KKTFAIL);
	}

	myQP->stats->ldl_numeric = 0;
	myQP->stats->tsetup = toc(&tsetup);
	return myQP;
}



// Main Solver Function
idxint QP_SOLVE(QP *myQP){
	timer tsolve;
	tic(&tsolve);
	timer kkt_t;
	realqp kkt_time = 0;

	realqp alpha_p = 0.0, alpha_d = 0.0;
	idxint Flag_kkt;

	if (myQP->stats->Flag != QP_KKTFAIL){

		for (idxint i = 0; i < myQP->options->maxit; i++){

			// Computes the residuals [rx;rz]

			computeresiduals(myQP);

			// Checks Exit condition if rx < ABSTOL rz < FEASTOl and s'z/m < 1e-6
			if (myQP->p){
				if (norm(myQP->rx, myQP->n) < myQP->options->reltol / sqrt(3.0) && norm(myQP->rz, myQP->m) < myQP->options->reltol / sqrt(3.0) && norm(myQP->ry, myQP->p) < myQP->options->reltol / sqrt(3.0) && innerproduct(myQP->s, myQP->z, myQP->m) / myQP->m < myQP->options->abstol) {
					myQP->stats->Flag = QP_OPTIMAL;
					break;
				}
			}
			else{
				if (norm(myQP->rx, myQP->n) < myQP->options->reltol / sqrt(3.0) && norm(myQP->rz, myQP->m) < myQP->options->reltol / sqrt(3.0) && innerproduct(myQP->s, myQP->z, myQP->m) / myQP->m < myQP->options->abstol) {
					myQP->stats->Flag = QP_OPTIMAL;
					break;
				}
			}


			// Updates lambda and mu
			formlambda(myQP->lambda, myQP->s, myQP->z, myQP->m);
			myQP->mu = innerproduct(myQP->lambda, myQP->lambda, myQP->m) / myQP->m;

			//printf("%f\t%f\t%f\t%f\n", myQP->kkt->D[336],myQP->kkt->D[337],myQP->s[0],myQP->z[0]);

			if (myQP->options->sigma > myQP->sigma_d){
				form_ds(myQP->ds, myQP->lambda, myQP->delta_s, myQP->delta_z, myQP->options->sigma, myQP->mu, myQP->m, 0);

				if (myQP->stats->IterationCount == 0){
					updatekktmatrix(myQP->kkt->kktmatrix, myQP->s, myQP->z, myQP->delta_s, myQP->delta_z, alpha_p, alpha_d, myQP->m, myQP->n, myQP->p, 0);
				}
				else{
					updatekktmatrix(myQP->kkt->kktmatrix, myQP->s, myQP->z, myQP->delta_s, myQP->delta_z, alpha_p, alpha_d, myQP->m, myQP->n, myQP->p, 0);
				}
				updatekktmatrix_b(myQP->kkt->b, myQP->rx, myQP->ry, myQP->rz, myQP->ds, myQP->z, myQP->n, myQP->m, myQP->p);

				tic(&kkt_t);
				Flag_kkt = kktsolve_1(myQP);
				if (!Flag_kkt){
					myQP->stats->Flag = QP_KKTFAIL;
					break;
				}

				kkt_time += toc(&kkt_t);


				findsteplength(myQP->s, myQP->delta_s, myQP->z, myQP->delta_z, myQP->m, &alpha_p, &alpha_d);

				myQP->rho = formrho(myQP->s, myQP->delta_s, myQP->z, myQP->delta_z, alpha_p, alpha_d, myQP->m);
				myQP->options->sigma = MAX(myQP->sigma_d, MIN(1, myQP->rho)*(MIN(1, myQP->rho))*(MIN(1, myQP->rho)));
				form_ds(myQP->ds, myQP->lambda, myQP->delta_s, myQP->delta_z, myQP->options->sigma, myQP->mu, myQP->m, 1);

			}
			else{
				myQP->options->sigma = myQP->sigma_d;
				form_ds(myQP->ds, myQP->lambda, myQP->delta_s, myQP->delta_z, myQP->options->sigma, myQP->mu, myQP->m, 2);
			}

			//updatekktmatrix(myQP->kkt->kktmatrix, myQP->s, myQP->z, myQP->m, myQP->n, myQP->p);
			updatekktmatrix_b(myQP->kkt->b, myQP->rx, myQP->ry, myQP->rz, myQP->ds, myQP->z, myQP->n, myQP->m, myQP->p);


			tic(&kkt_t);
			kktsolve_2(myQP);

			kkt_time += toc(&kkt_t);

			findsteplength(myQP->s, myQP->delta_s, myQP->z, myQP->delta_z, myQP->m, &alpha_p, &alpha_d);
			alpha_p = MIN(0.99*alpha_p, 1.0);
			alpha_d = MIN(0.99*alpha_d, 1.0);



			updatevariables(myQP->x, myQP->delta_x, alpha_p, myQP->n);
			updatevariables(myQP->y, myQP->delta_y, alpha_d, myQP->p);
			updatevariables(myQP->s, myQP->delta_s, alpha_p, myQP->m);
			updatevariables(myQP->z, myQP->delta_z, alpha_d, myQP->m);
			myQP->stats->IterationCount++;
		}



		if (myQP->stats->IterationCount == myQP->options->maxit){
			myQP->stats->Flag = QP_MAXIT;
		}

	}
	myQP->stats->tsolve = toc(&tsolve);
	myQP->stats->kkt_time = kkt_time;
	return myQP->stats->Flag;
}


// Solver Clean Up Routine
// Clears all the memeory except the input arguments of QP_SETUP function
// Cannot Access Solution once this function is invoked
void QP_CLEANUP(QP *myQP){

	// Free myQP Vectors
	free(myQP->x);
	free(myQP->y);
	free(myQP->z);
	free(myQP->s);
	free(myQP->delta);
	free(myQP->delta_x);
	free(myQP->delta_y);
	free(myQP->delta_z);
	free(myQP->delta_s);
	free(myQP->rx);
	free(myQP->ry);
	free(myQP->rz);
	free(myQP->ds);
	free(myQP->lambda);
	free(myQP->P);
	free(myQP->G);
	free(myQP->A);

	// Free myQP->kkt Structure
	free(myQP->kkt->kktmatrix->ir);
	free(myQP->kkt->kktmatrix->jc);
	free(myQP->kkt->kktmatrix->pr);
	free(myQP->kkt->kktmatrix);
	free(myQP->kkt->b);
	free(myQP->kkt->Parent);
	free(myQP->kkt->Flag);
	free(myQP->kkt->Lnz);
	free(myQP->kkt->Li);
	free(myQP->kkt->Lp);
	free(myQP->kkt->Lti);
	free(myQP->kkt->Ltp);
	free(myQP->kkt->Pattern);
	free(myQP->kkt->UPattern);
	free(myQP->kkt->Y);
	free(myQP->kkt->Lx);
	free(myQP->kkt->D);
	free(myQP->kkt->Pinv);
	free(myQP->kkt);

	// Free Settings
	free(myQP->options);

	// Free Statistics
	free(myQP->stats);

	// Free QP Vector
	free(myQP);

}
