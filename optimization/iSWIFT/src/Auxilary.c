#include "Prime.h"

// Forms the Upper triangular part of the kkt matrix
// Status: Inactive
smat* formkktmatrix_U(smat* P, smat* G) {

	idxint i, j, k, kkt_nnz;

	idxint *kkt_ir, *kkt_jc;
	realqp *kkt_pr;
	smat *kkt, *Gt;
	kkt = (smat*)malloc(sizeof(smat));
	Gt = (smat*)malloc(sizeof(smat));

	kkt_jc = (idxint*)malloc((P->n + G->m + 1)*sizeof(idxint));

	kkt_ir = (idxint*)malloc((G->nnz + ((P->nnz + P->n) / 2) + G->m)*sizeof(idxint));

	kkt_pr = (realqp*)malloc((G->nnz + ((P->nnz + P->n) / 2) + G->m)*sizeof(realqp));

	Gt = SparseMatrixTranspose(G);
	kkt_nnz = 0;
	kkt_jc[0] = 0;

	for (i = 0; i < P->n; i++){
		for (j = P->jc[i]; j < P->jc[i + 1]; j++){
			k = P->ir[j];
			if (k <= i){
				kkt_ir[kkt_nnz] = k;
				kkt_pr[kkt_nnz] = P->pr[j];
				kkt_nnz++;
			}
		}
		kkt_jc[i + 1] = kkt_nnz;
	}



	for (i = 0; i < Gt->n; i++){
		for (j = Gt->jc[i]; j < Gt->jc[i + 1]; j++){
			kkt_ir[kkt_nnz] = Gt->ir[j];
			kkt_pr[kkt_nnz] = Gt->pr[j];
			kkt_nnz++;
			if (j == Gt->jc[i + 1] - 1){
				kkt_ir[kkt_nnz] = P->n + i;
				kkt_pr[kkt_nnz] = -1.0;
				kkt_nnz++;
			}
		}
		kkt_jc[i + 1 + P->n] = kkt_nnz;
	}



	kkt = SparseMatrixSetup(P->m, P->n + Gt->n, kkt_nnz, kkt_jc, kkt_ir, kkt_pr);

	free(Gt->ir);
	free(Gt->jc);
	free(Gt->pr);
	free(Gt);
	return kkt;
}

// Forms the full kkt matrix based on P, G and A

smat* formkktmatrix_full(smat* P, smat* G, smat* A) {

	if (A){
		smat  *Gt, *At;
		Gt = SparseMatrixTranspose(G);
		At = SparseMatrixTranspose(A);
		idxint *kkt_ir, *kkt_jc;
		realqp *kkt_pr;
		idxint kkt_nnz = 0;
		idxint i, j;
		kkt_jc = (idxint*)malloc((P->n + A->m + G->m + 1)*sizeof(idxint));
		kkt_pr = (realqp*)malloc((P->nnz + (2 * G->nnz) + G->m + (2 * A->nnz))*sizeof(realqp));
		kkt_ir = (idxint*)malloc((P->nnz + (2 * G->nnz) + G->m + (2 * A->nnz))*sizeof(idxint));

		kkt_jc[0] = 0;

		// Concatenating Matrices P, A, G vertically
		for (i = 0; i < P->n; i++){
			for (j = P->jc[i]; j < P->jc[i + 1]; j++){
				kkt_pr[kkt_nnz] = P->pr[j];
				kkt_ir[kkt_nnz] = P->ir[j];
				kkt_nnz++;
			}

			for (j = A->jc[i]; j < A->jc[i + 1]; j++){
				kkt_pr[kkt_nnz] = A->pr[j];
				kkt_ir[kkt_nnz] = P->m + A->ir[j];
				kkt_nnz++;
			}

			for (j = G->jc[i]; j < G->jc[i + 1]; j++){
				kkt_pr[kkt_nnz] = G->pr[j];
				kkt_ir[kkt_nnz] = P->m + A->m + G->ir[j];
				kkt_nnz++;
			}
			kkt_jc[i + 1] = P->jc[i + 1] + G->jc[i + 1] + A->jc[i + 1];
		}


		// Concatenating Matrices A', G' horizontally and G' and -I vertically
		for (i = 0; i < At->n; i++){
			for (j = At->jc[i]; j < At->jc[i + 1]; j++){
				kkt_ir[kkt_nnz] = At->ir[j];
				kkt_pr[kkt_nnz] = At->pr[j];
				kkt_nnz++;
			}
			kkt_jc[i + 1 + P->n] = kkt_nnz;
		}

		for (i = 0; i < Gt->n; i++){
			for (j = Gt->jc[i]; j < Gt->jc[i + 1]; j++){
				kkt_ir[kkt_nnz] = Gt->ir[j];
				kkt_pr[kkt_nnz] = Gt->pr[j];
				kkt_nnz++;
				if (j == Gt->jc[i + 1] - 1){
					kkt_ir[kkt_nnz] = P->m + A->m + i;
					kkt_pr[kkt_nnz] = -1.0;
					kkt_nnz++;
				}
			}
			kkt_jc[i + 1 + P->n + At->n] = kkt_nnz;
		}


		free(At->ir);
		free(At->jc);
		free(At->pr);
		free(At);
		free(Gt->ir);
		free(Gt->jc);
		free(Gt->pr);
		free(Gt);

		return SparseMatrixSetup(P->n + A->m + G->m, P->n + A->m + G->m, kkt_nnz, kkt_jc, kkt_ir, kkt_pr);
	}
	else{
		smat  *Gt;
		Gt = SparseMatrixTranspose(G);
		idxint *kkt_ir, *kkt_jc;
		realqp *kkt_pr;
		idxint kkt_nnz = 0;
		idxint i, j;
		kkt_jc = (idxint*)malloc((P->n + G->m + 1)*sizeof(idxint));
		kkt_pr = (realqp*)malloc((P->nnz + (2 * G->nnz) + G->m)*sizeof(realqp));
		kkt_ir = (idxint*)malloc((P->nnz + (2 * G->nnz) + G->m)*sizeof(idxint));

		kkt_jc[0] = 0;

		// Concatenating Matrices P and G vertically
		for (i = 0; i < P->n; i++){
			for (j = P->jc[i]; j < P->jc[i + 1]; j++){
				kkt_pr[kkt_nnz] = P->pr[j];
				kkt_ir[kkt_nnz] = P->ir[j];
				kkt_nnz++;
			}

			for (j = G->jc[i]; j < G->jc[i + 1]; j++){
				kkt_pr[kkt_nnz] = G->pr[j];
				kkt_ir[kkt_nnz] = P->m + G->ir[j];
				kkt_nnz++;
			}
			kkt_jc[i + 1] = P->jc[i + 1] + G->jc[i + 1];
		}

		// Concatenating Matrices G' and -I horizontally
		for (i = 0; i < Gt->n; i++){
			for (j = Gt->jc[i]; j < Gt->jc[i + 1]; j++){
				kkt_ir[kkt_nnz] = Gt->ir[j];
				kkt_pr[kkt_nnz] = Gt->pr[j];
				kkt_nnz++;
				if (j == Gt->jc[i + 1] - 1){
					kkt_ir[kkt_nnz] = P->m + i;
					kkt_pr[kkt_nnz] = -1.0;
					kkt_nnz++;
				}
			}
			kkt_jc[i + 1 + P->n] = kkt_nnz;
		}

		free(Gt->ir);
		free(Gt->jc);
		free(Gt->pr);
		free(Gt);
		return SparseMatrixSetup(P->n + G->m, P->n + G->m, kkt_nnz, kkt_jc, kkt_ir, kkt_pr);

	}
}


// Updates the kkt matrix
// Status : Active
void updatekktmatrix(smat *kkt, realqp *s, realqp *z, realqp *delta_s, realqp *delta_z, realqp alpha_p, realqp alpha_d, idxint m, idxint n, idxint p, idxint indicator) {

	idxint index, i;
	if (indicator == 0){
		for (i = n + p; i < n + m + p; i++){
			index = kkt->jc[i + 1] - 1;
			kkt->pr[index] = -s[i - n - p] / z[i - n - p];
		}
	}
	if (indicator == 1){
		for (i = n + p; i < n + m + p; i++){
			index = kkt->jc[i + 1] - 1;
			kkt->pr[index] = -(s[i - n - p] / z[i - n - p] - 1);
		}
	}
	if (indicator == 2){
		for (i = n + p; i < n + m + p; i++){
			index = kkt->jc[i + 1] - 1;
			kkt->pr[index] = -(s[i - n - p] / z[i - n - p] - (s[i - n - p] - alpha_p*delta_s[i - n - p]) / (z[i - n - p] - alpha_d*delta_z[i - n - p]));
		}
	}
}


// Updates the variable x as
// x = x + alpha*delta_x;
//	Status : Active
void updatevariables(realqp *x, realqp *delta_x, realqp alpha, idxint count){
	for (long i = 0; i < count; i++)
		x[i] += delta_x[i] * alpha;
}


// Updates the cosntant part of the kkt matrix linear system
//	Status : Active
void updatekktmatrix_b(realqp *b, realqp *rx, realqp *ry, realqp *rz, realqp *ds, realqp *z, idxint n, idxint m, idxint p) {

	idxint i;
	for (i = 0; i < n; i++)
		b[i] = rx[i];

	for (i = n; i < n + p; i++)
		b[i] = ry[i - n];

	for (i = n + p; i < n + m + p; i++)
		b[i] = rz[i - n - p] - (ds[i - n - p] / z[i - n - p]);

}

// Updates the ds vector based on the selector as
// selector = 1 => ds = -lambda*lambda;
// selector = 2 => ds = -lambda*lambda - (delta_s*delta_z) + (sigma*mu);
// selector = 3 => ds = -lambda*lambda + (sigma*mu);
void form_ds(realqp* ds, realqp *lambda, realqp *delta_s, realqp *delta_z, realqp sigma, realqp mu, idxint m, idxint selector) {
	// Pure Newton Step
	if (selector == 0){
		for (idxint i = 0; i < m; i++)
			ds[i] = -lambda[i] * lambda[i];
	}
	// Centering + Corrector Step
	if (selector == 1){
		for (idxint i = 0; i < m; i++)
			ds[i] = -(lambda[i] * lambda[i]) - (delta_s[i] * delta_z[i]) + (sigma*mu);
	}
	// Centering Step
	if (selector == 2){
		for (idxint i = 0; i < m; i++)
			ds[i] = -(lambda[i] * lambda[i]) + (sigma*mu);
	}

}

// Calcualtes the step length alpha_p and alpha_d individually
// Status : Active
void findsteplength(realqp* s, realqp* delta_s, realqp* z, realqp* delta_z, idxint m, realqp* alpha_p, realqp* alpha_d) {

	alpha_p[0] = 1e10;
	alpha_d[0] = 1e10;
	idxint Flag1 = 0;
	idxint Flag2 = 0;

	for (idxint i = 0; i < m; i++){
		if (delta_s[i] < 0 && (-s[i] / delta_s[i]) < alpha_p[0]){
			alpha_p[0] = -(s[i] / delta_s[i]);
			Flag1 = 1;
		}
		if (delta_z[i] < 0 && (-z[i] / delta_z[i]) < alpha_d[0]) {
			alpha_d[0] = -(z[i] / delta_z[i]);
			Flag2 = 1;
		}
	}

	if (!Flag1)
		alpha_p[0] = 1;

	if (!Flag2)
		alpha_d[0] = 1;
}


// Checks if x + alpha*delta_x < 0
// Status : Active
idxint checksign(realqp*x, realqp *delta_x, realqp alpha, idxint count){

	idxint Flag = 0;
	for (idxint i = 0; i < count; i++)
		if (x[i] < -alpha*delta_x[i]){
			Flag = 1;
			break;
		}

	return Flag;
}

// Calculates the Eucledian norm of vector p
// Status: Active
realqp norm(realqp* p, idxint n){
	realqp k = 0;
	for (idxint i = 0; i < n; i++)
		k += p[i] * p[i];
	return sqrt(k);
}

// Calculates the inner product of vectors x and y
// Status: Active
realqp innerproduct(realqp* x, realqp*y, idxint n){
	realqp sum = 0;
	for (idxint i = 0; i < n; i++)
		sum += x[i] * y[i];

	return sum;

}


// Solves the kkt linear system and updates delta_z and delta_s
// Status : Active
idxint kktsolve_1(QP* myQP) {

	idxint i, Flag, d;
	idxint n = myQP->kkt->kktmatrix->n;
	timer t;
	tic(&t);
	d = ldl_numeric(n, myQP->kkt->kktmatrix->jc, myQP->kkt->kktmatrix->ir, myQP->kkt->kktmatrix->pr, myQP->kkt->Lp, myQP->kkt->Parent, myQP->kkt->Lnz, myQP->kkt->Li, myQP->kkt->Lx, myQP->kkt->D, myQP->kkt->Y, myQP->kkt->Pattern, myQP->kkt->Flag, myQP->kkt->P, myQP->kkt->Pinv);

	//  d = ldl_cache_numeric(n, myQP->kkt->kktmatrix->jc, myQP->kkt->kktmatrix->ir, myQP->kkt->kktmatrix->pr, myQP->kkt->Ltp, myQP->kkt->Lti, myQP->kkt->Li, myQP->kkt->Lp, myQP->kkt->Lx, myQP->kkt->D, myQP->kkt->Y, myQP->kkt->P, myQP->kkt->Pinv,myQP->kkt->UPattern,myQP->kkt->work);

	// d = ldl_row_cache_numeric(n, myQP->kkt->kktmatrix->jc, myQP->kkt->kktmatrix->ir, myQP->kkt->kktmatrix->pr, myQP->kkt->Ltp, myQP->kkt->Lti, myQP->kkt->Li, myQP->kkt->Lp, myQP->kkt->Lx, myQP->kkt->D, myQP->kkt->Y, myQP->kkt->P, myQP->kkt->Pinv, myQP->kkt->UPattern);

	myQP->stats->ldl_numeric += toc(&t);
	if (d == n)
	{
		/* solve Ax=b, overwriting b with the solution x */
		ldl_perm(n, myQP->delta, myQP->kkt->b, myQP->kkt->P);
		ldl_lsolve(n, myQP->delta, myQP->kkt->Lp, myQP->kkt->Li, myQP->kkt->Lx);
		ldl_dsolve(n, myQP->delta, myQP->kkt->D);
		ldl_ltsolve(n, myQP->delta, myQP->kkt->Lp, myQP->kkt->Li, myQP->kkt->Lx);
		ldl_permt(n, myQP->kkt->b, myQP->delta, myQP->kkt->P);
		Flag = 1;
	}
	else
		Flag = 0;


	if (Flag){
		for (i = myQP->n + myQP->p; i < myQP->n + myQP->p + myQP->m; i++)
			myQP->delta_z[i - myQP->n - myQP->p] = myQP->kkt->b[i];

		for (i = 0; i < myQP->m; i++)
			myQP->delta_s[i] = (myQP->ds[i] - (myQP->s[i] * myQP->delta_z[i])) / myQP->z[i];

	}
	return Flag;
}


// Solves the kktlinear system from results of kktsolve_1 and updates delta_x, delta_y, delta_z and delta_s
// Status : Active
void kktsolve_2(QP* myQP) {

	idxint i;
	idxint n = myQP->kkt->kktmatrix->n;

	ldl_perm(n, myQP->delta, myQP->kkt->b, myQP->kkt->P);
	ldl_lsolve(n, myQP->delta, myQP->kkt->Lp, myQP->kkt->Li, myQP->kkt->Lx);
	ldl_dsolve(n, myQP->delta, myQP->kkt->D);
	ldl_ltsolve(n, myQP->delta, myQP->kkt->Lp, myQP->kkt->Li, myQP->kkt->Lx);
	ldl_permt(n, myQP->kkt->b, myQP->delta, myQP->kkt->P);


	for (i = 0; i < myQP->n; i++)
		myQP->delta_x[i] = myQP->kkt->b[i];

	for (i = myQP->n; i < myQP->n + myQP->p; i++)
		myQP->delta_y[i - myQP->n] = myQP->kkt->b[i];

	for (i = myQP->n + myQP->p; i < myQP->n + myQP->p + myQP->m; i++)
		myQP->delta_z[i - myQP->n - myQP->p] = myQP->kkt->b[i];

	for (i = 0; i < myQP->m; i++)
		myQP->delta_s[i] = (myQP->ds[i] - (myQP->s[i] * myQP->delta_z[i])) / myQP->z[i];
}


// Invoked by kkt_initialize
// Creates and updates the LDL workspace variables
// Performs ldl_symbolic and stores the results
// Also Performs ldl_numeric ; ldl_perm; ldl_lsolve; ldl_dsolve; ldl_ltsolve; ldl_permt in the same order
// Status : Active
idxint ldlinitialsolve(kkt* mykkt, realqp* delta){

	idxint lnz, d;
	idxint n;
	n = mykkt->kktmatrix->n;

	// Allocate Memory

	mykkt->Y = (realqp*)malloc(n*sizeof(realqp));
	mykkt->D = (realqp*)malloc(n*sizeof(realqp));


	mykkt->Lp = (idxint*)malloc((n + 1)*sizeof(idxint));
	mykkt->Parent = (idxint*)malloc(n*sizeof(idxint));
	mykkt->Pattern = (idxint*)malloc(n*sizeof(idxint));
	mykkt->Flag = (idxint*)malloc(n*sizeof(idxint));
	mykkt->Lnz = (idxint*)malloc(n*sizeof(idxint));

	/* factorize A into LDL' (P and Pinv used) */
	ldl_symbolic(n, mykkt->kktmatrix->jc, mykkt->kktmatrix->ir, mykkt->Lp, mykkt->Parent, mykkt->Lnz, mykkt->Flag, mykkt->P, mykkt->Pinv);
	lnz = mykkt->Lp[n];

	mykkt->Li = (idxint*)malloc((lnz + 1)*sizeof(idxint));
	mykkt->Lx = (realqp*)malloc((lnz + 1)*sizeof(realqp));

	d = ldl_numeric(mykkt->kktmatrix->n, mykkt->kktmatrix->jc, mykkt->kktmatrix->ir, mykkt->kktmatrix->pr, mykkt->Lp, mykkt->Parent, mykkt->Lnz, mykkt->Li, mykkt->Lx, mykkt->D, mykkt->Y, mykkt->Pattern, mykkt->Flag, mykkt->P, mykkt->Pinv);

	mykkt->Lti = (idxint*)malloc(lnz*sizeof(idxint));
	mykkt->Ltp = (idxint*)malloc((n + 1)*sizeof(idxint));

	Transpose_Row_Count(n, n, mykkt->Li, mykkt->Lp, mykkt->Lti, mykkt->Ltp);

	if (d == n)
	{
		/* solve Ax=b, overwriting b with the solution x */
		ldl_perm(n, delta, mykkt->b, mykkt->P);
		ldl_lsolve(n, delta, mykkt->Lp, mykkt->Li, mykkt->Lx);
		ldl_dsolve(n, delta, mykkt->D);
		ldl_ltsolve(n, delta, mykkt->Lp, mykkt->Li, mykkt->Lx);
		ldl_permt(n, mykkt->b, delta, mykkt->P);
		return(1);
	}
	else
	{
		return (0);
	}




}


// Forms the vector lambda as
// lambda = sqrt(s/z)
// Status : Active
void formlambda(realqp *lambda, realqp *s, realqp*z, idxint n) {
	for (idxint i = 0; i < n; i++)
		lambda[i] = sqrt(s[i] * z[i]);
}


// Sets up the Sparse Matrix in Column Compressed Storage Format based on inputs
// Status : Active
smat * SparseMatrixSetup(idxint m, idxint n, idxint nnz, idxint* jc, idxint* ir, realqp* pr){
	smat *sparse;
	sparse = (smat*)malloc(sizeof(smat));
	sparse->ir = ir;
	sparse->jc = jc;
	sparse->pr = pr;
	sparse->m = m;
	sparse->n = n;
	sparse->nnz = nnz;
	return sparse;
}

// Computes ir and jc of the transpose matrix A
// Status : Active
void Transpose_Row_Count(idxint m, idxint n, idxint *Li, idxint *Lp, idxint *Lti, idxint *Ltp) {

	idxint i, j, k, index;

	idxint *count;
	count = (idxint*)malloc(m*sizeof(idxint));

	for (j = 0; j < m; j++)
		count[j] = 0;

	for (i = 0; i < n; i++) {
		for (j = Lp[i]; j < Lp[i + 1]; j++) {
			k = Li[j];
			count[k]++;
		}
	}

	Ltp[0] = 0;
	for (j = 0; j < m; j++)
		Ltp[j + 1] = Ltp[j] + count[j];

	for (j = 0; j < m; j++)
		count[j] = 0;

	for (i = 0; i < n; i++)
		for (j = Lp[i]; j < Lp[i + 1]; j++) {
			k = Li[j];
			index = Ltp[k] + count[k];
			Lti[index] = i;
			count[k]++;
		}


	free(count);

}


// Computes the residuals rx, ry and rz
// Status : Active
void computeresiduals(QP* myQP){

	// Compute rx = Px + G'z +c in three steps
	// Compute rx = -Px - G'z - c in three steps
	// Compute rx = -Px - A'y -G'z - c

	idxint i;
	SparseMatrixMultiply(myQP->P, myQP->x, myQP->rx, 1);
	SparseMatrixTransMultiply(myQP->G, myQP->z, myQP->rx, 0);
	if (myQP->p)
		SparseMatrixTransMultiply(myQP->A, myQP->y, myQP->rx, 0);

	updatevariables(myQP->rx, myQP->c, -1, myQP->n);

	// Compute ry = Ax - b in two steps
	// Compute ry = -Ax + b in two steps

	if (myQP->p){
		SparseMatrixMultiply(myQP->A, myQP->x, myQP->ry, 1);
		updatevariables(myQP->ry, myQP->b, 1, myQP->p);
	}

	// Compute rz = s + G*x - h in two steps
	// Compute rz = -s - Gx + h in two steps
	SparseMatrixMultiply(myQP->G, myQP->x, myQP->rz, 1);
	for (i = 0; i < myQP->m; i++)
		myQP->rz[i] += myQP->h[i] - myQP->s[i];


}

// Performs Sparse Matrix Vector Multiplication as
// start = 0 ; do nothing
// start !=0 ; y=0
// y = y - A'x
// Status : Active
void SparseMatrixTransMultiply(smat *A, realqp* x, realqp* y, idxint start){

	idxint i, j, k;

	if (start)
		for (i = 0; i < A->n; i++)
			y[i] = 0;

	for (j = 0; j<A->n; j++){
		for (k = A->jc[j]; k < A->jc[j + 1]; k++){
			y[j] -= A->pr[k] * x[A->ir[k]];
		}
	}

}

// Performs Sparse Matrix Vector Multiplication as
// start = 0 ; do nothing
// start !=0 ; y=0
// y = y - Ax
// Status : Active

void SparseMatrixMultiply(smat* A, realqp* x, realqp* y, idxint start) {
	idxint i, j;

	if (start)
		for (i = 0; i < A->m; i++)
			y[i] = 0;

	for (i = 0; i < A->n; i++){
		for (j = A->jc[i]; j < A->jc[i + 1]; j++){
			y[A->ir[j]] -= x[i] * A->pr[j];
		}
	}

}

// Gives the scalar rho as
// rho = (s+alpha_p*delta_s)*(z+alpha_d*delta_z)/s'z;
// Stauts : Active
realqp formrho(realqp *s, realqp*delta_s, realqp*z, realqp*delta_z, realqp alpha_p, realqp alpha_d, idxint n){

	realqp sum = 0.0;
	for (idxint i = 0; i < n; i++)
		sum += (s[i] + (alpha_p*delta_s[i])) *(z[i] + (alpha_d*delta_z[i]));

	sum = sum / innerproduct(s, z, n);

	return sum;
}

// Performs Sparse Matrix Transpose and outputs the transpose matrix in smat format
// Status : Active
smat* SparseMatrixTranspose(smat*A){
	idxint i, j, k, index;
	idxint *At_ir, *At_jc;
	realqp *At_pr;

	At_jc = (idxint*)malloc((A->m + 1)*sizeof(idxint));
	At_ir = (idxint*)malloc(A->nnz*sizeof(idxint));
	At_pr = (realqp*)malloc(A->nnz*sizeof(realqp));
	idxint *count;
	count = (idxint*)malloc(A->m*sizeof(idxint));

	for (j = 0; j < A->m; j++)
		count[j] = 0;

	for (i = 0; i < A->n; i++) {
		for (j = A->jc[i]; j < A->jc[i + 1]; j++) {
			k = A->ir[j];
			count[k]++;
		}
	}

	At_jc[0] = 0;
	for (j = 0; j < A->m; j++)
		At_jc[j + 1] = At_jc[j] + count[j];

	for (j = 0; j < A->m; j++)
		count[j] = 0;

	for (i = 0; i < A->n; i++)
		for (j = A->jc[i]; j < A->jc[i + 1]; j++) {
			k = A->ir[j];
			index = At_jc[k] + count[k];
			At_ir[index] = i;
			At_pr[index] = A->pr[j];

			count[k]++;
		}


	free(count);

	return SparseMatrixSetup(A->n, A->m, A->nnz, At_jc, At_ir, At_pr);

}

// Gives the minimum and maximum of the vector z
// Status : Active
void findminmax(realqp*z, long n, realqp*min, realqp*max){

	min[0] = z[0];
	max[0] = z[0];
	for (long i = 1; i < n; i++)
	{
		if (z[i] < min[0])
		{
			min[0] = z[i];
		}
		if (z[i] > max[0])
		{
			max[0] = z[i];
		}
	}
}


// Gets the initial condition by solving the kkt linear system and updates the variables x, y, z and s
// Status : Active
idxint kkt_initialize(QP* myQP){

	realqp *z_inter;
	realqp alpha_p;
	realqp alpha_d;
	idxint i;
	idxint n, m, p;
	n = myQP->n;
	m = myQP->m;
	p = myQP->p;


	z_inter = (realqp*)malloc(myQP->m*sizeof(realqp));

	for (i = 0; i < n; i++)
		myQP->kkt->b[i] = -myQP->c[i];

	for (i = n; i < n + p; i++)
		myQP->kkt->b[i] = myQP->b[i - n];

	for (i = n + p; i < n + p + m; i++)
		myQP->kkt->b[i] = myQP->h[i - n - p];

	idxint Flag = ldlinitialsolve(myQP->kkt, myQP->delta);

	myQP->kkt->UPattern = (idxint*)malloc(myQP->kkt->kktmatrix->n*sizeof(realqp));
	test_reach(myQP->kkt->Parent, myQP->kkt->Pinv, myQP->kkt->UPattern, n, m, p);

	if (Flag){
		for (i = 0; i < n; i++)
			myQP->x[i] = myQP->kkt->b[i];


		for (i = n; i < n + p; i++)
			myQP->y[i - n] = myQP->kkt->b[i];

		// Calculate z_inter = Gx - h in two steps
		// Calculate z_inter = -Gx
		SparseMatrixMultiply(myQP->G, myQP->x, z_inter, 1);
		// Add h to z
		updatevariables(z_inter, myQP->h, 1, m);


		// find alpha_p and alpha_d
		findminmax(z_inter, myQP->m, &alpha_p, &alpha_d);
		alpha_p = -alpha_p;
		if (alpha_p < 0)
		{
			for (i = 0; i < m; i++)
				myQP->s[i] = z_inter[i];
		}
		else
		{
			for (i = 0; i < m; i++)
				myQP->s[i] = z_inter[i] + (1 + alpha_p);
		}

		if (alpha_d < 0)
		{
			for (i = 0; i < m; i++)
				myQP->z[i] = -z_inter[i];
		}
		else
		{
			for (i = 0; i < m; i++)
				myQP->z[i] = -z_inter[i] + (1 + alpha_d);
		}


	}


	free(z_inter);

	return Flag;
}


void test_reach(idxint *Parent, idxint *Pinv, idxint *UPattern, idxint n, idxint m, idxint p){
	idxint i, j;
	idxint top = n + m + p;
	UPattern[n + m + p - 1] = top;

	for (j = n + p; j < n + p + m; j++){
		i = Pinv[j];

		for (; UPattern[i] != top && i != -1; i = Parent[i]) {
			UPattern[i] = top;
		}
	}

}
