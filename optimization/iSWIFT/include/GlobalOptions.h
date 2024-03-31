#ifndef __GLOBALOPTIONS_H__
#define __GLOBAlOPTIONS_H__

// QP Header Files
#include <stdio.h>
#include <stdlib.h>
#include "ldl.h"
#include <math.h>


// QP Macro Functions
#define MAX(X,Y)  ((X) < (Y) ? (Y) : (X))  // Maximum of two expressions
#define MIN(X,Y)  ((X) > (Y) ? (Y) : (X))  // Minimum of two expressions

// QP Specific Varibles
//#define real double
//#define idxint int

typedef double realqp;
typedef int idxint;

// QP SOLVER Settings
#define MAXIT 25			// Maximum Number of Iterations
#define RELTOL 1e-6			// Residual Error Tolerances
#define ABSTOl 1e-6			// s and z Tolerances
#define SIGMA 100			// Centering Parameter


// QP SOLVER FLAGS(EXITCODES) and their Definitions

#define	QP_OPTIMAL	(0)		// Optimal Solution Found
#define QP_KKTFAIL  (1)		// Failure in solving LDL' factorization
#define QP_MAXIT	(2)		// Maximum Number of Iterations Exceeded
#define QP_FATAL	(3)		// Unknown Problem in Solver

#endif
