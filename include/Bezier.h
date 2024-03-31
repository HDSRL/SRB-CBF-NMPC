#ifndef BEZIER_FUNCTIONS
#define BEZIER_FUNCTIONS

#include "stddef.h"

static void calcBezier(int n, const double *alpha, const double s, double *traj){
    n--;

    double x[18] = { 1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1 };
	double y[18] = { 1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1 };

	for (int i = 0; i < n; i++) {
		x[i + 1] = s * x[i];
		y[i + 1] = (1 - s) * y[i];
	}

	const double *k = NULL ;
	const double k2[2] = { 1,1 };
	const double k3[3] = { 1,2,1 };
	const double k4[4] = { 1, 3, 3, 1 };
	const double k5[5] = { 1, 4, 6, 4, 1 };
	const double k6[6] = { 1, 5, 10, 10, 5, 1 };
	const double k7[7] = { 1,6 ,15, 20, 15, 6, 1 };
	const double k8[8] = { 1, 7, 21, 35, 35, 21, 7, 1 };
	const double k9[9] = { 1, 8, 28, 56, 70, 56, 28, 8, 1 };
	const double k10[10] = { 1, 9, 36, 84, 126, 126, 84, 36, 9, 1 };
	const double k11[11] = { 1,10,45,120,210,252,210,120,45,10,1 };
	const double k21[21] = { 1,20,190,1140,4845,15504,38760,77520,125970,167960,184756,167960,125970,77520,38760,15504,4845,1140,190,20,1 };

    switch (n){
        case 1: k = k2;
            break;
        case 2: k = k3;
            break;
        case 3: k = k4;
            break;
        case 4: k = k5;
            break;
        case 5: k = k6;
            break;
        case 6: k = k7;
            break;
        case 7: k = k8;
            break;
        case 8: k = k9;
            break;
        case 9: k = k10;
            break;
        case 10: k = k11;
            break;
        case 20: k = k21;
            break;
        default:
            break;
    }
    *traj = 0.0;
    for(int i=0; i<n+1; i++){
        *traj+=alpha[i]*k[i]*x[i]*y[n-i];
    }
}

static void calcBezierd(int n, const double *alpha, const double s, double *traj){
    n--;

    double x[18] = { 1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1 };
    double y[18] = { 1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1 };

    for (int i = 0; i < n - 1; i++) {
        x[i + 1] = s * x[i];
        y[i + 1] = (1 - s) * y[i];
    }

    const double *k = NULL;
	const double k2[2] = { 2,2 };
	const double k3[3] = { 3,6,3 };
	const double k4[4] = { 4, 12, 12, 4 };
	const double k5[5] = { 5, 20, 30, 20, 5 };
	const double k6[6] = { 6, 30, 60, 60, 30, 6 };
	const double k7[7] = { 7, 42 ,105, 140, 105, 42, 7 };
	const double k8[8] = { 8, 56, 168, 280, 280, 168, 56, 8 };
	const double k9[9] = { 9, 72, 252, 504, 630, 504, 252, 72, 9 };
	const double k20[20] = { 20,380,3420,19380,77520,232560,542640,1007760,1511640,1847560,1847560,1511640,1007760,542640,232560,77520,19380,3420,380,20 };

    switch (n){
    	case 2: k = k2;
    		break;
    	case 3: k = k3;
    		break;
    	case 4: k = k4;
    		break;
    	case 5: k = k5;
    		break;
    	case 6: k = k6;
    		break;
    	case 7: k = k7;
    		break;
    	case 8: k = k8;
    		break;
    	case 9: k = k9;
    		break;
    	case 20: k = k20;
    		break;
    	default:
    		break;
	}
    *traj = 0.0;
    for(int i=0; i<n; i++){
        *traj+=(alpha[i+1]-alpha[i])*k[i]*x[i]*y[n-1-i];
    }
}

static void calcBeziera(int n, const double *alpha, const double s, double *traj){
    n--;

    double x[18] = { 1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1 };
    double y[18] = { 1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1 };

    for (int i = 0; i < n - 2; i++) {
        x[i + 1] = s * x[i];
        y[i + 1] = (1 - s) * y[i];
    }

    const double *k = NULL;
	const double k1[2] = { 2 };
	const double k2[2] = { 6,6 };
	const double k3[3] = { 12,24,12 };
	const double k4[4] = { 20, 60, 60, 20 };
	const double k5[5] = { 30, 120, 180, 120, 30 };
	const double k6[6] = { 42, 210, 420, 420, 210, 42 };
	const double k7[7] = { 56, 336 ,840, 1120, 840, 336, 56 };
	const double k8[8] = { 72, 504, 1512, 2520, 2520, 1512, 504, 72 };
	const double k20[19] = { 380,6840,58140,310080,1162800,3255840,7054320,12093120,16628040,18475600,12093120,7054320,3255840,1162800,310080,58140,6840,380 };

    switch (n){
    	case 2: k = k1;
    		break;
    	case 3: k = k2;
    		break;
    	case 4: k = k3;
    		break;
    	case 5: k = k4;
    		break;
    	case 6: k = k5;
    		break;
    	case 7: k = k6;
    		break;
    	case 8: k = k7;
    		break;
    	case 9: k = k8;
    		break;
    	case 20: k = k20;
    		break;
    	default:
    		break;
	}
    *traj = 0.0;
    for(int i=0; i<n-1; i++){
        *traj+=(alpha[i+2]-2*alpha[i+1]+alpha[i])*k[i]*x[i]*y[n-2-i];
    }
}

static void calcBezierAll(int n, double alpha[], const double s, double traj[3]){
    calcBezier( n, alpha, s, &traj[0]);
    calcBezierd(n, alpha, s, &traj[1]);
    calcBeziera(n, alpha, s, &traj[2]);
}

static void calcVaryingBezierAll(int n, double dt, const double alpha[], const  double dalpha[], const  double ddalpha[], const double s, double traj[3]){
	double temp1, temp2, temp3;
	
	/* Calculate position */
	/* C(alpha,s) */
	calcBezier(n,alpha,s,&traj[0]);
	
	/* Calculate velocity */
	/* C'(alpha,s)*(1/T) + C(dalpha,s) */
	calcBezierd(n,  alpha, s, &temp1);
	calcBezier( n, dalpha, s, &temp2);
	traj[1] = temp1/dt+temp2;

	/* Calculate acceleration */
	/* C''(alpha,s)*(1/T^2)+2C'(dalpha,s)*(1/t)+C(ddalpha,s) */
	calcBeziera(n,   alpha, s, &temp1);
	calcBezierd(n,  dalpha, s, &temp2);
	calcBezier( n, ddalpha, s, &temp3);
	traj[2] = temp1/(dt*dt)+2*temp2/(dt)+temp3;
}

#endif