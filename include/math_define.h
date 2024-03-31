#ifndef MATH_DEFINE_H
#define MATH_DEFINE_H

#include "math.h"
#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Sparse"

/* Matrix Data Structures*/
/* Column Major Storage Format for Matrix*/
typedef struct hzd_dmat {
	double *pr;               /* Vector of matrix values in column major format*/
	size_t m;					/* Number of Rows of the Matrix */
	size_t n;                  /* Number of Columns of the Matrix*/
} hzd_dmat;
/* Matrix Data Structures*/


static double cubic(
            double time,     ///< Current time
            double time_0,   ///< Start time
            double time_f,   ///< End time
            double x_0,      ///< Start state
            double x_f,      ///< End state
            double x_dot_0,  ///< Start state dot
            double x_dot_f   ///< End state dot
            )
{
  double x_t;

  if (time < time_0){
    x_t = x_0;
  }
  else if (time > time_f){
    x_t = x_f;
  }
  else{
    double elapsed_time = time - time_0;
    double total_time = time_f - time_0;
    double total_time2 = total_time * total_time;  // pow(t,2)
    double total_time3 = total_time2 * total_time; // pow(t,3)
    double total_x    = x_f - x_0;

    x_t = x_0 + x_dot_0 * elapsed_time

        + (3 * total_x / total_time2
           - 2 * x_dot_0 / total_time
           - x_dot_f / total_time)
        * elapsed_time * elapsed_time

        + (-2 * total_x / total_time3 +
           (x_dot_0 + x_dot_f) / total_time2)
        * elapsed_time * elapsed_time * elapsed_time;
  }
  return x_t;
}

static double cubicDot(double time,     ///< Current time
            double time_0,   ///< Start time
            double time_f,   ///< End time
            double x_0,      ///< Start state
            double x_f,      ///< End state
            double x_dot_0,  ///< Start state dot
            double x_dot_f   ///< End state dot
            )
{
    double x_t;

    if (time < time_0){
        x_t = x_dot_0;
    }
    else if (time > time_f){
        x_t = x_dot_f;
    }
    else{
        double elapsed_time = time - time_0;
        double total_time = time_f - time_0;
        double total_time2 = total_time * total_time;  // pow(t,2)
        double total_time3 = total_time2 * total_time; // pow(t,3)
        double total_x    = x_f - x_0;

        x_t = x_dot_0

            + 2*(3 * total_x / total_time2
            - 2 * x_dot_0 / total_time
            - x_dot_f / total_time)
            * elapsed_time

            + 3*(-2 * total_x / total_time3 +
            (x_dot_0 + x_dot_f) / total_time2)
            * elapsed_time * elapsed_time;
    }
    return x_t;
}

static double cubicDotDot(double time,     ///< Current time
            double time_0,   ///< Start time
            double time_f,   ///< End time
            double x_0,      ///< Start state
            double x_f,      ///< End state
            double x_dot_0,  ///< Start state dot
            double x_dot_f   ///< End state dot
            )
{
    double x_t;

    if (time < time_0){
        x_t = 0;
    }
    else if (time > time_f){
        x_t = 0;
    }
    else{
        double elapsed_time = time - time_0;
        double total_time = time_f - time_0;
        double total_time2 = total_time * total_time;  // pow(t,2)
        double total_time3 = total_time2 * total_time; // pow(t,3)
        double total_x    = x_f - x_0;

        x_t =  2*(3 * total_x / total_time2
            - 2 * x_dot_0 / total_time
            - x_dot_f / total_time)

            + 6*(-2 * total_x / total_time3 +
            (x_dot_0 + x_dot_f) / total_time2)
            * elapsed_time;
    }
    return x_t;
}

static Eigen::Vector3d QuinticSpline(
                double time,       ///< Current time
                double time_0,     ///< Start time
                double time_f,     ///< End time
                double x_0,        ///< Start state
                double x_dot_0,    ///< Start state dot
                double x_ddot_0,   ///< Start state ddot
                double x_f,        ///< End state
                double x_dot_f,    ///< End state
                double x_ddot_f )  ///< End state ddot
{
    double a1,a2,a3,a4,a5,a6;
    double time_s;

    Eigen::Vector3d result;

    if(time < time_0)
    {
        result << x_0, x_dot_0, x_ddot_0;
        return result;
    }
    else if (time > time_f)
    {
        result << x_f, x_dot_f, x_ddot_f;
        return result;
    }


    time_s = time_f - time_0;
    a1=x_0;
    a2=x_dot_0;
    a3=x_ddot_0/2.0;

    Eigen::Matrix3d Temp;
    Temp<<pow(time_s, 3), pow(time_s, 4), pow(time_s, 5),
            3.0 * pow(time_s, 2), 4.0 * pow(time_s, 3), 5.0 * pow(time_s, 4),
            6.0 * time_s, 12.0 * pow(time_s, 2), 20.0 * pow(time_s, 3);

    Eigen::Vector3d R_temp;
    R_temp<<x_f-x_0-x_dot_0*time_s-x_ddot_0*pow(time_s,2)/2.0,
            x_dot_f-x_dot_0-x_ddot_0*time_s,
            x_ddot_f-x_ddot_0;

    Eigen::Vector3d RES;

    RES = Temp.inverse()*R_temp;

    a4=RES(0);
    a5=RES(1);
    a6=RES(2);

    double time_fs = time - time_0;

    double position = a1+a2*pow(time_fs,1)+a3*pow(time_fs,2)+a4*pow(time_fs,3)+a5*pow(time_fs,4)+a6*pow(time_fs,5);
    double velocity = a2+2.0*a3*pow(time_fs,1)+3.0*a4*pow(time_fs,2)+4.0*a5*pow(time_fs,3)+5.0*a6*pow(time_fs,4);
    double acceleration =2.0*a3+6.0*a4*pow(time_fs,1)+12.0*a5*pow(time_fs,2)+20.0*a6*pow(time_fs,3);


    result << position,velocity,acceleration;

    return result;
}

static void toEulerAngle(double qw, double qx, double qy, double qz, double& roll, double& pitch, double& yaw)
{
    double sinr = +2.0*(qw * qx + qy * qz);
    double cosr = +1.0-2.0*(qx * qx + qy * qy);
    roll = atan2(sinr,cosr);

    double sinp = +2.0*(qw * qy - qz * qx);
    if (fabs(sinp) >= 1)
        pitch = copysign(M_PI/2, sinp);
    else
        pitch = asin(sinp);

    double siny = +2.0*(qw * qz + qx * qy);
    double cosy = +1.0-2.0*(qy * qy + qz * qz);
    yaw = atan2(siny, cosy);

}

static double HZD_phaseVariable(double time, double time_0, double time_f){
    double s;
    s= (time-time_0)/(time_f-time_0);
    return s;
}

static void Eigen_mtx_to_hzd_dmat(Eigen::MatrixXd e_mtx, hzd_dmat* hzddmat){
    size_t rownum = e_mtx.rows();
    size_t colnum = e_mtx.cols();
    hzddmat->m = rownum;
    hzddmat->n = colnum;
    for(size_t i=0; i< colnum; i++){
        for(size_t j=0; j< rownum; j++){
            hzddmat->pr[j+i*rownum]=e_mtx(j,i);
        }
    }
}

static void HZD_bezier(hzd_dmat *alpha_mat, double s, double *val) {
	size_t m = alpha_mat->m;
	size_t n = alpha_mat->n;
	size_t M = n - 1;

	double x[18] = { 1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1 };
	double y[18] = { 1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1 };

	for (size_t i = 0; i < M; i++) {
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

	switch (M){
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

	for (size_t i = 0; i < m; i++) {
		val[i] = 0.0;
		for (size_t j = 0; j < M + 1; j++) {
			val[i] += alpha_mat->pr[(j*m) + i] * k[j] * x[j] * y[M - j];
		}
	}
}

static void HZD_bezierd(hzd_dmat *alpha_mat, double s, double *val) {
	size_t m = alpha_mat->m;
	size_t n = alpha_mat->n;
	size_t M = n - 1;

	double x[18] = { 1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1 };
	double y[18] = { 1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1 };

	for (size_t i = 0; i < M - 1; i++) {
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
    //TODO: high order part looks weird. Modify it someday
	switch (M){
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

	for (size_t i = 0; i < m; i++) {
		val[i] = 0.0;
		for (size_t j = 0; j < M; j++) {
			val[i] += (alpha_mat->pr[((j + 1)*m) + i] - alpha_mat->pr[(j*m) + i])* k[j] * x[j] * y[M - j - 1];
		}
	}
}

static void HZD_beziera(hzd_dmat *alpha_mat, double s, double *val) {
	size_t m = alpha_mat->m;
	size_t n = alpha_mat->n;
	size_t M = n - 1;

	double x[18] = { 1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1 };
	double y[18] = { 1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1 };

	for (size_t i = 0; i < M - 2; i++) {
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
    //TODO: high order part looks weird. Modify it someday
	switch (M){
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

	for (size_t i = 0; i < m; i++) {
		val[i] = 0.0;
		for (size_t j = 0; j < M - 1; j++) {
			val[i] += (alpha_mat->pr[((j + 2)*m) + i] - (2 * alpha_mat->pr[((j + 1)*m) + i]) + alpha_mat->pr[(j*m) + i])* k[j] * x[j] * y[M - j - 2];
		}
	}
}

static int factorial(int n){
    if(n>1){
        return n * factorial(n-1);
    }
    else{
        return 1;
    }
}

static double binomial(size_t M, size_t K, double phase){
    double mck = factorial(M)/(factorial(K)*factorial(M-K));
    double sxs_1 = pow(phase, K)*pow((1-phase), M-K);

    return mck * sxs_1;
}





#endif