#ifndef FORCE_UPDATE
#define FORCE_UPDATE

#include "global_loco_structs.hpp"
#include "global_loco_opts.h"
#include "shared_structs.hpp"
#include "iswift_qp.h"
#include "EigenUtils.hpp"

namespace HighLevel{
	void updateDesiredForce(Eigen::Matrix<double, 6,1> &desAcc, Eigen::MatrixXd &desForce, sharedData *info);
}



#endif
