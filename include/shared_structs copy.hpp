#ifndef SHARED_DATA
#define SHARED_DATA

#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/Sparse"

#include "mutex"
#include <boost/thread/mutex.hpp>
#include <boost/thread/locks.hpp>
#define SET_DATA 1
#define GET_DATA 0
#define HL_DATA 1
#define LL_DATA 0
boost::mutex mtx0;
boost::mutex mtx1;


struct sharedData
{
	int resetRun = 0;

	// LL sets, used by HL
	double q[18] = {0};
	double dq[18] = {0};
	int ind[4] = {1};
	int domain = 0;
	Eigen::Matrix<double, 3, 4> toePos = Eigen::MatrixXd::Zero(3,4);

	// HL sets, used by LL
	Eigen::MatrixXd alpha_COM = Eigen::MatrixXd::Zero(4, 5);
	Eigen::MatrixXd MPC_sol_;

	// Both set, both use
	Eigen::Vector4d last_state0 = Eigen::MatrixXd::Zero(4,1);
	Eigen::Vector4d last_state1 = Eigen::MatrixXd::Zero(4,1); 
	bool runMPC = 1;
	bool MPC_data_available = 0;
};

sharedData data0;
sharedData data1;

void updateData0(int setget, int highlow, sharedData* newData)	// SETGET, HIGHLOW,  sharedData* newData
{ 
	// set = 1, get = 0
	// high = 1, low = 0
	// boost::lock_guard<boost::mutex> guard(mtx0);

	if (setget == SET_DATA)
	{
		// SET DATA
		if (highlow == HL_DATA)
		{
			data0.alpha_COM = newData->alpha_COM;
			data0.MPC_sol_ = newData->MPC_sol_;
			data0.domain = newData->domain;
			data0.MPC_data_available = newData->MPC_data_available;
			data0.runMPC = newData->runMPC;
			data0.resetRun = newData->resetRun;
			data0.last_state0 = newData->last_state0;
			// std::cout << "data0.last_state0: " << data0.last_state0.transpose(); std::cin.get();
		}
		else 
		{
			memcpy(data0.q,newData->q,18*sizeof(double)); 
			memcpy(data0.dq,newData->dq,18*sizeof(double));
			memcpy(data0.ind,newData->ind,4*sizeof(int));
			data0.domain = newData->domain;
			data0.toePos = newData->toePos;
			if(data0.resetRun==-1 && newData->resetRun==-2){
				data0.resetRun = 0;
			}
			if (data0.resetRun==0){
				data0.runMPC = newData->runMPC;
				data0.MPC_data_available = newData->MPC_data_available;
			}
		}
	}
	else	// GET DATA
	{
		if (highlow == HL_DATA)
		{
			newData->domain = data0.domain;
			memcpy(newData->q,data0.q,18*sizeof(double)); 
			memcpy(newData->dq,data0.dq,18*sizeof(double));
			memcpy(newData->ind,data0.ind,4*sizeof(int));
			newData->MPC_data_available = data0.MPC_data_available;
			newData->runMPC = data0.runMPC;
			newData->resetRun = data0.resetRun;
			newData->toePos = data0.toePos;
			newData->last_state1 = data1.last_state1;
		}
		else
		{
			newData->alpha_COM = data0.alpha_COM;
			newData->MPC_sol_ = data0.MPC_sol_;
			newData->MPC_data_available = data0.MPC_data_available;
			newData->runMPC = data0.runMPC;
			newData->resetRun = data0.resetRun;
			newData->domain = data0.domain;
			newData->last_state1 = data1.last_state1;
		}
	}
}

void updateData1(int setget, int highlow, sharedData* newData)	// SETGET, HIGHLOW,  sharedData* newData
{ 
	// set = 1, get = 0
	// high = 1, low = 0
	// boost::lock_guard<boost::mutex> guard(mtx1);

	if (setget == SET_DATA)
	{
		// SET DATA
		if (highlow == HL_DATA)
		{
			data1.alpha_COM = newData->alpha_COM;
			data1.MPC_sol_ = newData->MPC_sol_;
			data1.domain = newData->domain;
			data1.MPC_data_available = newData->MPC_data_available;
			data1.runMPC = newData->runMPC;
			data1.resetRun = newData->resetRun;
			data1.last_state1 = newData->last_state1;
		}
		else 
		{
			memcpy(data1.q,newData->q,18*sizeof(double)); 
			memcpy(data1.dq,newData->dq,18*sizeof(double));
			memcpy(data1.ind,newData->ind,4*sizeof(int));
			data1.domain = newData->domain;
			data1.toePos = newData->toePos;
			if(data1.resetRun==-1 && newData->resetRun==-2){
				data1.resetRun = 0;
			}
			if (data1.resetRun==0){
				data1.runMPC = newData->runMPC;
				data1.MPC_data_available = newData->MPC_data_available;
			}
		}
	}
	else	// GET DATA
	{
		if (highlow == HL_DATA)
		{
			newData->domain = data1.domain;
			memcpy(newData->q,data1.q,18*sizeof(double)); 
			memcpy(newData->dq,data1.dq,18*sizeof(double));
			memcpy(newData->ind,data1.ind,4*sizeof(int));
			newData->MPC_data_available = data1.MPC_data_available;
			newData->runMPC = data1.runMPC;
			newData->resetRun = data1.resetRun;
			newData->toePos = data1.toePos;
			newData->last_state0 = data0.last_state0;
			// std::cout << "newData->last_state0: " << newData->last_state0.transpose(); std::cin.get();
		}
		else
		{
			newData->alpha_COM = data1.alpha_COM;
			newData->MPC_sol_ = data1.MPC_sol_;
			newData->MPC_data_available = data1.MPC_data_available;
			newData->runMPC = data1.runMPC;
			newData->resetRun = data1.resetRun;
			newData->domain = data1.domain;
			newData->last_state0 = data0.last_state0;
		}
	}
}

void updateData(int setget, int highlow, sharedData* newData, int agent_number)
{
	boost::lock_guard<boost::mutex> guard(mtx0);

	if (agent_number == 0)
	{
		updateData0(setget, highlow, newData);
	}
	else if (agent_number == 1)
	{
		updateData1(setget, highlow, newData);
	}
}

void backupData(sharedData& copy_to, const sharedData& copy_from) {
    // Use a lock guard to ensure thread safety if necessary
    // boost::lock_guard<boost::mutex> guard(your_mutex);

    // Copying all fields from copy_from to copy_to
    copy_to.resetRun = copy_from.resetRun;
    
    // For arrays, use std::copy
    std::copy(std::begin(copy_from.q), std::end(copy_from.q), std::begin(copy_to.q));
    std::copy(std::begin(copy_from.dq), std::end(copy_from.dq), std::begin(copy_to.dq));
    std::copy(std::begin(copy_from.ind), std::end(copy_from.ind), std::begin(copy_to.ind));
    
    copy_to.domain = copy_from.domain;
    copy_to.toePos = copy_from.toePos;

    // For Eigen matrices, you can simply use assignment as they support deep copy by default
    copy_to.alpha_COM = copy_from.alpha_COM;
    copy_to.MPC_sol_ = copy_from.MPC_sol_;
    copy_to.last_state0 = copy_from.last_state0;
    copy_to.last_state1 = copy_from.last_state1;

    copy_to.runMPC = copy_from.runMPC;
    copy_to.MPC_data_available = copy_from.MPC_data_available;
}


#endif
