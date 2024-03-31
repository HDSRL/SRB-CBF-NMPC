#include "LocoWrapper.hpp"
#include "iostream"
#include "dec_vars_constr_cost.h"
#include <ifopt/ipopt_solver.h>
#include <ifopt/snopt_solver.h>
#include <ifopt/problem.h>

using std::cin;
using std::cout;
using namespace ifopt;

void LocoWrapper::stepSetup(double totalStepNum, double step_X, double step_Y, double body_R, double body_P, double body_Y, Eigen::MatrixXd agent_Initial){
    totalStepNum_ = totalStepNum;
    // step_X_ = step_X;
    // step_Y_ = step_Y;
    body_R_ = body_R;
    body_P_ = body_P;
    body_Y_ = body_Y;
    agent_Initial_ = agent_Initial;
}

LocoWrapper::LocoWrapper(int argc, char *argv[]) : Parameters(argc,argv){
    std::string exp_name = "B_";
    static int counter_agents = 0;
    std::string filename = "///home/hdsrl7/Desktop/A1_RaiSim_Outputs/";
    filename += "agent_" + std::to_string(counter_agents); filename += ".txt"; log_ID = counter_agents;
    counter_agents++;
    if (counter_agents = 2) { counter_agents = 0; }
    data = std::unique_ptr<DataLog>( new DataLog(filename) ); // make_unique DNE in c++11
    quad = new RobotModel();
    conEst = new ContactEst();
    LL = new LowLevelCtrl();
    VC = new VirtualConstraints();
    PP = new MotionPlanner();

    state = quad->getStatePointer();
    dyn = quad->getDynamicsPointer();
    kin = quad->getKinematicsPointer();
    con = conEst->getConInfoPointer();
    traj = PP->getTrajInfoPointer();
    vcon = VC->getVCPointer();
    ll = LL->getllPointer();
    
    locoTick = 0;
    maxPhase = 1.05;

    onegaitCycle_ = 4; //const
    ts_OptTick_ = TSOPTTICK;
    gridNum_ = NDOMAIN;
    agent_id_ = 0;  // CHANGE FOR MULTIPLE AGENTS
    gaitDomain_ = 0;     // number of current Index column in totalCycleIndex: starting from 0th column =[1,1,1,1]
    body_R_ = BODYROLL;
    body_P_ = BODYPITCH;
    body_Y_ = BODYYAW;
    totalStepNum_ = TOTALSTEPNUM;

    mpc_state_alpha_buffer_.setZero(4,1);

    comTraj_[2] = {0};
    dcomTraj_[2] = {0};
    ddcomTraj_[2] = {0};

    //Eigen::Matrix<double, 2*NUMBER_OF_AGENTS, 1> Pstart;
    //Pstart << -1.5+3,3-3,-1.5+3,3-3+1,-1+3,3-3,-1+3,2-3,-2+3,2-3,0,0,0,0,0,0,0,0,0,0;
    //Pstart << -3,3,-2,3,-1,3,-1,2,-2,2,0,0,0,0,0,0,0,0,0,0;
    //Pstart << 2.0,-2,  2.0,-3;
    domain_ = 0;
    gaitDomain_ = domain_;
}

LocoWrapper::~LocoWrapper(){
    delete quad;
    delete conEst;
    delete LL;
    delete VC;
    delete PP;
}

void LocoWrapper::totalCycleIndexwHalf(size_t gait, size_t gaitCycleNum){

    oneCycleIndex(gait);
    totalCycleIndex_.setOnes(4,4*gaitCycleNum+4);
    for(size_t i=0; i< gaitCycleNum; i++){
        totalCycleIndex_.block(0,i*4+1,4,4)=oneCycleIndex_;
    }
    totalCycleIndex_.block(0,gaitCycleNum*4+1,4,2)=oneCycleIndex_.block(0,0,4,2);
    //totalStepNum_ = totalCycleIndex_.cols();
}

void LocoWrapper::oneCycleIndex(size_t gait){ //total half forward
    if(gait == STAND){
        oneCycleIndex_.setOnes(4,4);
    }    
    if(gait == WALK){
        oneCycleIndex_.setOnes(4,4);
        Eigen::Vector4d leg0stride;
        Eigen::Vector4d leg1stride;
        Eigen::Vector4d leg2stride;
        Eigen::Vector4d leg3stride;
        leg0stride<< 0,1,1,1;
        leg1stride<< 1,0,1,1;
        leg2stride<< 1,1,0,1;
        leg3stride<< 1,1,1,0;
        oneCycleIndex_.block(0,0,4,1) = leg0stride; //half stridekaveh
        oneCycleIndex_.block(0,2,4,1) = leg1stride; //half stride
        oneCycleIndex_.block(0,3,4,1) = leg2stride; //half stride
        oneCycleIndex_.block(0,1,4,1) = leg3stride; //half stride
    }
    if(gait == TROT){
        oneCycleIndex_.setOnes(4,4);
        Eigen::Vector4d leg03stride;
        Eigen::Vector4d leg12stride;
        leg03stride<< 0,1,1,0;
        leg12stride<< 1,0,0,1;
        oneCycleIndex_.block(0,0,4,1) = leg03stride; //half stride
        oneCycleIndex_.block(0,1,4,1) = leg12stride; //full stride
        oneCycleIndex_.block(0,2,4,1) = leg03stride; //full stride
        oneCycleIndex_.block(0,3,4,1) = leg12stride; //half stride       
    }
}

void LocoWrapper::totalCycleIndex(size_t gait, size_t gaitCycleNum){
    oneCycleIndex(gait);
    totalCycleIndex_.setOnes(4, 4*gaitCycleNum+2);
    for(size_t i=0; i< gaitCycleNum; i++){
        totalCycleIndex_.block(0,i*4+1,4,4)=oneCycleIndex_;
    }
    //totalStepNum_ = totalCycleIndex_.cols();
}

void LocoWrapper::plannedCycleIndex(size_t gait){
    size_t cycleNum;
    size_t remainder;
    remainder = (totalStepNum_-2) % onegaitCycle_;
    cycleNum = (totalStepNum_-2-remainder) / onegaitCycle_;

    if (remainder == 0){
        totalCycleIndex(gait, cycleNum);
    }
    else if(remainder == 2){
        totalCycleIndexwHalf(gait, cycleNum);
    }
    else{
        std::cout << "index generation error" << std::endl;
    }
}

void LocoWrapper::calcTau(const double q[18], const double dq[18], const double R[9], const int force[4], size_t gait, size_t ctrlTick, bool* checkrunMPC){
    //phaseVar = getPhase(1.0*locoTick, 0.0, 1.0*traj->domLen);   // update phase variable
    // phaseVar = getPhase(1.0*locoTick, 0.0, 1.0*ts_OptTick_*gridNum_);


    if(gait==STAND){
        phaseVar = getPhase(1.0*locoTick, 0.0, 1.0*traj->domLen);   // update phase variable
    }else{
        phaseVar = getPhase(1.0*locoTick, 0.0, 1.0*ts_OptTick_*gridNum_);
    }
    quad->updateState(q,dq,R); // update state
    // std::cout << "IS THE DATA ALREADY UPDATED? ===== " << MPC_data_set; std::cout << std::endl;

    static double comx_last = 0;
    static double comy_last = 0;
    
    float footPos[4] = {0};  // DUMMY VARS
    if (gait!=gaitTemp || (phaseVar>maxPhase && gait!=STAND) ){ 
        // Change domain immediately since gait changed
        conEst->forceDomChange();                                       // force con->changeDomain=1 to plan properly
        std::cout << "================================ Time trigger: ================================" << phaseVar << std::endl;
        
        *checkrunMPC = 1;  MPC_data_available = 0; 
        // Check the footholds
        

        phaseVar = (gaitTemp==STAND) ? 0 : phaseVar;
        getComTrajectoryEventbase(gait, gaitDomain_);
        // Eigen::Vector4d comTraj; comTraj << comTraj_[0], comTraj_[1], dcomTraj_[0], dcomTraj_[1];
        Eigen::Vector4d comTraj; comTraj << state->q[0] + 0.001*dcomTraj_[0], state->q[01] + 0.001*dcomTraj_[1], dcomTraj_[0], dcomTraj_[1];
        if (domain_ >= TOTALSTEPNUM) { comTraj(0) = comx_last; comTraj(1) = comy_last; comTraj(3) = 0; comTraj(4) = 0; }
        else {
            comx_last = comTraj(0); comy_last = comTraj(1);
        }
        phaseVar = 0;



        PP->setComDes(comTraj);
        PP->planTraj(state, kin, conEst, gait, phaseVar, ctrlTick, &motion_params, Pr_refined_, agent_id_, gaitDomain_, mpc_state_e_x_eventbased_);
        conEst->updateConState(footPos,phaseVar,force);
        locoTick = 0;
        gaitDomain_++;
    }else {
        // Wait for impact to change domain
        conEst->updateConState(footPos,phaseVar,force);                 // impact detection
        if (con->changeDomain==1 && gait!=STAND){
            locoTick = 0;
            std::cout << "Contact trigger: " << phaseVar << std::endl;
            phaseVar = 0;
            *checkrunMPC = 1; MPC_data_available = 0;
            gaitDomain_++;
        }

        getComTrajectoryEventbase(gait, gaitDomain_);
        if (gaitDomain_ == 0) {
            comTraj_[0] = agent_Initial_(0); comTraj_[1] = agent_Initial_(1);
        }
        // cout << "COM (x, y, dx, dy) = " << comTraj_[0] << "  " << comTraj_[1] << "  " << dcomTraj_[0] << "  " << dcomTraj_[0] << "\n";
        Eigen::Vector4d comTraj; comTraj << state->q[0] + 0.001*dcomTraj_[0], state->q[01] + 0.001*dcomTraj_[1], dcomTraj_[0], dcomTraj_[1];
        // Eigen::Vector4d comTraj; comTraj << 0, 0, 0, 0;
        // comTraj(2) = 0; comTraj(3) = 0;
        if (domain_ >= TOTALSTEPNUM) { comTraj(0) = comx_last; comTraj(1) = comy_last; comTraj(3) = 0; comTraj(4) = 0; }
        else {
            comx_last = comTraj(0); comy_last = comTraj(1);
        }

        if (!STAND) { PP->setComDes(comTraj);}

        PP->planTraj(state, kin, conEst, gait, phaseVar, ctrlTick, &motion_params, Pr_refined_, agent_id_, gaitDomain_, mpc_state_e_x_eventbased_);     // plan trajectory
    }

    quad->updateSwingMatrices(con->ind,con->cnt);                                           // update the jacobians
    VC->updateVirtualConstraints(state, kin, traj, con, gait, phaseVar, &motion_params, ll);     // update VC's
    LL->calcTorque(state, dyn, kin, vcon, con, &ll_params);                             // run low level controller
    data->writeData(state,vcon,traj,ll,ctrlTick,force,phaseVar);                      // log relavent data

    locoTick += (ctrlHz)/LL_Hz; // increment locoTick
    gaitTemp = gait; // update the previous gait used
}

void LocoWrapper::logMPC_Data()
{
    std::fstream MPC_Log_Data; ///home/hdsrl7/Desktop/A1_RaiSim_Outputs/exp_
    //MPC_Log_Data.open("/home/hdsrl7/Documents/RAISIM_WORKSPACE/A1_NMPC_oldLL_working/NMPC_Distributed_A1-master/matlab_dbg/MPC_loco.txt",std::fstream::out);
    MPC_Log_Data.open("///home/hdsrl7/Desktop/A1_RaiSim_Outputs/exp_B_MPC_loco" + std::to_string(log_ID) + ".txt",std::fstream::out);
    MPC_Log_Data << mpc_state_e_eventbased_;
    MPC_Log_Data.close();

    std::fstream COM_DES;
    COM_DES.open("///home/hdsrl7/Desktop/A1_RaiSim_Outputs/exp_B_COM_loco" + std::to_string(log_ID) + ".txt",std::fstream::out);
    COM_DES << com_desired_Traj_eventbased_;
    COM_DES.close();

    std::fstream COP;
    COP.open("///home/hdsrl7/Desktop/A1_RaiSim_Outputs/exp_B_COM_loco" + std::to_string(log_ID) + ".txt",std::fstream::out);
    COP << mpc_state_e_u_eventbased_;
    COP.close();
}

void LocoWrapper::setAgentID(size_t agent_id)
{
    agent_id_ = agent_id;
}


void LocoWrapper::generateReferenceTrajectory()
{
    double c = 4500.0, m = 8.0, epsilon = 1.0, sigma = 0.75;
    double dth = 1.0, alpha = 100.0, eta = 200.0, dmin = 2.0;
    int loopSize = 100000; size_t num_obs = Pobs.cols();

    double Ts = TSOPTTICK*0.001/10;

    MatrixXd Ad; Ad.setZero(4, 4);
    MatrixXd Ae; Ae.setZero(8, 8);

    Ad <<   1,                   0,   0.001021788129226,                   0,
            0,                   1,                   0,   0.001021788129226,
            0,                   0,   0.923365890308039,                   0,
            0,                   0,                   0,   0.923365890308039;

    Ae.block(0, 0, 4, 4) = Ad;
    Ae.block(4, 4, 4, 4) = Ad;

    MatrixXd Bd; Bd.setZero(4, 2);
    MatrixXd Be; Be.setZero(8, 4);

    Bd <<   0.000000067853117956,                   0,
            0,                   0.000000067853117956,
            0.000127723516153268,                   0,
            0,                   0.000127723516153268;

    Be.block(0, 0, 4, 2) = Bd;
    Be.block(4, 2, 4, 2) = Bd;

    // cout << "Ae is \n" << Ae << "\n\n\nBe is\n" << Be; cin.get();

    // Reference Loop
    MatrixXd q; q.setZero(8, loopSize+1);
    

    q.block(0, 0, 8, 1) << Pstart_(0), Pstart_(1), 0, 0, Pstart_(2), Pstart_(3), 0, 0;
    // cout << "q.first_col = " << q.block(0, 0, 8, 1).transpose(); cin.get(); 

    for (size_t i = 0; i < loopSize; i++)
    {   
        Vector4d F; F.setZero();
        // for each agent
        for (size_t k = 0; k <= 1; k++)     
        {
            // Initialized required variables
            Vector2d qk_p, qk_p_other, p_g, F_att, F_rep, F_agent; qk_p.setZero(); qk_p_other.setZero(); p_g.setZero(); F_att.setZero(); F_rep.setZero(); F_agent.setZero(); 
            VectorXd d_obs; d_obs.setZero(NUMBER_OF_OBS);
            double d_goal = 0.0, d_agent = 0.0;

            // Index current and other agent positions
            qk_p = q.block(0+k*4, i, 2, 1);
            qk_p_other = q.block(0+(!k)*4, i, 2, 1);


            // cout << "qk_p = " << qk_p.transpose(); cin.get(); 
            // cout << "qk_p_other = " << qk_p_other.transpose(); cin.get();

            // Goal point
            p_g << GOAL_X, GOAL_Y;
            // cout << "p_g = " << p_g.transpose(); cin.get();

            // Distance from goal
            d_goal = (qk_p - p_g).norm();
            // cout << "d_goal = " << d_goal; cin.get();

            // Attractive Force
            F_att = -alpha*((qk_p - p_g)/(qk_p - p_g).norm());
            // cout << "F_att = " << F_att.transpose(); cin.get();

            // Distance from Obstacle
            for (size_t j = 0; j < num_obs; j++)
            {
                d_obs(j) = (qk_p - Pobs.col(j)).norm();
            }
            //cout << "d_obs = " << d_obs.transpose();// cin.get();


            // Distance form other agent
            d_agent = (qk_p - qk_p_other).norm();
            // cout << "d_agent = " << d_agent; cin.get();

            // Repulsive Force due to obstacles
            Vector2d F_tmp; F_tmp.setZero();
            for (size_t j = 0; j < num_obs; j++)
            {
                if (d_obs(j) < dmin)
                {
                    F_tmp += eta*(1/d_obs(j) - 1/dmin)*(1/pow(d_obs(j), 2))*((qk_p - Pobs.col(j))/(qk_p - Pobs.col(j)).norm());
                }
            }
            F_rep = F_tmp;
            //cout << "\nF_rep = " << F_rep.transpose(); cin.get();

            // Lennard Jones Potential
            F_agent = -4*epsilon*( (6*pow(sigma, 6))/pow(d_agent, 7) - (12*pow(sigma, 12))/pow(d_agent, 13))*( (qk_p - qk_p_other)/(qk_p - qk_p_other).norm() );
            // cout << "F_agent = " << F_agent.transpose(); cin.get();

            // cout << "( (6*pow(sigma, 6))/pow(d_agent, 7) - (12*pow(sigma, 12))/pow(d_agent, 13)) = " <<  (6*pow(sigma, 6))/pow(d_agent, 7) - (12*pow(sigma, 12))/pow(d_agent, 13); cin.get();
            
            F.block(0 + k*2, 0, 2, 1) = F_att + F_rep + F_agent;

            if (d_goal < 0.001)
            {
                F.block(0 + k*2, 0, 2, 1) = 0*F_att;
            }
        }
        // cout << "F = " << F.transpose(); cin.get(); 

        q.col(i+1) = Ae*q.col(i) + Be*F;

        // if (i % 1000 == 0)
        // cout << "loop " << i << std::endl;

    }

    for (size_t i = 0; i < loopSize/40; i++)
    {
        q.col(i) = q.col(40*i);
    }

    MatrixXd Pr; Pr.setZero(4, loopSize/40);
    MatrixXd Prd; Prd.setZero(4, loopSize/40);
    
    // Agent 1
    Pr.row(0) = q.block(0, 0, 1, loopSize/40);
    Pr.row(1) = q.block(1, 0, 1, loopSize/40);
    Prd.row(0) = q.block(2, 0, 1, loopSize/40);
    Prd.row(1) = q.block(3, 0, 1, loopSize/40);

    // Agent 2
    Pr.row(2) = q.block(4, 0, 1, loopSize/40);
    Pr.row(3) = q.block(5, 0, 1, loopSize/40);
    Prd.row(2) = q.block(6, 0, 1, loopSize/40);
    Prd.row(3) = q.block(7, 0, 1, loopSize/40);

    // Pr_ = Pr;
    // Prd_ = Prd;

    // for (size_t i = 0; i < NAgents; i++)
    // {
    //     Pr.block(2*i,k+1,2,1) = q.block(4*i,k+1,2,1);
    //     Prd.block(2*i,k+1,2,1) = q.block(4*i+2,k+1,2,1);
    // }

    // =================== PROVIDE CUSTOM REFERENCE =========================== //

    // for (int i = 0; i < Pr.cols(); i++)
    // {
    //     // Pr.block(0, i, 4, 1) << 0 + 0.001*i, 0 - 0.001*i, 0 + 0.001*i, 0.001*i-1.5;
    //     // Prd.block(0, i, 4, 1) << 0.05, -0.05, 0.05, 0.05; 
    //     // Go straight - agent 1
    //     double step_x1 = 0.005*2; double step_dx1 = 0.1*2;
    //     double step_x2 = 0.005; double step_dx2 = 0.1;

    //     Pr.block(0, i, 4, 1) << 0 + step_x1*(1-exp(-0.1*i))*i, 0 - step_x1*(1-exp(-0.1*i))*i, 0 + step_x1*(1-exp(-0.1*i))*i, + step_x1*(1-exp(-0.1*i))*i-0.75;
    //     Prd.block(0, i, 4, 1) << step_dx1*(1-exp(-0.1*i)), -step_dx1*(1-exp(-0.1*i)), step_dx1*(1-exp(-0.1*i)), step_dx1*(1-exp(-0.1*i));
    // }
    // ======= END ======= PROVIDE CUSTOM REFERENCE =========== END =========== //


    

    Pr_refined_ = Pr;
    Prd_refined_ = Prd;
    
    std::fstream myfile5;
    myfile5.open("///home/hdsrl7/Desktop/A1_RaiSim_Outputs/RefTrajMPC_EXPE_" + std::to_string(log_ID) + ".txt",std::fstream::out);
    myfile5 << Pr_refined_;
    myfile5.close();

    std::fstream myfile6;
    myfile6.open("///home/hdsrl7/Desktop/A1_RaiSim_Outputs/VelTrajMPC_EXPE_" + std::to_string(log_ID) + ".txt",std::fstream::out);
    myfile6 << Prd_refined_;
    myfile6.close();


    // cout << "Can we reach here???????"; cin.get();
}


// void LocoWrapper::generateReferenceTrajectory()
// {
//     double c = 4500.0, m = 8.0, epsilon = 1.0, sigma = 0.75;
//     double dth = 1.0, alpha = 100.0, eta = 200.0, dmin = 2.0;
//     int loopSize = 100000; size_t num_obs = Pobs.cols();

//     double Ts = TSOPTTICK*0.001/10;

//     MatrixXd Ad; Ad.setZero(4, 4);
//     MatrixXd Ae; Ae.setZero(8, 8);

//     Ad <<   1,                   0,   0.00102178812NUMBER_OF_OBS226,                   0,
//             0,                   1,                   0,   0.00102178812NUMBER_OF_OBS226,
//             0,                   0,   0.923365890308039,                   0,
//             0,                   0,                   0,   0.923365890308039;

//     Ae.block(0, 0, 4, 4) = Ad;
//     Ae.block(4, 4, 4, 4) = Ad;

//     MatrixXd Bd; Bd.setZero(4, 2);
//     MatrixXd Be; Be.setZero(8, 4);

//     Bd <<   0.000000067853117956,                   0,
//             0,                   0.000000067853117956,
//             0.000127723516153268,                   0,
//             0,                   0.000127723516153268;

//     Be.block(0, 0, 4, 2) = Bd;
//     Be.block(4, 2, 4, 2) = Bd;

//     // cout << "Ae is \n" << Ae << "\n\n\nBe is\n" << Be; cin.get();

//     // Reference Loop
//     MatrixXd q; q.setZero(8, loopSize);
    

//     q.block(0, 0, 8, 1) << Pstart_(0), Pstart_(1), 0, 0, Pstart_(2), Pstart_(3), 0, 0;
//     // cout << "q.first_col = " << q.block(0, 0, 8, 1).transpose(); cin.get(); 

//     for (size_t i = 0; i < loopSize; i++)
//     {   
//         Vector4d F; F.setZero();
//         // for each agent
//         for (size_t k = 0; k <= 1; k++)     
//         {
//             // Initialized required variables
//             Vector2d qk_p, qk_p_other, p_g, F_att, F_rep, F_agent; qk_p.setZero(); qk_p_other.setZero(); p_g.setZero(); F_att.setZero(); F_rep.setZero(); F_agent.setZero(); 
//             VectorXd d_obs; d_obs.setZero(9);
//             double d_goal = 0.0, d_agent = 0.0;

//             // Index current and other agent positions
//             qk_p = q.block(0+k*4, i, 2, 1);
//             qk_p_other = q.block(0+(!k)*4, i, 2, 1);


//             // cout << "qk_p = " << qk_p.transpose(); cin.get(); 
//             // cout << "qk_p_other = " << qk_p_other.transpose(); cin.get();

//             // Goal point
//             p_g << GOAL_X, GOAL_Y;
//             // cout << "p_g = " << p_g.transpose(); cin.get();

//             // Distance from goal
//             d_goal = (qk_p - p_g).norm();
//             // cout << "d_goal = " << d_goal; cin.get();

//             // Attractive Force
//             F_att = -alpha*((qk_p - p_g)/(qk_p - p_g).norm());
//             // cout << "F_att = " << F_att.transpose(); cin.get();

//             // Distance from Obstacle
//             for (size_t j = 0; j < num_obs; j++)
//             {
//                 d_obs(j) = (qk_p - Pobs.col(j)).norm();
//             }
//             //cout << "d_obs = " << d_obs.transpose();// cin.get();


//             // Distance form other agent
//             d_agent = (qk_p - qk_p_other).norm();
//             // cout << "d_agent = " << d_agent; cin.get();

//             // Repulsive Force due to obstacles
//             Vector2d F_tmp; F_tmp.setZero();
//             for (size_t j = 0; j < num_obs; j++)
//             {
//                 if (d_obs(j) < dmin)
//                 {
//                     F_tmp += eta*(1/d_obs(j) - 1/dmin)*(1/pow(d_obs(j), 2))*((qk_p - Pobs.col(j))/(qk_p - Pobs.col(j)).norm());
//                 }
//             }
//             F_rep = F_tmp;
//             //cout << "\nF_rep = " << F_rep.transpose(); cin.get();

//             // Lennard Jones Potential
//             F_agent = -4*epsilon*( (6*pow(sigma, 6))/pow(d_agent, 7) - (12*pow(sigma, 12))/pow(d_agent, 13))*( (qk_p - qk_p_other)/(qk_p - qk_p_other).norm() );
//             // cout << "F_agent = " << F_agent.transpose(); cin.get();

//             // cout << "( (6*pow(sigma, 6))/pow(d_agent, 7) - (12*pow(sigma, 12))/pow(d_agent, 13)) = " <<  (6*pow(sigma, 6))/pow(d_agent, 7) - (12*pow(sigma, 12))/pow(d_agent, 13); cin.get();
            
//             F.block(0 + k*2, 0, 2, 1) = F_att + F_rep + F_agent;

//             if (d_goal < 0.001)
//             {
//                 F.block(0 + k*2, 0, 2, 1) = 0*F_att;
//             }
//         }
//         // cout << "F = " << F.transpose(); cin.get(); 

//         q.col(i+1) = Ae*q.col(i) + Be*F;
//     }

//     for (size_t i = 0; i < loopSize/40; i++)
//     {
//         q.col(i) = q.col(40*i);
//     }

//     MatrixXd Pr; Pr.setZero(4, loopSize/40);
//     MatrixXd Prd; Prd.setZero(4, loopSize/40);
    
//     // Agent 1
//     Pr.row(0) = q.block(0, 0, 1, loopSize/40);
//     Pr.row(1) = q.block(1, 0, 1, loopSize/40);
//     Prd.row(0) = q.block(2, 0, 1, loopSize/40);
//     Prd.row(1) = q.block(3, 0, 1, loopSize/40);

//     // Agent 2
//     Pr.row(2) = q.block(4, 0, 1, loopSize/40);
//     Pr.row(3) = q.block(5, 0, 1, loopSize/40);
//     Prd.row(2) = q.block(6, 0, 1, loopSize/40);
//     Prd.row(3) = q.block(7, 0, 1, loopSize/40);

//     // Pr_ = Pr;
//     // Prd_ = Prd;

//     // for (size_t i = 0; i < NAgents; i++)
//     // {
//     //     Pr.block(2*i,k+1,2,1) = q.block(4*i,k+1,2,1);
//     //     Prd.block(2*i,k+1,2,1) = q.block(4*i+2,k+1,2,1);
//     // }

//     // =================== PROVIDE CUSTOM REFERENCE =========================== //

//     // for (int i = 0; i < Pr.cols(); i++)
//     // {
//     //     // Pr.block(0, i, 4, 1) << 0 + 0.001*i, 0 - 0.001*i, 0 + 0.001*i, 0.001*i-1.5;
//     //     // Prd.block(0, i, 4, 1) << 0.05, -0.05, 0.05, 0.05; 
//     //     // Go straight - agent 1
//     //     double step_x1 = 0.005*2; double step_dx1 = 0.1*2;
//     //     double step_x2 = 0.005; double step_dx2 = 0.1;

//     //     Pr.block(0, i, 4, 1) << 0 + step_x1*(1-exp(-0.1*i))*i, 0 - step_x1*(1-exp(-0.1*i))*i, 0 + step_x1*(1-exp(-0.1*i))*i, + step_x1*(1-exp(-0.1*i))*i-0.75;
//     //     Prd.block(0, i, 4, 1) << step_dx1*(1-exp(-0.1*i)), -step_dx1*(1-exp(-0.1*i)), step_dx1*(1-exp(-0.1*i)), step_dx1*(1-exp(-0.1*i));
//     // }
//     // ======= END ======= PROVIDE CUSTOM REFERENCE =========== END =========== //


    

//     Pr_refined_ = Pr;
//     Prd_refined_ = Prd;
    
//     std::fstream myfile5;
//     myfile5.open("///home/hdsrl7/Desktop/A1_RaiSim_Outputs/RefTrajMPC_EXPE_" + std::to_string(log_ID) + ".txt",std::fstream::out);
//     myfile5 << Pr_refined_;
//     myfile5.close();

//     std::fstream myfile6;
//     myfile6.open("///home/hdsrl7/Desktop/A1_RaiSim_Outputs/VelTrajMPC_EXPE_" + std::to_string(log_ID) + ".txt",std::fstream::out);
//     myfile6 << Prd_refined_;
//     myfile6.close();


//     cout << "Can we reach here??????? LOCOWRAPPER"; cin.get();
// }


void LocoWrapper::setPstart(Eigen::Matrix<double, 2*NUMBER_OF_AGENTS, 1>& Pstart)
{
    Pstart_ << Pstart;
    agent_Initial_(0) = Pstart_(2*agent_id_);
    agent_Initial_(1) = Pstart_(2*agent_id_+1);
    mpc_state_alpha_buffer_<< agent_Initial_(0), 0, agent_Initial_(1), 0;
}

void LocoWrapper::printPstart()
{
    std::cout << "Pstart is:\n " << Pstart_ << std::endl;
}

void LocoWrapper::setPobs(Eigen::Matrix<double, 2,NUMBER_OF_OBS>& Pobs_in)
{
    Pobs << Pobs_in;
}

void LocoWrapper::printPobs()
{
    std::cout << "Pobs is:\n " << Pobs << std::endl;
}


void LocoWrapper::fitComTrajectory_eventbase(double M, double Ndomain, double N, double swingPhase){

    Eigen::Map<Eigen::MatrixXd> mpc_state_e_x_mtx(mpc_state_e_x_eventbased_.data(), 4, N);
    Eigen::MatrixXd mpc_state_e_x_mtxforalpha;
    Eigen::MatrixXd alpha_COM_traj_e;
    mpc_state_e_x_mtxforalpha.setZero(4*N/Ndomain, (Ndomain+1));
    alpha_COM_traj_e.setZero(4*N/Ndomain, (Ndomain+1));

    // std::cout << "DBG: mpc_state_alpha_buffer_\n"; printSize(mpc_state_alpha_buffer_);

    mpc_state_e_x_mtxforalpha.block(0,0,4,1) = mpc_state_alpha_buffer_;
    mpc_state_e_x_mtxforalpha.block(0,1,4,4) = mpc_state_e_x_mtx.block(0,0,4,4);
    for(size_t k = 1; k< N/Ndomain; k++){
        mpc_state_e_x_mtxforalpha.block(k*4,0,4,5)=mpc_state_e_x_mtx.block(0,k*4-1,4,5);
    }

    mpc_state_alpha_buffer_ = mpc_state_e_x_mtx.block(0,3,4,1);

    // Eigen::Map<Eigen::MatrixXd> mpc_state_e_x_mtx(com_desired_Traj_eventbased_.data(), 4, N);
    // Eigen::MatrixXd mpc_state_e_x_mtxforalpha;
    // Eigen::MatrixXd alpha_COM_traj_e;
    // mpc_state_e_x_mtxforalpha.setZero(4*N/Ndomain, (Ndomain+1));
    // alpha_COM_traj_e.setZero(4*N/Ndomain, (Ndomain+1));

    // std::cout << "DBG: mpc_state_alpha_buffer_\n"; printSize(mpc_state_alpha_buffer_);

    // mpc_state_e_x_mtxforalpha.block(0,0,4,1) = com_desired_Traj_eventbased_.block(0,0,4,1);
    // mpc_state_e_x_mtxforalpha.block(0,1,4,4) = mpc_state_e_x_mtx.block(0,0,4,4);
    // for(size_t k = 1; k< N/Ndomain; k++){
    //     mpc_state_e_x_mtxforalpha.block(k*4,0,4,5)=mpc_state_e_x_mtx.block(0,k*4-1,4,5);
    // }

    // mpc_state_alpha_buffer_ = mpc_state_e_x_mtx.block(0,3,4,1);

    

    // find alpha coefficient for future bezier polynomial in low level controller
    Eigen::MatrixXd binomialmtx_foronedomain;
    binomialmtx_foronedomain.setZero(4*(Ndomain+1), 4*(Ndomain+1));

    for(size_t i=0; i<Ndomain+1; i++){
        for(size_t j=0; j<Ndomain+1; j++){
            binomialmtx_foronedomain.block(i*4,j*4,4,4) = binomial(4,j,i*(1/Ndomain))*Eigen::MatrixXd::Identity(4,4);
        }
    }

    Eigen::MatrixXd eqconstraintmtx;
    Eigen::MatrixXd eqconstraintvec;
    Eigen::MatrixXd mpc_state_e_foronedomain;
    Eigen::MatrixXd alpha_e_vec_foronedomain;
    eqconstraintmtx.setZero(4*2, 4*(Ndomain+1));
    eqconstraintvec.setZero(4*2,1);

    mpc_state_e_foronedomain.setZero(4*5, 1);
    alpha_e_vec_foronedomain.setZero(4*5,1);
    
    eqconstraintmtx.block(0,0,4,4*(Ndomain+1))=binomialmtx_foronedomain.block(0,0,4,4*(Ndomain+1));
    eqconstraintmtx.block(4,0,4,4*(Ndomain+1))=binomialmtx_foronedomain.block(4*Ndomain,0,4,4*(Ndomain+1));

    for(size_t i=0; i<N/Ndomain; i++){
        eqconstraintvec.block(0,0,4,1) = mpc_state_e_x_mtxforalpha.block(i*4,0,4,1);
        eqconstraintvec.block(4,0,4,1) = mpc_state_e_x_mtxforalpha.block(i*4,4,4,1);
        mpc_state_e_foronedomain.block(0,0,4,1) = mpc_state_e_x_mtxforalpha.block(i*4,0,4,1);
        mpc_state_e_foronedomain.block(4,0,4,1) = mpc_state_e_x_mtxforalpha.block(i*4,1,4,1);
        mpc_state_e_foronedomain.block(8,0,4,1) = mpc_state_e_x_mtxforalpha.block(i*4,2,4,1);
        mpc_state_e_foronedomain.block(12,0,4,1) = mpc_state_e_x_mtxforalpha.block(i*4,3,4,1);
        mpc_state_e_foronedomain.block(16,0,4,1) = mpc_state_e_x_mtxforalpha.block(i*4,4,4,1);
        Eigen::MatrixXd QQQ;
        Eigen::MatrixXd PPP;

        QQQ.setZero(4*(Ndomain+1)+8, 4*(Ndomain+1)+8); //QQQ.setZero(4*(Ndomain+1)+4, 4*(Ndomain+1)+4);
        PPP.setZero(4*(Ndomain+1)+8,1);

        QQQ.block(0,0, 4*(Ndomain+1), 4*(Ndomain+1)) = binomialmtx_foronedomain.transpose() * binomialmtx_foronedomain;
        QQQ.block(0, 4*(Ndomain+1),4*(Ndomain+1), 8) = eqconstraintmtx.transpose();
        QQQ.block(4*(Ndomain+1), 0, 8, 4*(Ndomain+1)) = eqconstraintmtx;
        PPP.block(0,0, 4*(Ndomain+1),1) = binomialmtx_foronedomain.transpose()*mpc_state_e_foronedomain;
        PPP.block(4*(Ndomain+1), 0 , 8, 1) = eqconstraintvec;
        alpha_e_vec_foronedomain = (QQQ.inverse()*PPP).block(0,0, 4*(Ndomain+1),1);
        
        Eigen::Map<Eigen::MatrixXd> alpha_mtx_foronedomain(alpha_e_vec_foronedomain.data(), 4, 5);
        alpha_COM_traj_e.block(i*4,0,4,5) = alpha_mtx_foronedomain;
    }
    alpha_COM_traj_e_ = alpha_COM_traj_e;
}

void LocoWrapper::copPlanner_eventbase(double M, double Ndomain, double N, double swingPhase){
    // this function is used for generating desired COM in the MPC as prof style(newMPC branch in hdsrl_github)
    
    Eigen::Matrix<double, 4, 1> com_end;
    Eigen::Matrix<double, 4, 1> com_start_counter;
    Eigen::Matrix<double, 4, 1> com_end_counter;

    com_start.setZero();
    com_end.setZero();
    com_start_counter.setZero();
    com_end_counter.setZero();
    
    int gap2 = 0;
    for(size_t i=0; i<4; i++){
        if(footPrintTruncated_(i*2, 0) != swingPhase){
            com_start(0) = com_start(0) + footPrintTruncated_(i*2,0);
            com_start_counter(0) += 1;
        }

        if(footPrintTruncated_(i*2+1, 0) != swingPhase){
            com_start(2) = com_start(2) + footPrintTruncated_(i*2+1,0);
            com_start_counter(2) += 1;
        }
        if(1) {
            if(footPrintTruncated_(i*2, 1) != swingPhase){
                com_end(0) = com_end(0) + footPrintTruncated_(i*2,1);
                com_end_counter(0) += 1;
            }

            if(footPrintTruncated_(i*2+1, 1) != swingPhase){
                com_end(2) = com_end(2) + footPrintTruncated_(i*2+1,1);
                com_end_counter(2) += 1;
            }
        }
    } 

    com_start(0) = com_start(0)/com_start_counter(0);
    com_start(2) = com_start(2)/com_start_counter(2);
    // com_start(0) = state->q[0];//event base MPC - current com pos x
    // com_start(1) = 0; //event base MPC - current com vel x
    // com_start(2) = state->q[1]; //event base MPC - current com pos y
    // com_start(3) = 0; //event base MPC - current com vel y



    com_end(0) = com_end(0)/com_end_counter(0);
    com_end(2) = com_end(2)/com_end_counter(2);
    
    Eigen::MatrixXd com_desired_Traj;
    com_desired_Traj.setZero(4, N);
    com_desired_Traj.block(0,0, 1, N) = Eigen::VectorXd::LinSpaced(N, com_start(0), com_end(0)).transpose();
    com_desired_Traj.block(1,0, 1, N) = Eigen::VectorXd::LinSpaced(N, com_start(1), com_end(1)).transpose();
    com_desired_Traj.block(2,0, 1, N) = Eigen::VectorXd::LinSpaced(N, com_start(2), com_end(2)).transpose();
    com_desired_Traj.block(3,0, 1, N) = Eigen::VectorXd::LinSpaced(N, com_start(3), com_end(3)).transpose();

    Eigen::Vector2d com_P; com_P << state->q[0], state->q[1];
    Eigen::Vector2d goal_P; goal_P << GOAL_X, GOAL_Y;
    Eigen::Vector2d diff_G; diff_G.setZero(); diff_G << com_P - goal_P;
     
    // std::cout << "COM: " << state->q[0] << "    " << state->q[1] << "\n";  
    // std::cout << "NORM: " << diff_G.norm() << "\n";

    com_desired_Traj_eventbased_ = com_desired_Traj;
    Eigen::VectorXd com_desired_Traj_vec(Eigen::Map<Eigen::VectorXd>(com_desired_Traj.data(), com_desired_Traj.cols()*com_desired_Traj.rows()));
    com_desired_Traj_vec_eventbased_ = com_desired_Traj_vec;
    // cout << "In LOCO: com_desired_Traj: \n" << com_desired_Traj; cout << std::endl;    
}

void LocoWrapper::footstepPlanner_eventbase(double& M, double& Ndomain, double& N, double& mu, double& swingPhase, double& row_totalFootPrintGlobal, double& col_totalFootPrintGlobal){
    Eigen::Matrix<double, 2, 1> step_length;
    // std::cout << "In footstepPlanner_eventbase DBG\n";

    step_length << STEPLENGTHX, STEPLENGTHY;
    
    // THIS totalcycleindex generation part will be the gluing part when intergrating with the main control framework
    plannedCycleIndex(TROT); // total step num is defined in locomotion_controller.h
    //std::cout <<totalCycleIndex_<<std::endl;

    M = totalCycleIndex_.cols(); // number of domains - this var. is same with "TOTALSTEPNUM" in locomotion_controller.h
    Ndomain = NDOMAIN; // numer of grids per domain
    N = CTRL_HORIZ*NDOMAIN; // control horizon, commonly 2*Ndomain, if offlineMPC: M*Ndomain
    mu = 0.4; // friction coefficient
    swingPhase = HUGE_VAL; //notifying that leg is swing //HUGE_VAL = inf

    // row_totalFootPrintGlobal = Ndomain * M * 2; //same with m*N
    // col_totalFootPrintGlobal = Ndomain * (totalCycleIndex_.transpose()*totalCycleIndex_).diagonal().sum();

    // std::cout << "Line Number: 322\n";
    // std::cin >> tmp1; 

    Eigen::Matrix<double, 4, 1> currentCycleIndex;
    Eigen::Matrix<double, 8, 1> initialFootprint;
    Eigen::Matrix<double, 8 ,1> secondFootprint;
    Eigen::Matrix<double, 8, 1> fulltempFootprint;
    Eigen::MatrixXd tempFootprint;
    Eigen::MatrixXd tempFootprintmtx;
    Eigen::Matrix<double, 2, 1> swingPhaseprint;
    Eigen::Matrix<double, 8, 1> newFoothold;    
    Eigen::Matrix<double, 8, 1> rotatednewFoothold;

    Eigen::MatrixXd totalFootprint;
    Eigen::MatrixXd totalFootprintGlobal;
    Eigen::MatrixXd totalFootprintGlobalOnes;

    Eigen::MatrixXd cycleIndexTruncated;
    Eigen::MatrixXd footPrintTruncated;

    Eigen::Matrix<double,8,1> com_big;
    Eigen::Matrix<double, 2, 2> yawRotation;

    char tmp2;
    double com_x;
    double com_y;
    float yaw;

    newFoothold.setZero();
    currentCycleIndex.setZero();
    initialFootprint.setZero();
    secondFootprint.setZero();
    fulltempFootprint.setZero();
    swingPhaseprint << swingPhase, swingPhase;

    totalFootprint.setZero(2*totalCycleIndex_.rows(), totalCycleIndex_.cols());
 

    yawRotation.setZero(2,2);
    yawRotation << cos(0.1), -sin(0.1), sin(0.1), cos(0.1);
 
    // initialFootprint <<  0.3156,  0.2200,
    //                     -0.3444,  0.2200,
    //                      0.3156, -0.2200, 
    //                     -0.3444, -0.2200;

    initialFootprint <<  0.1830,  0.1320,
                        -0.1830,  0.1320,
                         0.1830, -0.1320, 
                        -0.1830, -0.1320;

    // apply yaw angle at initial footprint
    // KEYPOINT: first rotate the footprint without translation
    // after that, translate rotated footprint based on agent initial pos info
    // cout << "agent_Initial (loco)" << agent_Initial_ << std::endl;

    initialFootprint.block(0,0,2,1) = initialFootprint.block(0,0,2,1)+agent_Initial_;
    initialFootprint.block(2,0,2,1) = initialFootprint.block(2,0,2,1)+agent_Initial_;
    initialFootprint.block(4,0,2,1) = initialFootprint.block(4,0,2,1)+agent_Initial_;
    initialFootprint.block(6,0,2,1) = initialFootprint.block(6,0,2,1)+agent_Initial_;
    
    secondFootprint <<  swingPhase, swingPhase, 
                        initialFootprint(2), initialFootprint(3), 
                        initialFootprint(4), initialFootprint(5), 
                        swingPhase, swingPhase;


    totalFootprint.block(0,0,8,1) = initialFootprint;
    totalFootprint.block(0,1,8,1) = secondFootprint;

    int gap = 7;

    if(gaitDomain_ >= 0)
        M = 5;
    for(size_t k=2; k<M; k++){
    
        qref.setZero();
        qref.block(0,0,1,(gap+1)) = Pr_refined_.block(2*agent_id_,(gap+1)*(k-2),1,(gap+1));
        qref.block(2,0,1,(gap+1)) = Pr_refined_.block(2*agent_id_+1,(gap+1)*(k-2),1,(gap+1));

        // cout << "qref (LOCO):\n" << qref << std::endl; 
        
        newFoothold.setZero();
        newFoothold << qref(0,gap)+0.1830, qref(2,gap)+0.1320, qref(0,gap)-0.1830,qref(2,gap)+0.1320,qref(0,gap)+0.1830,qref(2,gap)-0.1320,qref(0,gap)-0.1830,qref(2,gap)-0.1320;     // A1 Params
 
        for(size_t i=0; i<4; i++){
            if(totalCycleIndex_(i,k) == 1){
                if(totalCycleIndex_(i,k-1)==1){
                    totalFootprint.block(2*i,k,2,1)=totalFootprint.block(2*i,k-1,2,1);
                }
                else{
                    totalFootprint.block(2*i,k,2,1)=newFoothold.block(2*i,0,2,1);
                }
            }
            else{
                totalFootprint.block(2*i,k,2,1)=swingPhaseprint;
            }
        }
    }
     M = totalCycleIndex_.cols();

    //  std::cout << "\n\n\nTotal number of Domains is:     " << M << "\n\n\n";
    //  std::cin >> tmp1;

    // cout << "NEW FOOTHOLD (LOCO): \n" << newFoothold << std::endl;  

    if(totalCycleIndex_.cols() % 2 == 0){
      totalFootprint.block(0,2,8,1) << initialFootprint(0)+step_length(0)*0.5, initialFootprint(1)+step_length(1)*0.5, 
                                        swingPhase, swingPhase, 
                                        swingPhase, swingPhase, 
                                        initialFootprint(6)+step_length(0)*0.5, initialFootprint(7)+step_length(1)*0.5;
  
        for(size_t k=3; k<M-1; k++){
        //for(size_t k=3; k<5; k++){

            qref.setZero();
            {
                qref.block(0,0,1,(gap+1)) = Pr_refined_.block(2*agent_id_,(gap+1)*(k-2),1,(gap+1));
                qref.block(2,0,1,(gap+1)) = Pr_refined_.block(2*agent_id_+1,(gap+1)*(k-2),1,(gap+1));
            }
  
            newFoothold.setZero();
            newFoothold << qref(0,gap)+0.1830, qref(2,gap)+0.1320, qref(0,gap)-0.1830,qref(2,gap)+0.1320,qref(0,gap)+0.1830,qref(2,gap)-0.1320,qref(0,gap)-0.1830,qref(2,gap)-0.1320; 
            // newFoothold << qref(0,gap)+0.33, qref(2,gap)+0.22, qref(0,gap)-0.33,qref(2,gap)+0.22,qref(0,gap)+0.33,qref(2,gap)-0.22,qref(0,gap)-0.33,qref(2,gap)-0.22;

            for(size_t i=0; i<4; i++){
                if(totalCycleIndex_(i,k) == 1){
                    if(totalCycleIndex_(i,k-1)==1){
                        totalFootprint.block(2*i,k,2,1)=totalFootprint.block(2*i,k-1,2,1);
                    }
                    else{
                        totalFootprint.block(2*i,k,2,1)=newFoothold.block(2*i,0,2,1);
                    }
                }
                else{
                    totalFootprint.block(2*i,k,2,1)=swingPhaseprint;
                }
            }            
        }

        qref.setZero();
        qref.block(0,0,1,(gap+1)) = Pr_refined_.block(2*agent_id_,(gap+1)*(M-1-2),1,(gap+1));
        qref.block(2,0,1,(gap+1)) = Pr_refined_.block(2*agent_id_+1,(gap+1)*(M-1-2),1,(gap+1));

        //cout << "qref (LOCO):\n" << qref << std::endl;

        newFoothold.setZero();
        newFoothold << qref(0,gap)+0.1830, qref(2,gap)+0.1320, qref(0,gap)-0.1830,qref(2,gap)+0.1320,qref(0,gap)+0.1830,qref(2,gap)-0.1320,qref(0,gap)-0.1830,qref(2,gap)-0.1320; 
        // newFoothold << qref(0,gap)+0.33, qref(2,gap)+0.22, qref(0,gap)-0.33,qref(2,gap)+0.22,qref(0,gap)+0.33,qref(2,gap)-0.22,qref(0,gap)-0.33,qref(2,gap)-0.22;

        for(size_t i=0; i<4; i++){
            if(totalCycleIndex_(i,M-1) == 1){
                if(totalCycleIndex_(i,M-1-1)==1){
                    totalFootprint.block(2*i,M-1,2,1)=totalFootprint.block(2*i,M-1-1,2,1);
                }
                else{
                    totalFootprint.block(2*i,M-1,2,1)=newFoothold.block(2*i,0,2,1);
                }
            }
            else{
                totalFootprint.block(2*i,M-1,2,1)=swingPhaseprint;
            }
        }
    }

    // From Jeeseop's Code --- Rotate Continuously 
    // for(size_t i=2; i< totalFootprint.cols(); i++){
    //     for(size_t j=0; j<4; j++){
    //         if(totalFootprint.block(2*j,i,2,1) != swingPhaseprint){
    //             Eigen::Matrix<double, 2, 2> step_length_yaw_rot_foroneleg;
    //             step_length_yaw_rot_foroneleg << cos(body_Y_*(i)), -sin(body_Y_*(i)), sin(body_Y_*(i)), cos(body_Y_*(i));
    //             //if(i == 2 || i == totalFootprint.cols()-1){
    //             //    step_length_yaw_rot_foroneleg << cos(0.5*BODYYAW*(i-1)), -sin(0.5*BODYYAW*(i-1)), sin(0.5*BODYYAW*(i-1)), cos(0.5*BODYYAW*(i-1));
    //             //}
    //             totalFootprint.block(2*j,i,2,1) = step_length_yaw_rot_foroneleg*totalFootprint.block(2*j,i,2,1);
    //         }
    //     }
    // }

    //truncating the matrix when gait number is increasing: maybe using gaitDomain_
    cycleIndexTruncated = totalCycleIndex_;
    footPrintTruncated = totalFootprint;

    // cout << "cycleIndexTruncated (LOCO):   \n" << cycleIndexTruncated << std:: endl;
    // cout << "totalFootPrint:   \n" << totalFootprint.block(0,0,8,8) << std::endl;


    cycleIndexTruncated.block(0,0, totalCycleIndex_.rows(), totalCycleIndex_.cols()-gaitDomain_) = totalCycleIndex_.block(0,gaitDomain_, totalCycleIndex_.rows(), totalCycleIndex_.cols()-gaitDomain_);
    footPrintTruncated.block(0,0, totalFootprint.rows(), totalFootprint.cols()-gaitDomain_) = totalFootprint.block(0, gaitDomain_, totalFootprint.rows(), totalFootprint.cols()-gaitDomain_);

    if(gaitDomain_ !=0){    
        for(size_t i = 0; i< gaitDomain_; i++){
            cycleIndexTruncated.block(0, totalCycleIndex_.cols()-gaitDomain_+i, totalCycleIndex_.rows(), 1) = totalCycleIndex_.block(0,totalCycleIndex_.cols()-1,totalCycleIndex_.rows(), 1);
            footPrintTruncated.block(0, totalFootprint.cols()-gaitDomain_+i, totalFootprint.rows(), 1) = totalFootprint.block(0, totalFootprint.cols()-1, totalFootprint.rows(), 1); 
        }
    }
    //std::cout << "GAIT DOMAIN: " << gaitDomain_<< std::endl;
    //std::cout << cycleIndexTruncated <<std::endl;
    //std::cout << footPrintTruncated <<std::endl;

    row_totalFootPrintGlobal = Ndomain * M * 2; //same with m*(M * Ndomain)
    col_totalFootPrintGlobal = Ndomain * (cycleIndexTruncated.transpose()*cycleIndexTruncated).diagonal().sum();
    size_t fpg_row = 0;
    size_t fpg_col = 0;
    totalFootprintGlobal.setZero(row_totalFootPrintGlobal, col_totalFootPrintGlobal);
    totalFootprintGlobalOnes.setZero(row_totalFootPrintGlobal, col_totalFootPrintGlobal);

    //for(size_t dom=0; dom< M; dom++){
    for(size_t dom=0; dom< 5; dom++){                               // ------------------------------------------- <<<<<<<<<<<<<<<<<<<<<<<< dom = 5

        currentCycleIndex = cycleIndexTruncated.block(0, dom, 4, 1);
        double m_contact = currentCycleIndex.transpose().dot(currentCycleIndex);
        //fulltempFootprint = totalFootprint.block(0,dom,8,1);
        fulltempFootprint = footPrintTruncated.block(0,dom,8,1); //event base mpc
        if(m_contact == 4){
            tempFootprint.setZero(2*m_contact, 1);
            tempFootprintmtx.setZero(2, m_contact);
            tempFootprint = fulltempFootprint;
            tempFootprintmtx = Eigen::Map< Eigen::Matrix<double,2, 4, Eigen::ColMajor> > (tempFootprint.data());
            for(size_t i = 0; i<Ndomain; i++){
                totalFootprintGlobal.block(fpg_row, fpg_col, 2, 4)=tempFootprintmtx;
                totalFootprintGlobalOnes.block(fpg_row, fpg_col, 2, 4)=Eigen::MatrixXd::Ones(2,4);
                fpg_row += 2;
                fpg_col += 4;
            }

        }
        else if(m_contact == 3){
            std::cout << "other gait except trot is not ready" << std::endl;
        }
        else if(m_contact == 2){
            tempFootprint.setZero(2*m_contact, 1);
            tempFootprintmtx.setZero(2, m_contact);
            size_t tempFootprint_index = 0;
            for(size_t i=0; i<8; i++){
                if(fulltempFootprint(i) != swingPhase){
                    tempFootprint(tempFootprint_index) = fulltempFootprint(i);
                    tempFootprint_index++;
                }
            }
            tempFootprintmtx = Eigen::Map< Eigen::Matrix<double,2, 2, Eigen::ColMajor> > (tempFootprint.data());
            for(size_t i = 0; i<Ndomain; i++){
                totalFootprintGlobal.block(fpg_row, fpg_col, 2, 2)=tempFootprintmtx;
                totalFootprintGlobalOnes.block(fpg_row, fpg_col, 2, 2)=Eigen::MatrixXd::Ones(2,2);
                fpg_row += 2;
                fpg_col += 2;
            }
        }
        else if(m_contact == 1){
            std::cout << "dynamic gait is not ready" << std::endl;
        }
        else{
            std::cout << "flying?" << std::endl;
        }
        //std::cout << tempFootprintmtx << std::endl;
        
    }

    totalFootprint_ = totalFootprint; // pure foot print data
    cycleIndexTruncated_ = cycleIndexTruncated;

    footPrintTruncated_ = footPrintTruncated;
    footPrintGlobalTruncated_ = totalFootprintGlobal; // foot print data expanded to the grid based matrix
    footPrintGlobalOnesTruncated_ = totalFootprintGlobalOnes;
    
    if(agent_id_ == 0)
            std::cout << "Domain: " << domain_++ << std::endl;

    // std::fstream myfile;
    // myfile.open("/home/hdsrl7/Documents/total_footprint.txt",std::fstream::out);
    // myfile << totalFootprint;
    // myfile.close();    
}

void LocoWrapper::getComTrajectoryEventbase(size_t gait, size_t gaitDomain){
    const size_t rownum = 4*TOTALSTEPNUM; //totalstepnumber * 4

    if(gait == STAND){
        comTraj_[0] = 0;
        comTraj_[1] = 0;
        dcomTraj_[0] = 0;
        dcomTraj_[1] = 0;
        ddcomTraj_[0] = 0;
        ddcomTraj_[1] = 0;

        comOriTraj_[0] = 0;
        comOriTraj_[1] = 0;
        comOriTraj_[2] = 0;
        dcomOriTraj_[0] = 0;
        dcomOriTraj_[1] = 0;
        dcomOriTraj_[2] = 0;
        ddcomOriTraj_[0] = 0;
        ddcomOriTraj_[1] = 0;
        ddcomOriTraj_[2] = 0;

        if(gaitDomain >= totalStepNum_-1){
            gaitDomain = totalStepNum_-1;
            Eigen::Matrix<double, 2, 5> alpha_COM_traj_e_current_pos;
            Eigen::Matrix<double, 2, 5> alpha_COM_traj_e_current_vel;

            // cout << "GET COM DBG 2\n";
            
            alpha_COM_traj_e_current_pos.block<1,5>(0,0) = alpha_COM_traj_e_.block(0,0,1,5);
            alpha_COM_traj_e_current_pos.block<1,5>(1,0) = alpha_COM_traj_e_.block(2,0,1,5);
            alpha_COM_traj_e_current_vel.block<1,5>(0,0) = alpha_COM_traj_e_.block(1,0,1,5);
            alpha_COM_traj_e_current_vel.block<1,5>(1,0) = alpha_COM_traj_e_.block(3,0,1,5);

            // cout << "GET COM DBG 3\n";

            hzd_dmat* alpha_COM_traj_dmat_pos = new hzd_dmat;
            hzd_dmat* alpha_COM_traj_dmat_vel = new hzd_dmat;
            alpha_COM_traj_dmat_pos->pr = new double[alpha_COM_traj_e_current_pos.rows()*alpha_COM_traj_e_current_pos.cols()];
            alpha_COM_traj_dmat_vel->pr = new double[alpha_COM_traj_e_current_vel.rows()*alpha_COM_traj_e_current_vel.cols()];

            Eigen_mtx_to_hzd_dmat(alpha_COM_traj_e_current_pos, alpha_COM_traj_dmat_pos);
            Eigen_mtx_to_hzd_dmat(alpha_COM_traj_e_current_vel, alpha_COM_traj_dmat_vel);

            HZD_bezier(alpha_COM_traj_dmat_pos, phaseVar, comTraj_); //TODO// just interpolating from the MPC result
            HZD_bezier(alpha_COM_traj_dmat_vel, phaseVar, dcomTraj_); //TODO// little bit weird interpolating. Do it more logically

            delete[] alpha_COM_traj_dmat_pos->pr;
            delete[] alpha_COM_traj_dmat_vel->pr;
            delete alpha_COM_traj_dmat_pos;
            delete alpha_COM_traj_dmat_vel;

            comOriTraj_[0] = cubic(locoTick, 0, gridNum_*ts_OptTick_,body_R_*(gaitDomain-1),body_R_*(gaitDomain-1),0,0);
            comOriTraj_[1] = cubic(locoTick, 0, gridNum_*ts_OptTick_,body_P_*(gaitDomain-1),body_P_*(gaitDomain-1),0,0);
            comOriTraj_[2] = cubic(locoTick, 0, gridNum_*ts_OptTick_,body_Y_*(gaitDomain-1),body_Y_*(gaitDomain-1),0,0);
            dcomOriTraj_[0] = cubicDot(locoTick, 0, gridNum_*ts_OptTick_,body_R_*(gaitDomain-1),body_R_*(gaitDomain-1),0,0);
            dcomOriTraj_[1] = cubicDot(locoTick, 0, gridNum_*ts_OptTick_,body_P_*(gaitDomain-1),body_P_*(gaitDomain-1),0,0);
            dcomOriTraj_[2] = cubicDot(locoTick, 0, gridNum_*ts_OptTick_,body_Y_*(gaitDomain-1),body_Y_*(gaitDomain-1),0,0);
            ddcomOriTraj_[0] = cubicDotDot(locoTick, 0, gridNum_*ts_OptTick_,body_R_*(gaitDomain-1),body_R_*(gaitDomain-1),0,0);
            ddcomOriTraj_[1] = cubicDotDot(locoTick, 0, gridNum_*ts_OptTick_,body_P_*(gaitDomain-1),body_P_*(gaitDomain-1),0,0);
            ddcomOriTraj_[2] = cubicDotDot(locoTick, 0, gridNum_*ts_OptTick_,body_Y_*(gaitDomain-1),body_Y_*(gaitDomain-1),0,0);
        
        }
    }
    
    if(gait != STAND){

        Eigen::Matrix<double, 2, 5> alpha_COM_traj_e_current_pos;
        Eigen::Matrix<double, 2, 5> alpha_COM_traj_e_current_vel;
        alpha_COM_traj_e_current_pos.block<1,5>(0,0) = alpha_COM_traj_e_.block(0,0,1,5);
        alpha_COM_traj_e_current_pos.block<1,5>(1,0) = alpha_COM_traj_e_.block(2,0,1,5);
        alpha_COM_traj_e_current_vel.block<1,5>(0,0) = alpha_COM_traj_e_.block(1,0,1,5);
        alpha_COM_traj_e_current_vel.block<1,5>(1,0) = alpha_COM_traj_e_.block(3,0,1,5);
        hzd_dmat* alpha_COM_traj_dmat_pos = new hzd_dmat;
        hzd_dmat* alpha_COM_traj_dmat_vel = new hzd_dmat;
        alpha_COM_traj_dmat_pos->pr = new double[alpha_COM_traj_e_current_pos.rows()*alpha_COM_traj_e_current_pos.cols()];
        alpha_COM_traj_dmat_vel->pr = new double[alpha_COM_traj_e_current_vel.rows()*alpha_COM_traj_e_current_vel.cols()];

        Eigen_mtx_to_hzd_dmat(alpha_COM_traj_e_current_pos, alpha_COM_traj_dmat_pos);
        Eigen_mtx_to_hzd_dmat(alpha_COM_traj_e_current_vel, alpha_COM_traj_dmat_vel);

        if(MPC_data_available){ 
            HZD_bezier(alpha_COM_traj_dmat_pos, phaseVar, comTraj_); //TODO// just interpolating from the MPC result
            HZD_bezier(alpha_COM_traj_dmat_vel, phaseVar, dcomTraj_); //TODO// little bit weird interpolating. Do it more logically
            HZD_bezierd(alpha_COM_traj_dmat_vel, phaseVar, ddcomTraj_); //TODO// little bit weird interpolating. Do it more logically
        }
        else
        {
            HZD_bezier(alpha_COM_traj_dmat_pos, 1.05, comTraj_); //TODO// just interpolating from the MPC result
            HZD_bezier(alpha_COM_traj_dmat_vel, 1.05, dcomTraj_); //TODO// little bit weird interpolating. Do it more logically
            HZD_bezierd(alpha_COM_traj_dmat_vel, 1.05, ddcomTraj_); //TODO// little bit weird interpolating. Do it more logically
            // cout << "comx " << comTraj_[0] << "\n"; 
        }

        delete[] alpha_COM_traj_dmat_pos->pr;
        delete[] alpha_COM_traj_dmat_vel->pr;
        delete alpha_COM_traj_dmat_pos;
        delete alpha_COM_traj_dmat_vel;

        comOriTraj_[0] = cubic(locoTick, 0, gridNum_*ts_OptTick_,body_R_*(gaitDomain-1),body_R_*gaitDomain,0,0);
        comOriTraj_[1] = cubic(locoTick, 0, gridNum_*ts_OptTick_,body_P_*(gaitDomain-1),body_P_*gaitDomain,0,0);
        comOriTraj_[2] = cubic(locoTick, 0, gridNum_*ts_OptTick_,body_Y_*(gaitDomain-1),body_Y_*gaitDomain,0,0);
        dcomOriTraj_[0] = cubicDot(locoTick, 0, gridNum_*ts_OptTick_,body_R_*(gaitDomain-1),body_R_*gaitDomain,0,0);
        dcomOriTraj_[1] = cubicDot(locoTick, 0, gridNum_*ts_OptTick_,body_P_*(gaitDomain-1),body_P_*gaitDomain,0,0);
        dcomOriTraj_[2] = cubicDot(locoTick, 0, gridNum_*ts_OptTick_,body_Y_*(gaitDomain-1),body_Y_*gaitDomain,0,0);
        ddcomOriTraj_[0] = cubicDotDot(locoTick, 0, gridNum_*ts_OptTick_,body_R_*(gaitDomain-1),body_R_*gaitDomain,0,0);
        ddcomOriTraj_[1] = cubicDotDot(locoTick, 0, gridNum_*ts_OptTick_,body_P_*(gaitDomain-1),body_P_*gaitDomain,0,0);
        ddcomOriTraj_[2] = cubicDotDot(locoTick, 0, gridNum_*ts_OptTick_,body_Y_*(gaitDomain-1),body_Y_*gaitDomain,0,0);

        // cout << "GET COM DBG 12\n";
    }
}

void LocoWrapper::printSize(const Eigen::MatrixXd& mat)
{
    cout << "\nRows(): " << mat.rows() << "   cols(): " << mat.cols() << std::endl;
    cout << mat << std::endl;
} 

void LocoWrapper::set_MPC_DATA(Eigen::MatrixXd alpha_COM, Eigen::MatrixXd MPC_sol, bool avail){
    alpha_COM_traj_e_.setZero(4, 5);
    alpha_COM_traj_e_ = alpha_COM;
    mpc_state_e_x_eventbased_ = MPC_sol;
    MPC_data_available = avail;
};

void LocoWrapper::reset_MPC_DATA()
{
    // DO I NEED TO DO ANYTHING WITH THE alpha_COM_traj_e_ ? or HOW TO ENFORCE LAST known COM positions before MPC solution becomes available? 
}