// Author: Basit Muhammad Imran
// Date: 01/20/2022

#include "MPC_dist.hpp"
#include <fstream>
#include "RobotModel.hpp"
#include <chrono>
#include <cstdlib> // For getenv
#include <string>  // For string manipulation
#include <filesystem>


using namespace ifopt;

double MPC_dist::calculateDistance(const Eigen::VectorXd& point2) {
    Eigen::Vector2d point1; point1 << q[0], q[1];
    return std::sqrt((point1(0) - point2(0)) * (point1(0) - point2(0)) + 
                     (point1(1) - point2(1)) * (point1(1) - point2(1)));
}

void MPC_dist::updateDistance_to_fail() {
    if (isSuccess) {
        for (int i = 0; i < Pobs_real.cols(); ++i) {
            Eigen::VectorXd obstacle = Pobs_real.col(i);
            double distance = calculateDistance(obstacle);
            // std::cout << "Dist to obs = " << distance << std::endl;

            if (distance < 0.5) {
                isSuccess = false;
                distance_to_fail = calculateDistance(Eigen::VectorXd::Zero(2)); // Distance to (0, 0)
                std::cout << "\033[1;31m" << "DISTANCE TO FAIL:  " << "\033[0m" << distance_to_fail << std::endl;
                break;
            }
        }
    }
    else
    {
        std::cout << "\033[1;31m" << "BYPASSING - DISTANCE TO FAIL:  " << "\033[0m" << distance_to_fail << std::endl;
    }
}

MPC_dist::MPC_dist() // inside arguments (const StateInfo* st)
{   
    static int agent_counter = 0;
    log_ID = agent_counter;
    agent_counter++;

    if(agent_counter == 2)
        agent_counter = 0;

    distance_to_fail = 10;
    ts_OptTick_ = TSOPTTICK;
    mpc_state_alpha_buffer_.setZero(4,1);
    domain_ = 0;
    gaitDomain_ = 0;
    onegaitCycle_ = 4; //const
    gridNum_ = NDOMAIN;
    agent_id_ = 0;  // CHANGE FOR MULTIPLE AGENTS
    totalStepNum_ = TOTALSTEPNUM;
    // state = st;                     // Update the pointer to point to the QuadModels state

    comTraj_[2] = {0};
    dcomTraj_[2] = {0};
    ddcomTraj_[2] = {0};

     ///home/hdsrl7/Desktop/A1_RaiSim_Outputs/" + std::to_string(log_ID) + ".txt"
    std::string mpc_log_filename = "///home/hdsrl7/Desktop/A1_RaiSim_Outputs/MPC_LOG_AGENT_" + std::to_string(log_ID) + ".txt";
    std::string qp_log_filename = "///home/hdsrl7/Desktop/A1_RaiSim_Outputs/QP_LOG_AGENT_" + std::to_string(log_ID) + ".txt";
    std::string obs_dist_file_name = "///home/hdsrl7/Desktop/A1_RaiSim_Outputs/obs_dist_" + std::to_string(log_ID) + ".txt";
    std::string agent_dist_file_name = "///home/hdsrl7/Desktop/A1_RaiSim_Outputs/agent_dist_" + std::to_string(log_ID) + ".txt";
    mpc_file = std::fstream(mpc_log_filename, std::ios::out);
    qp_file = std::fstream(qp_log_filename, std::ios::out);

    obs_dist_file = std::fstream(obs_dist_file_name, std::ios::out);
    agent_dist_file = std::fstream(agent_dist_file_name, std::ios::out);
    // mpc_file.open(); qp_file.open();
}

//MPC_dist::~MPC_dist() {};

void MPC_dist::lipMPC_eventbase() {
    double M;
    double Ndomain;
    double N;
    double mu;
    double swingPhase;
    double row_totalFootPrintGlobal; //same with m*N
    double col_totalFootPrintGlobal;

    if (agent_id_ == 0) { mu = 0.7; } else { mu = 0.7; }

    M = TOTALSTEPNUM; Ndomain = 4; N = 4; swingPhase = inf; swingPhase = HUGE_VAL;

    // cout << "Can we reach here?"; cin.get();
    copPlanner_eventbase(M, Ndomain, N, swingPhase);
    // cout << "Can we reach here?  1"; cin.get();
    footholdsPlanner();

    double g = 9.81;
    double h = 0.29;

    //if (agent_id_ == 0) { h = 0.2745; }

    double Ts_opt = ts_OptTick_*0.001;
    // continuous dynamics
    Eigen::Matrix<double, 4, 4> A;
    Eigen::Matrix<double, 4, 2> B;
    // discrete dynamics
    Eigen::Matrix<double, 4, 4> Ad;
    Eigen::Matrix<double, 4, 2> Bd;

    A.setZero(4,4);
    B.setZero(4,2);
    Ad.setZero(4,4);
    Bd.setZero(4,2);

    A(0,1) = 1;
    A(1,0) = g/h;
    A(2,3) = 1;
    A(3,2) = g/h;
    B(1,0) = -g/h;
    B(3,1) = -g/h;

    
    // Try changing it and seeing what happens
    Ad = Eigen::MatrixXd::Identity(4,4)+A*Ts_opt+0.5*A*A*Ts_opt*Ts_opt+A*A*A*Ts_opt*Ts_opt*Ts_opt/6;
    Bd = A.inverse()*(Ad-Eigen::MatrixXd::Identity(4,4))*B;

    double n = A.cols(); // vector size of the state: (x, xdot, y, ydot)
    double m = B.cols(); // vector size of the control input: (x_cop, y_cop)

    int contacts = contactInd[0]+contactInd[1]+contactInd[2]+contactInd[3];//= (domain_ == 0) ? 4 : 2;
    col_totalFootPrintGlobal = Ndomain*contacts;
    int use_slack = 1;
    double num_dec_vars = n*N + m*N + Ndomain*contacts + use_slack; // mod_col_totalFootPrintGlobal; + 1 for the slack variable
    

    Eigen::MatrixXd Q; // point state cost matrix
    Eigen::MatrixXd R; // point control cost matrix
    Eigen::MatrixXd P; // final cost matrix

    Eigen::MatrixXd Pbig;
    Eigen::MatrixXd Rbig;

    Eigen::MatrixXd Q_qp;
    Eigen::MatrixXd f_qp;
    Eigen::MatrixXd f_qpt;

    Eigen::MatrixXd Abig;
    Eigen::MatrixXd Bbig;
    Eigen::MatrixXd idntymtxA;
    Eigen::MatrixXd idntymtxB;
    Eigen::MatrixXd Abigx0;
    //Eigen::Matrix<double, 4, 1> x0;
    Eigen::Vector4d x0;

    Eigen::MatrixXd Aeq;
    Eigen::MatrixXd beq;
    Eigen::MatrixXd Gineq;
    Eigen::MatrixXd hineq;

    //QP formulation
    //// cost function
    Q.setIdentity(n, n); Q(0,0) = 1; Q(1,1) = 1; Q(2,2) = 1; Q(3,3) = 1; 
    R.setIdentity(m, m);
    P.setIdentity(n, n);

    double sl_gain = 0.0;
    // mpcgains
    if (1) //agent_id_ == 1
    {
        Q = 3e2 * Q;
        R = 1e-1 * R;
        P = 2e3 * P;
        sl_gain = 0.3e4; //sl_gain = 0.6e3;

        // sl_gain = 0.6e5;      // For RObot to robot
    }
    else{
        Q = 6e5 * Q;
        R = 1e-1 * R;
        P = 9e6 * P;
        sl_gain = 0.5e5;
        // Q = 2e1 * Q;
        // R = 1e-1 * R;
        // P = 2e2 * P;
        // sl_gain = 0.1e3;
    }

    Pbig.setZero(n*N, n*N);
    Rbig.setZero(m*N, m*N);

    for(size_t i=0; i<N; i++){
        Pbig.block(n*i, n*i, n,n) = Q;
        Rbig.block(m*i, m*i, m,m) = R;
    }
    Pbig.block(n*(N-1), n*(N-1), n,n) = P;

    Q_qp.setZero(num_dec_vars, num_dec_vars);
    f_qpt.setZero(1, num_dec_vars);
    f_qp.setZero(num_dec_vars, 1);

    Q_qp.block(0,0, n*N, n*N) = Pbig;
    Q_qp.block(n*N, n*N, m*N, m*N) = Rbig;
    if (use_slack) { Q_qp.block(Q_qp.rows()-1, Q_qp.cols()-1, 1, 1) << sl_gain; } //Defect variable slackcost

    // std::cout << "com_desired_Traj_vec: \n" << com_desired_Traj_vec_eventbased_ << "\n"; cin.get();

    f_qpt.block(0,0, 1, n*N) = -com_desired_Traj_vec_eventbased_.transpose()*Pbig; //2* should not be in here!!!
    f_qp = f_qpt.transpose();

    //// constraints
    //////// equality constraints
    Abig.setZero(n*N, n*N);
    Bbig.setZero(n*N, m*N);
    idntymtxA.setIdentity(n*N, n*N);
    idntymtxB.setIdentity(m*N, m*N);
    Abigx0.setZero(n*N, n);

    // if (domain_ < 1) {
    //     x0(0) = 0; //event base MPC - current com pos x
    //     x0(1) = 0.05; //event base MPC - current com vel x
    //     x0(2) = 0; //event base MPC - current com pos y
    //     x0(3) = 0; //event base MPC - current com vel y
    // } else {
        x0(0) = q[0]; //event base MPC - current com pos x
        x0(1) = dq[0]; //event base MPC - current com vel x
        x0(2) = q[1]; //event base MPC - current com pos y
        x0(3) = dq[1]; //event base MPC - current com vel y
    // }

    for(size_t i=0; i<N-1; i++){
        Abig.block((i+1)*n, i*n, n, n) = Ad;
        Bbig.block(i*n, i*m, n, m) = Bd;
    }
    Bbig.block((N-1)*n, (N-1)*m, n, m) = Bd;
    Abigx0.block(0, 0, n, n) = Ad;


    //CHECK// lambda summation = 1 equality constraint is not repeated if we use totalFootprintGlobalOnes_half
    Eigen::MatrixXd totalFootprintGlobalOnes_half; int rowTmp = footPrintGlobalOnesTruncated_.rows(); int colTmp = footPrintGlobalOnesTruncated_.cols();
    totalFootprintGlobalOnes_half.setZero(rowTmp, colTmp);
    totalFootprintGlobalOnes_half.block(0, 0, 1, colTmp) =  footPrintGlobalOnesTruncated_.block(0, 0, 1, colTmp);
    totalFootprintGlobalOnes_half.block(1, 0, 1, colTmp) =  footPrintGlobalOnesTruncated_.block(2, 0, 1, colTmp);
    totalFootprintGlobalOnes_half.block(2, 0, 1, colTmp) =  footPrintGlobalOnesTruncated_.block(4, 0, 1, colTmp);
    totalFootprintGlobalOnes_half.block(3, 0, 1, colTmp) =  footPrintGlobalOnesTruncated_.block(6, 0, 1, colTmp);

    // cout << "totalFootprintGlobalOnes_half.block(0,0, m*N*0.5, Ndomain*contacts)\n" << totalFootprintGlobalOnes_half.block(0,0, m*N*0.5, Ndomain*contacts); cin.get();
    Aeq.setZero(n*N + m*N + m*N*0.5, num_dec_vars);
    beq.setZero(n*N + m*N + m*N*0.5, 1);
    
    Aeq.block(0, 0, n*N, n*N) = Abig - idntymtxA;
    Aeq.block(0,n*N, n*N, m*N) = Bbig;
    
    Aeq.block(n*N, n*N, m*N, m*N) = idntymtxB;
    //Aeq.block(n*N, n*N+m*N, row_totalFootPrintGlobal, col_totalFootPrintGlobal) = -footPrintGlobalTruncated_;
    Aeq.block(n*N, n*N+m*N, m*N, Ndomain*contacts) = -footPrintGlobalTruncated_.block(0,0, m*N, Ndomain*contacts);
    Aeq.block(n*N + m*N, n*N + m*N, m*N*0.5, Ndomain*contacts) = totalFootprintGlobalOnes_half.block(0,0, m*N*0.5, Ndomain*contacts);
    
    beq.block(0,0,n*N,1) = -Abigx0*x0;
    beq.block(n*N+m*N, 0, m*N*0.5, 1) = Eigen::MatrixXd::Ones(m*N*0.5,1);

    // //////// inequality constraints
    Eigen::MatrixXd Gsubx;
    Eigen::MatrixXd Gsubxdot;
    Eigen::MatrixXd Gsubu;
    Eigen::MatrixXd xportion;
    Eigen::MatrixXd xdotportion;
    Eigen::MatrixXd uportion;
    xportion.setZero(2,4);
    xdotportion.setZero(2,4);
    uportion.setIdentity(2,2);
    xportion << 1,0,0,0,
                0,0,1,0;
    xdotportion << 0,1,0,0,
                   0,0,0,1;

    Gsubx.setZero(m*(N-1), n*N);
    Gsubu.setZero(m*(N-1), m*N);
    for(size_t i=0; i<N-1; i++){
        Gsubx.block(i*m, i*n, m, n) = xportion;
        Gsubu.block(i*m, (i+1)*m, m, m) = uportion;
    }

    //CHECK// add boundaries for every decision variable + reorder ineq. constraints//
    Gineq.setZero(2*m*(N-1) + 2*n*N+2*m*N + 2*Ndomain*contacts, num_dec_vars); // +1 for my test inequality
    hineq.setOnes(2*m*(N-1) + 2*n*N+2*m*N + 2*Ndomain*contacts, 1);                // +1 for added ineq

    Gineq.block(0,0, m*(N-1) , n*N) = Gsubx;
    Gineq.block(0,n*N, m*(N-1), m*N) = -Gsubu;
    Gineq.block(m*(N-1), 0, m*(N-1) , n*N) = -Gsubx;
    Gineq.block(m*(N-1), n*N, m*(N-1), m*N) = Gsubu;

    Gineq.block(2*m*(N-1), 0, n*N, n*N) = Eigen::MatrixXd::Identity(n*N, n*N);
    Gineq.block(2*m*(N-1) +n*N, 0, n*N, n*N) = -Eigen::MatrixXd::Identity(n*N, n*N);
    Gineq.block(2*m*(N-1) +2*n*N, n*N, m*N, m*N) = Eigen::MatrixXd::Identity(m*N, m*N);
    Gineq.block(2*m*(N-1) +2*n*N+ m*N, n*N, m*N, m*N) = -Eigen::MatrixXd::Identity(m*N, m*N);

    Gineq.block(2*m*(N-1) + 2*n*N+2*m*N, n*N+m*N, Ndomain*contacts, Ndomain*contacts) = -Eigen::MatrixXd::Identity(Ndomain*contacts, Ndomain*contacts);
    Gineq.block(2*m*(N-1) + 2*n*N+2*m*N + Ndomain*contacts, n*N+m*N, Ndomain*contacts, Ndomain*contacts) = Eigen::MatrixXd::Identity(Ndomain*contacts, Ndomain*contacts);

    // Adding this --- test
    // Eigen::VectorXd myInq; myInq.setZero(num_dec_vars); myInq(0) = 1; myInq(2) = 2;
    // Eigen::VectorXd myInq1; myInq1.setZero(num_dec_vars); myInq1(0+4) = 1; myInq1(2+4) = 2; 
    // Eigen::VectorXd myInq2; myInq2.setZero(num_dec_vars); myInq2(0+8) = 1; myInq2(2+8) = 2; 
    // Eigen::VectorXd myInq3; myInq3.setZero(num_dec_vars); myInq3(0+12) = 1; myInq3(2+12) = 2; 

    // Gineq.block(2*m*(N-1) + 2*n*N+2*m*N + 2*Ndomain*contacts, 0, 1, num_dec_vars) = myInq.transpose();
    // Gineq.block(2*m*(N-1) + 2*n*N+2*m*N + 2*Ndomain*contacts + 1, 0, 1, num_dec_vars) = myInq1.transpose();
    // Gineq.block(2*m*(N-1) + 2*n*N+2*m*N + 2*Ndomain*contacts + 2, 0, 1, num_dec_vars) = myInq2.transpose();
    // Gineq.block(2*m*(N-1) + 2*n*N+2*m*N + 2*Ndomain*contacts + 3, 0, 1, num_dec_vars) = myInq3.transpose();

    // --------------------

    hineq.block(0,0,2*m*(N-1),1) = (mu*h/sqrt(2))*Eigen::MatrixXd::Ones(2*m*(N-1),1);

    hineq.block(2*m*(N-1), 0,2*n*N,1) = 1e3 * Eigen::MatrixXd::Ones(2*n*N,1);
    hineq.block(2*m*(N-1) +2*n*N, 0, 2*m*N, 1) = 1e3 * Eigen::MatrixXd::Ones(2*m*N, 1);

    hineq.block(2*m*(N-1) + 2*n*N+2*m*N, 0, Ndomain*contacts, 1) = 0*Eigen::MatrixXd::Ones(Ndomain*contacts, 1);
    hineq.block(2*m*(N-1) + 2*n*N+2*m*N+Ndomain*contacts, 0, Ndomain*contacts, 1) = Eigen::MatrixXd::Ones(Ndomain*contacts, 1);

    // float a, b, c; if (gaitDomain_ < 100) { a = 1; b = 2; c = 1; } else { a = -1; b = 2; c = -5; }
    // Gineq.block(Gineq.rows()-1, 0, 1, num_dec_vars) << a, 0, b, 0;
    // Gineq.block(Gineq.rows()-1, Gineq.cols()-1, 1, 1) << c;
    // cout << "Test Gineq last row:  \n" << Gineq.row(Gineq.rows()-1); std::cin.get();
    // cout << "Test Gineq*x <= hinq last row:  \n" << Gineq.row(Gineq.rows()-1)*hineq; std::cin.get(); 

    // Adding - obstacle test
    // hineq.block(hineq.rows()-1, 0, 1, 1) << 0.5;

    // TO DO - BASIT
    // Try to add obstacle constraints in the Ginq * x <= hinq          // Friction cone, support polygon constraints - add obstacles
    //                                        Aeq  * x == beq           // Terminal and pointwise state constraints
    // Relax terminal constraints in y direction by a slack variable (delta), minimize the norm of (delta) in the cost function
    
    // DONE AS OF YET
    // Added one more decision variable - slack variable to relax tracking cost (unconstrained)

    // std::cout << "NUM DEC VARS = " << num_dec_vars << "   beq = " << beq.rows() << "   hineq = " << hineq.rows() << "\n";
    ////// QP solve
    mpc_state_e_eventbased_.setZero(num_dec_vars, 1);
    mpc_state_e_x_eventbased_.setZero(n*N,1);
    mpc_state_e_u_eventbased_.setZero(N,1);
    mpc_state_eventbased_ = new double[(int)num_dec_vars];


    iswiftQp_e(Q_qp, f_qp, Aeq, beq, Gineq, hineq, mpc_state_eventbased_);

    qp_solution_eventbased_.setZero(num_dec_vars, 1);
    for (size_t i = 0; i < num_dec_vars; i++)
    {
        qp_solution_eventbased_(i) = mpc_state_eventbased_[i];
    }
    
    Eigen::VectorXd x00;
    x00.setZero(num_dec_vars);
    for (int i = 0; i < x00.rows(); i++)
    {
        x00(i,0) = mpc_state_eventbased_[i];
    }
        
    SnoptSolver solver;
    Eigen::VectorXd x;
    Problem nlp;
    Eigen::Vector2d pos; pos << q[0], q[1];
    Vector2d close_obs;
    Vector2d close_agent_dist;
    Vector2d close_obs_dist;

    double min_dist = 1000;
    int min_i = 0;
    for (int i = 0; i < NUMBER_OF_OBS; i++)
    {   
        Vector2d ith_obs; ith_obs = Pobs_real.block(0,i,2,1);
        double dist_to_obs = sqrt(pow(pos(0)-ith_obs(0),2)+pow(pos(1)-ith_obs(1),2));
        if (dist_to_obs < min_dist)
        {
            min_dist = dist_to_obs;
            min_i = i;
        }
    }

    close_obs_dist << Pobs_real.coeff(0, min_i)-q[0], Pobs_real.coeff(1, min_i)-q[1]; 
    close_agent_dist << state_other(0)-q[0], state_other(1)-q[1];

    // std::cout << "The two norm is: " << close_obs_dist.norm() << std::endl;
    // agent_dist_file <<  close_obs_dist.norm() << std::endl;
    obs_dist_file << close_agent_dist.norm() << std::endl;

    // close_obs << state_other(0)+1*state_other(2)*4*TSOPTTICK*0.001, state_other(1)+1*state_other(3)*4*TSOPTTICK*0.001;
    //if(agent_id_ == 1) {cout << "x y of the other agent === " << close_obs.transpose() << std::endl;}// cin.get(); }
    // close_obs << 1.0, -0.1;

    // cout << "CHECK THE CLOSE OBS THING:\n" << close_obs << std::endl; std::cin.get();
    close_obs << Pobs_real.coeff(0, min_i), Pobs_real.coeff(1, min_i); 

    // SnoptSolver solver;
    // Problem nlp;

    // Send the closest obstacle as well --- TO BE DONE
    if (this->use_snopt == true) {
        std::shared_ptr<nlpTuple> nlpdata = std::make_shared<nlpTuple>(Q_qp, f_qp, Aeq, beq, Gineq, hineq, x00, pos, state_other, close_obs); 
        nlp.AddVariableSet(std::make_shared<ExVariables>(nlpdata, num_dec_vars, "var_set1"));
        nlp.AddConstraintSet(std::make_shared<EqConstraints>(nlpdata, num_dec_vars, "constraint1"));
        nlp.AddConstraintSet(std::make_shared<NeqConstraints>(nlpdata, num_dec_vars, "constraint2"));
    
        nlp.AddCostSet(std::make_shared<ExCost>(nlpdata, num_dec_vars, "cost_term1"));

        // Start the timer
        std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();

        solver.Solve(nlp);

        // Stop the timer
        std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
        // Calculate the elapsed time in milliseconds
        std::chrono::duration<double, std::milli> elapsed = end - begin;
        agent_dist_file <<  elapsed.count() << std::endl;

        x = nlp.GetOptVariables()->GetValues();

        for (size_t i = 0; i < num_dec_vars; i++)
        {
            mpc_state_eventbased_[i] = x(i);
        }   
    }

/////////////////////////////////// OLD QP SOLVE //////////////////////////////////////

    for(size_t i=0; i<num_dec_vars; i++){
        mpc_state_e_eventbased_(i) = mpc_state_eventbased_[i];
    }
    for(size_t i=0; i<n*N; i++){
        mpc_state_e_x_eventbased_(i) = mpc_state_e_eventbased_(i);
    }
    // cout << "******************* MPC OUTPUT ******************* \n" << mpc_state_e_x_eventbased_ << "\n";
    for(size_t i=0; i<N; i++){
        mpc_state_e_u_eventbased_(i) = mpc_state_e_eventbased_(n*N+m*i);
    }
    // cout << "deleting mpc_state_eventbased_\n";
    delete[] mpc_state_eventbased_;
    // cout << "deleted mpc_state_eventbased_\n";
    // std::cout << "event based MPC solution done correctly" <<std::endl;
    std::cout << "Domain is: " << gaitDomain_ << std::endl;

    //std::cout << mpc_state_e_x_eventbased_ << std::endl;
    //std::cout << "MPC running test" << std::endl;

    fitComTrajectory_eventbase(M, Ndomain, N, swingPhase);
    logMPC_Data();
    gaitDomain_++;
    // cout << "DBG: Does this happen? 1\n";
}

void MPC_dist::footstepPlanner_eventbase(double& M, double& Ndomain, double& N, double& mu, double& swingPhase, double& row_totalFootPrintGlobal, double& col_totalFootPrintGlobal){
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
    
    int scale = 1.0;
    // // old
    // initialFootprint <<  0.1830*scale,  0.1320*scale,    // FR 0 + -
    //                     -0.1830*scale,  0.1320*scale,    // FL 1 + + 
    //                      0.1830*scale, -0.1320*scale,    // RR 2 - -
    //                     -0.1830*scale, -0.1320*scale;    // RL 3 - +
    // [0.3156; 0.2200; -0.3444; 0.2200; 0.3156; -0.2200; -0.3444; -0.2200];

    // New - doesn't work
    
    initialFootprint <<  0.1830*scale, -0.1320*scale,    // FR 0 + -
                         0.1830*scale,  0.1320*scale,    // FL 1 + + 
                        -0.1830*scale, -0.1320*scale,    // RR 2 - -
                        -0.1830*scale,  0.1320*scale;    // RL 3 - +

    // apply yaw angle at initial footprint
    // KEYPOINT: first rotate the footprint without translation
    // after that, translate rotated footprint based on agent initial pos info
    // cout << "agent_Initial (MPC):\n" << agent_Initial_ << std::endl;

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
    

    // The following code makes a big footprint matrix for all domains from 0 to M
    M = totalCycleIndex_.cols();

    if(totalCycleIndex_.cols() % 2 == 0){
    //   totalFootprint.block(0,2,8,1) << initialFootprint(0)+step_length(0)*0.5, initialFootprint(1)+step_length(1)*0.5, 
    //                                     swingPhase, swingPhase, 
    //                                     swingPhase, swingPhase, 
    //                                     initialFootprint(6)+step_length(0)*0.5, initialFootprint(7)+step_length(1)*0.5;
  
        for(size_t k=2; k<M-1; k++){
    
            qref.block(0,0,1,(gap+1)) = Pr_refined_.block(2*agent_id_,(gap+1)*(k-2),1,(gap+1));
            qref.block(2,0,1,(gap+1)) = Pr_refined_.block(2*agent_id_+1,(gap+1)*(k-2),1,(gap+1));

            newFoothold.setZero();
            newFoothold << qref(0,gap)+ initialFootprint(0), qref(2,gap)+ initialFootprint(1), 
                            qref(0,gap)+ initialFootprint(2), qref(2,gap)+ initialFootprint(3),
                            qref(0,gap)+ initialFootprint(4), qref(2,gap)+ initialFootprint(5),
                            qref(0,gap)+ initialFootprint(6), qref(2,gap)+ initialFootprint(7); 
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

        // The following code handles the last 3rd domain where all contacts are happening
        // qref.setZero();
        qref.block(0,0,1,(gap+1)) = Pr_refined_.block(2*agent_id_,(gap+1)*(M-1-2),1,(gap+1));
        qref.block(2,0,1,(gap+1)) = Pr_refined_.block(2*agent_id_+1,(gap+1)*(M-1-2),1,(gap+1));

        newFoothold.setZero();
        newFoothold <<  qref(0,gap)+ initialFootprint(0), qref(2,gap)+ initialFootprint(1), 
                        qref(0,gap)+ initialFootprint(2), qref(2,gap)+ initialFootprint(3),
                        qref(0,gap)+ initialFootprint(4), qref(2,gap)+ initialFootprint(5),
                        qref(0,gap)+ initialFootprint(6), qref(2,gap)+ initialFootprint(7); // newFoothold << qref(0,gap)+0.33, qref(2,gap)+0.22, qref(0,gap)-0.33,qref(2,gap)+0.22,qref(0,gap)+0.33,qref(2,gap)-0.22,qref(0,gap)-0.33,qref(2,gap)-0.22;

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
    //truncating the matrix when gait number is increasing: maybe using gaitDomain_
    cycleIndexTruncated = totalCycleIndex_;
    footPrintTruncated = totalFootprint;

    // cout << "cycleIndexTruncated (MPC_dist):   \n" << cycleIndexTruncated << std:: endl;
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
    for(size_t dom=0; dom< 2; dom++){                               // ------------------------------------------- <<<<<<<<<<<<<<<<<<<<<<<< dom = 5

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
        else{
            std::cout << "flying?" << std::endl;
        }
    }

    totalFootprint_ = totalFootprint; // pure foot print data
    cycleIndexTruncated_ = cycleIndexTruncated;
    footPrintTruncated_ = footPrintTruncated;
    footPrintGlobalTruncated_ = totalFootprintGlobal; // foot print data expanded to the grid based matrix
    footPrintGlobalOnesTruncated_ = totalFootprintGlobalOnes;

    // cout << "FootPrint << \n" << footPrintTruncated_.block(0,0,8,4);

    // if (0) {std::cout << "MPC footprinttruncated:"; printSize(footPrintTruncated_.block(0,0,8,6)); cin.get();}
    
    if(agent_id_ == 0)
            std::cout << "Domain: " << domain_++ << std::endl;          // REF FLAG

    // cout << "FootprintTruncated = \n" << footPrintTruncated_.block(0,0,8,4) << "\n\n";
    // cout << "qref = \n" << qref << "\n\n"; 
    // cin.get();


    // std::fstream myfile;
    // myfile.open("/home/hdsrl7/Documents/total_footprint.txt",std::fstream::out);
    // myfile << totalFootprint;
    // myfile.close();    
}

void MPC_dist::copPlanner_eventbase(double M, double Ndomain, double N, double swingPhase){
    // this function is used for generating desired COM in the MPC as prof style(newMPC branch in hdsrl_github)
    
    Eigen::Matrix<double, 4, 1> com_end;
    Eigen::Matrix<double, 4, 1> com_start_counter;
    Eigen::Matrix<double, 4, 1> com_end_counter;

    com_start.setZero();
    com_end.setZero();
    com_start_counter.setZero();
    com_end_counter.setZero();
    
    // MAJOR CHANGE NEEDED -> get com_traj_desired from qref
    Eigen::MatrixXd qref_dbg; qref_dbg.setZero(4,N);
    qref_dbg.block(0,0,1,N) = Pr_refined_.block(2*agent_id_,(N)*(gaitDomain_+0),1,(N));
    qref_dbg.block(1,0,1,N) = Prd_refined_.block(2*agent_id_,(N)*(gaitDomain_+0),1,(N));
    qref_dbg.block(2,0,1,N) = Pr_refined_.block(2*agent_id_+1,(N)*(gaitDomain_+0),1,(N));
    qref_dbg.block(3,0,1,N) = Prd_refined_.block(2*agent_id_+1,(N)*(gaitDomain_+0),1,(N));

    // Get the first point of trajectory from 4 domains ahead

    Eigen::MatrixXd com_desired_Traj;
    bool use_bezier = false;
    if (use_bezier) // replan trajectory
    {
        Eigen::Vector4d qref_ahead; qref_ahead.setZero();
        qref_ahead(0) = Pr_refined_(2*agent_id_,(1)*(gaitDomain_+1));
        qref_ahead(1) = Prd_refined_(2*agent_id_,(1)*(gaitDomain_+1));
        qref_ahead(2) = Pr_refined_(2*agent_id_+1,(1)*(gaitDomain_+1));
        qref_ahead(3) = Prd_refined_(2*agent_id_+1,(1)*(gaitDomain_+1));

        //cout << "BEZIER TEST:\n"; cout << "point ahead = " << qref_ahead.transpose() << "\n\n" << "Current State =  (x) " << q[0] << "   (y) " << q[1]; cin.get(); 
        
        // Use the current state and use a cubic bezier to plan for the intermediate trajectory
        // Define a matrix -> a = [a0 a1 a2 a3];
        //cout << "DBG << " << dq[0] << "    " << dq[1] << "\n\n"; cin.get(); 

        MatrixXd a; a.setZero(2, 4);
        a << q[0], q[0] + 0.2, qref_ahead(0)- 0.0, qref_ahead(0),
            q[1], q[1] , qref_ahead(2) , qref_ahead(2);

        // Linear Interpolation

        qref_dbg.setZero(4, 4);
        MatrixXd qref_bzr; qref_bzr.setZero(4, 4);
        double scalex = 10; double scaley = 4;
        qref_bzr.block(0, 0, 2, 1) = bezier2d(0.25/scalex, a); qref_bzr.block(0, 1, 2, 1) = bezier2d(0.5/scalex, a); qref_bzr.block(0, 2, 2, 1) = bezier2d(0.75/scalex, a); qref_bzr.block(0, 3, 2, 1) = bezier2d(1/scalex, a); 
        qref_bzr.block(2, 0, 2, 1) = bezierd2d(0.25/scaley, a); qref_bzr.block(2, 1, 2, 1) = bezierd2d(0.5/scaley, a); qref_bzr.block(2, 2, 2, 1) = bezierd2d(0.75/scaley, a); qref_bzr.block(2, 3, 2, 1) = bezierd2d(1/scaley, a);

        //cout << "DBG 9901 "; cin.get();

        // qref_dbg.block(0, 0, 1, 4) = qref_bzr.block(0, 0, 1, 4);
        // qref_dbg.block(1, 0, 1, 4) = qref_bzr.block(2, 0, 1, 4);
        qref_dbg.block(0,0,1,N) = Pr_refined_.block(2*agent_id_,(N)*(gaitDomain_+0),1,(N));
        qref_dbg.block(1,0,1,N) = Prd_refined_.block(2*agent_id_,(N)*(gaitDomain_+0),1,(N));
        qref_dbg.block(2, 0, 1, 4) = qref_bzr.block(1, 0, 1, 4);
        qref_dbg.block(3, 0, 1, 4) = qref_bzr.block(3, 0, 1, 4);
    }
    //cout << "qref_bzr = " << qref_dbg; cin.get();

    // VERIFY THIS BEZIER NUMBERS IN MATLAB


    com_desired_Traj.setZero(4, N);
    // Use qred as desired traj
    Eigen::Vector4d qref_tmp; qref_tmp << 0,0,0,0;
    for (size_t i = 0; i < 4; i++)
    {   
        com_desired_Traj.block(i,0,1,N) << qref_dbg(i, 0), qref_dbg(i, 1), qref_dbg(i, 2), qref_dbg(i, 3);
        com_start(i) = qref_dbg(i, 0);
    }

    Eigen::Vector2d com_P; com_P << q[0], q[1];

    Eigen::Vector2d goal_P; goal_P << GOAL_X, GOAL_Y;
    Eigen::Vector2d diff_G; diff_G.setZero(); diff_G << com_P - goal_P;

    com_desired_Traj_eventbased_ = com_desired_Traj;
    Eigen::VectorXd com_desired_Traj_vec(Eigen::Map<Eigen::VectorXd>(com_desired_Traj.data(), com_desired_Traj.cols()*com_desired_Traj.rows()));
    com_desired_Traj_vec_eventbased_ = com_desired_Traj_vec;
}

void MPC_dist::fitComTrajectory_eventbase(double M, double Ndomain, double N, double swingPhase){

    Eigen::Map<Eigen::MatrixXd> mpc_state_e_x_mtx(mpc_state_e_x_eventbased_.data(), 4, N);
    Eigen::MatrixXd mpc_state_e_x_mtxforalpha;
    Eigen::MatrixXd alpha_COM_traj_e;
    mpc_state_e_x_mtxforalpha.setZero(4*N/Ndomain, (Ndomain+1));
    alpha_COM_traj_e.setZero(4*N/Ndomain, (Ndomain+1));

    mpc_state_e_x_mtxforalpha.block(0,0,4,1) = mpc_state_alpha_buffer_;
    mpc_state_e_x_mtxforalpha.block(0,1,4,4) = mpc_state_e_x_mtx.block(0,0,4,4);
    for(size_t k = 1; k< N/Ndomain; k++){
        mpc_state_e_x_mtxforalpha.block(k*4,0,4,5)=mpc_state_e_x_mtx.block(0,k*4-1,4,5);
    }

    mpc_state_alpha_buffer_ = mpc_state_e_x_mtx.block(0,3,4,1);

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

        // QQQ.setZero(4*(Ndomain+1)+8, 4*(Ndomain+1)+8); //QQQ.setZero(4*(Ndomain+1)+4, 4*(Ndomain+1)+4);
        // PPP.setZero(4*(Ndomain+1)+8,1);
        // QQQ.block(0,0, 4*(Ndomain+1), 4*(Ndomain+1)) = binomialmtx_foronedomain.transpose() * binomialmtx_foronedomain;
        // QQQ.block(0, 4*(Ndomain+1),4*(Ndomain+1), 8) = eqconstraintmtx.transpose();
        // QQQ.block(4*(Ndomain+1), 0, 8, 4*(Ndomain+1)) = eqconstraintmtx;
        // PPP.block(0,0, 4*(Ndomain+1),1) = binomialmtx_foronedomain.transpose()*mpc_state_e_foronedomain;
        // PPP.block(4*(Ndomain+1), 0 , 8, 1) = eqconstraintvec;

        QQQ.setZero(4*(Ndomain+1)+4, 4*(Ndomain+1)+4); //QQQ.setZero(4*(Ndomain+1)+4, 4*(Ndomain+1)+4);
        PPP.setZero(4*(Ndomain+1)+4,1);
        QQQ.block(0,0, 4*(Ndomain+1), 4*(Ndomain+1)) = binomialmtx_foronedomain.transpose() * binomialmtx_foronedomain;
        QQQ.block(0, 4*(Ndomain+1),4*(Ndomain+1), 4) = eqconstraintmtx.transpose();
        QQQ.block(4*(Ndomain+1), 0, 4, 4*(Ndomain+1)) = eqconstraintmtx;
        PPP.block(0,0, 4*(Ndomain+1),1) = binomialmtx_foronedomain.transpose()*mpc_state_e_foronedomain;
        PPP.block(4*(Ndomain+1), 0 , 4, 1) = eqconstraintvec;

        alpha_e_vec_foronedomain = (QQQ.inverse()*PPP).block(0,0, 4*(Ndomain+1),1);
        
        Eigen::Map<Eigen::MatrixXd> alpha_mtx_foronedomain(alpha_e_vec_foronedomain.data(), 4, 5);
        alpha_COM_traj_e.block(i*4,0,4,5) = alpha_mtx_foronedomain;
    }
    alpha_COM_traj_e_ = alpha_COM_traj_e;
}

void MPC_dist::plannedCycleIndex(size_t gait){
    //std::cout << "ENTER plannedCycleIndex DBG 1\n";
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
        // std::cout << "index generation error" << std::endl;
    }
    //std::cout << "EXIT plannedCycleIndex DBG 1\n";
}

void MPC_dist::totalCycleIndex(size_t gait, size_t gaitCycleNum){
    //std::cout << "ENTER totalCycleIndex DBG 1\n";

    oneCycleIndex(gait);
    totalCycleIndex_.setOnes(4, 4*gaitCycleNum+2);
    for(size_t i=0; i< gaitCycleNum; i++){
        totalCycleIndex_.block(0,i*4+1,4,4)=oneCycleIndex_;
    }
    //totalStepNum_ = totalCycleIndex_.cols();
}

void MPC_dist::oneCycleIndex(size_t gait){ //total half forward
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
        oneCycleIndex_.block(0,0,4,1) = leg0stride; //half stride
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

void MPC_dist::totalCycleIndexwHalf(size_t gait, size_t gaitCycleNum){
    //std::cout << "ENTER totalCycleIndexwHalf DBG 1\n";
    oneCycleIndex(gait);
    totalCycleIndex_.setOnes(4,4*gaitCycleNum+4);
    for(size_t i=0; i< gaitCycleNum; i++){
        totalCycleIndex_.block(0,i*4+1,4,4)=oneCycleIndex_;
    }
    totalCycleIndex_.block(0,gaitCycleNum*4+1,4,2)=oneCycleIndex_.block(0,0,4,2);
    //totalStepNum_ = totalCycleIndex_.cols();
}

void MPC_dist::generateReferenceTrajectory()
{
    double c = 550.0, m = 8.0, epsilon = 60, sigma = 1.0;
    double dth = 0.8, alpha = 150.0, eta = 400.0, dmin = 1.0;
    int loopSize = 100000; size_t num_obs = Pobs.cols();
    const size_t ramp_up_iterations = 1000;

    double Ts = TSOPTTICK*0.001/10;

    MatrixXd Ad; Ad.setZero(4, 4);
    MatrixXd Ae; Ae.setZero(4*NUMBER_OF_AGENTS, 4*NUMBER_OF_AGENTS);

    Ad <<   1,                   0,   0.001025061239872,                   0,
            0,                   1,                   0,   0.001025061239872,
            0,                   0,   0.929527039758809,                   0,
            0,                   0,                   0,   0.929527039758809;


    // Ae.block(0, 0, 4, 4) = Ad;  // Needs to be extended to generic number of agents case
    // Ae.block(4, 4, 4, 4) = Ad;  // Needs to be extended to generic number of agents case
    // Ae.block(8, 8, 4, 4) = Ad;  // Needs to be extended to generic number of agents case
    // Ae.block(12, 12, 4, 4) = Ad;  // Needs to be extended to generic number of agents case

    for (int agent = 0; agent < NUMBER_OF_AGENTS; ++agent) 
    {
        int blockStartIndex = agent * 4; // Calculate the start index for both row and column
        Ae.block(blockStartIndex, blockStartIndex, 4, 4) = Ad;
    }


    MatrixXd Bd; Bd.setZero(4, 2);
    MatrixXd Be; Be.setZero(4*NUMBER_OF_AGENTS, 2*NUMBER_OF_AGENTS);

    Bd <<   0.000000068070472960,                   0,
            0,                   0.000000068070472960,
            0.000128132654983983,                   0,
            0,                   0.000128132654983983;

    Be.block(0, 0, 4, 2) = Bd;
    Be.block(4, 2, 4, 2) = Bd;
    Be.block(8, 4, 4, 2) = Bd;
    Be.block(12, 6, 4, 2) = Bd;
    
    // Reference Loop
    MatrixXd q; q.setZero(4*NUMBER_OF_AGENTS, loopSize+1);
    

    // Generic initialization of starting positions and velocities
    for (int k = 0; k < NUMBER_OF_AGENTS; ++k) {
        q.block(4*k, 0, 2, 1) = Pstart_.segment(2*k, 2); // Set starting positions for each agent
        q.block(4*k + 2, 0, 2, 1).setZero(); // Set initial velocities to zero
    }


    for (size_t i = 0; i < loopSize; i++)
    {   
        MatrixXd F; F.setZero(8, 1);
        // for each agent
        for (size_t k = 0; k < NUMBER_OF_AGENTS; k++)     
        {
            // Initialized required variables
            Vector2d qk_p, qk_p_other, p_g, F_att, F_rep, F_agent; qk_p.setZero(); qk_p_other.setZero(); p_g.setZero(); F_att.setZero(); F_rep.setZero(); F_agent.setZero(); 
            VectorXd d_obs; d_obs.setZero(NUMBER_OF_OBS);
            double d_goal = 0.0, d_agent = 0.0;

            // Index current and other agent positions
            qk_p = q.block(0+k*4, i, 2, 1);

            // Goal point
            p_g << GOAL_X, GOAL_Y;

            // Distance from goal
            d_goal = (qk_p - p_g).norm();

            // Attractive Force
            F_att = -alpha*((qk_p - p_g)/(qk_p - p_g).norm());

            // Distance from Obstacle
            for (size_t j = 0; j < num_obs; j++)
            {
                d_obs(j) = (qk_p - Pobs.col(j)).norm();
            }

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

            for (size_t j = 0; j < NUMBER_OF_AGENTS; j++)
            {
                if ( k != j)
                {
                    // Distance form other agent
                    qk_p_other = q.block(0+(j)*4, i, 2, 1);
                    d_agent = (qk_p - qk_p_other).norm();
                    // Lennard Jones Potential
                    F_agent = F_agent - 4*epsilon*( (6*pow(sigma, 6))/pow(d_agent, 7) - (12*pow(sigma, 12))/pow(d_agent, 13))*( (qk_p - qk_p_other)/(qk_p - qk_p_other).norm() );
                }
            }

            F.block(0 + k*2, 0, 2, 1) = (F_att + F_rep + F_agent);

            if (d_goal < 0.001)
            {
                F.block(0 + k*2, 0, 2, 1) = 0*F_att;
            }
        }
        // Scale the force vector during the ramp-up phase
        double scale_factor = (i < ramp_up_iterations) ? (static_cast<double>(i) / ramp_up_iterations) : 1.0;

        q.col(i+1) = Ae*q.col(i) + Be*F*scale_factor;
    }

    for (size_t i = 0; i < loopSize/40; i++)
    {
        q.col(i) = q.col(40*i);
    }

    MatrixXd Pr; Pr.setZero(8,   loopSize/40);
    MatrixXd Prd; Prd.setZero(8, loopSize/40);

    for (int agent = 0; agent < NUMBER_OF_AGENTS; ++agent) 
    {
        int baseRowPr = agent * 2; // Base row for Pr operations
        int baseRowPrd = agent * 2; // Base row for Prd operations
        int baseRowQ = agent * 4; // Base row for q.block operations

        Pr.row(baseRowPr) = q.block(baseRowQ, 2, 1, loopSize / 40);
        Pr.row(baseRowPr + 1) = q.block(baseRowQ + 1, 2, 1, loopSize / 40);
        Prd.row(baseRowPrd) = q.block(baseRowQ + 2, 2, 1, loopSize / 40);
        Prd.row(baseRowPrd + 1) = q.block(baseRowQ + 3, 2, 1, loopSize / 40);
    }


    Pr_refined_ = Pr;
    Prd_refined_ = Prd;
    
    // =========================================================================//
    // ================================= LOGGING ===============================//
    // =========================================================================//

    // Step back one directory from the current working directory
    std::filesystem::path baseDir = std::filesystem::current_path().parent_path();
    std::filesystem::path outputsDir = baseDir / "Sim_Outputs";

    // Check if the Sim_Outputs directory exists, and create it if it doesn't
    if (!std::filesystem::exists(outputsDir)) {
        std::filesystem::create_directories(outputsDir);
    }

    // Log HL Path
    std::filesystem::path filePath = outputsDir / "HLPath.txt";
    std::fstream myfile5;
    myfile5.open(filePath, std::fstream::out);
    if (!myfile5.is_open()) {
        std::cerr << "Failed to open file: " << filePath << std::endl;
        // Handle error, possibly exit the function or program
    } else {
        myfile5 << Pr_refined_;
        myfile5.close();
    }

    // Log HL Velocity
    filePath = outputsDir / "HLVelocity.txt";
    std::fstream myfile6;
    myfile6.open(filePath, std::fstream::out);
    if (!myfile6.is_open()) {
        std::cerr << "Failed to open file: " << filePath << std::endl;
        // Handle error, possibly exit the function or program
    } else {
        myfile6 << Prd_refined_;
        myfile6.close();
    }
}

void MPC_dist::setPstart(Eigen::Matrix<double, 2*NUMBER_OF_AGENTS, 1>& Pstart)
{
    Pstart_ << Pstart;
    agent_Initial_(0) = Pstart_(2*agent_id_);
    agent_Initial_(1) = Pstart_(2*agent_id_+1);
    mpc_state_alpha_buffer_<< agent_Initial_(0), 0, agent_Initial_(1), 0;
}

void MPC_dist::printPstart()
{
    std::cout << "Pstart is:\n " << Pstart_ << std::endl;
}

void MPC_dist::setPobs(Eigen::Matrix<double, 2,NUMBER_OF_OBS>& Pobs_in)
{
    Pobs << Pobs_in;
}

void MPC_dist::setPobs_real(Eigen::Matrix<double, 2,NUMBER_OF_OBS>& Pobs_in)
{
    Pobs_real << Pobs_in;
}

void MPC_dist::printPobs()
{
    std::cout << "Pobs is:\n " << Pobs << std::endl;
}

void MPC_dist::setAgentID(size_t agent_id)
{
    agent_id_ = agent_id;
}

void MPC_dist::logMPC_Data()
{

    qp_file <<  qp_solution_eventbased_(0) << "," << qp_solution_eventbased_(1) << "," << qp_solution_eventbased_(2) << "," << qp_solution_eventbased_(3) << "," << 
                qp_solution_eventbased_(4) << "," << qp_solution_eventbased_(5) << "," << qp_solution_eventbased_(6) << "," << qp_solution_eventbased_(7) << "," << 
                qp_solution_eventbased_(8) << "," << qp_solution_eventbased_(9) << "," << qp_solution_eventbased_(10) << "," << qp_solution_eventbased_(11) << "," << 
                qp_solution_eventbased_(12) << "," << qp_solution_eventbased_(13) << "," << qp_solution_eventbased_(14) << "," << qp_solution_eventbased_(15) << "," << 
                qp_solution_eventbased_(16) << "," << qp_solution_eventbased_(17) << "," << qp_solution_eventbased_(18) << "," << qp_solution_eventbased_(19) << "," << 
                qp_solution_eventbased_(20) << "," << qp_solution_eventbased_(21) << "," << qp_solution_eventbased_(22) << "," << qp_solution_eventbased_(23) << "," << std::endl;

    mpc_file << mpc_state_e_x_eventbased_(0) << "," <<  mpc_state_e_x_eventbased_(1) << "," << mpc_state_e_x_eventbased_(2) << "," << mpc_state_e_x_eventbased_(3) << "," << 
                mpc_state_e_x_eventbased_(4) << "," <<  mpc_state_e_x_eventbased_(5) << "," << mpc_state_e_x_eventbased_(6) << "," << mpc_state_e_x_eventbased_(7) << "," << 
                mpc_state_e_x_eventbased_(8) << "," <<  mpc_state_e_x_eventbased_(9) << "," << mpc_state_e_x_eventbased_(10) << "," << mpc_state_e_x_eventbased_(11) << "," << 
                mpc_state_e_x_eventbased_(12) << "," << mpc_state_e_x_eventbased_(13) << "," << mpc_state_e_x_eventbased_(14) << "," << mpc_state_e_x_eventbased_(15) << "," << 
                mpc_state_e_u_eventbased_(16) << "," << mpc_state_e_u_eventbased_(17) << "," << mpc_state_e_u_eventbased_(18) << "," << mpc_state_e_u_eventbased_(19) << "," << 
                mpc_state_e_u_eventbased_(20) << "," << mpc_state_e_u_eventbased_(21) << "," << mpc_state_e_u_eventbased_(22) << "," << mpc_state_e_u_eventbased_(23) << "," << std::endl;

    
    // std::fstream MPC_Log_Data; ///home/hdsrl7/Desktop/A1_RaiSim_Outputs/exp_
    // //MPC_Log_Data.open("/home/hdsrl7/Documents/RAISIM_WORKSPACE/A1_NMPC_oldLL_working/NMPC_Distributed_A1-master/matlab_dbg/MPC_loco.txt",std::fstream::out);
    // MPC_Log_Data.open("///home/hdsrl7/Desktop/A1_RaiSim_Outputs/exp_B_MPC_loco" + std::to_string(log_ID) + ".txt",std::fstream::out);
    // MPC_Log_Data << mpc_state_e_eventbased_;
    // MPC_Log_Data.close();

    // std::fstream COM_DES;
    // COM_DES.open("///home/hdsrl7/Desktop/A1_RaiSim_Outputs/exp_B_COP_loco" + std::to_string(log_ID) + ".txt",std::fstream::out);
    // COM_DES << com_desired_Traj_eventbased_;
    // COM_DES.close();

    // std::fstream COP;
    // COP.open("///home/hdsrl7/Desktop/A1_RaiSim_Outputs/exp_B_COP_loco" + std::to_string(log_ID) + ".txt",std::fstream::out);
    // COP << mpc_state_e_u_eventbased_;
    // COP.close();

    // std::fstream QP_SOL;
    // QP_SOL.open("///home/hdsrl7/Desktop/A1_RaiSim_Outputs/exp_B_QP_SOL_loco" + std::to_string(log_ID) + ".txt",std::fstream::out);
    // QP_SOL << qp_solution_eventbased_;
    // QP_SOL.close();
}

void MPC_dist::printSize(const Eigen::MatrixXd& mat)
{
    cout << "\nRows(): " << mat.rows() << "   cols(): " << mat.cols() << std::endl;
    cout << mat << std::endl;
} 

void MPC_dist::stepSetup(double totalStepNum, double step_X, double step_Y, double body_R, double body_P, double body_Y, Eigen::MatrixXd agent_Initial){
    totalStepNum_ = totalStepNum;
    // step_X_ = step_X;
    // step_Y_ = step_Y;
    body_R_ = body_R;
    body_P_ = body_P;
    body_Y_ = body_Y;
    agent_Initial_ = agent_Initial;
}

void MPC_dist::updateState(double* qin, double* dqin, int* ind, Eigen::Matrix<double, 3, 4> toePos, Eigen::Vector4d state_vec) { 
    memcpy(q,qin,18*sizeof(double)); 
    memcpy(dq,dqin,18*sizeof(double)); 
    memcpy(contactInd, ind,4*sizeof(int));
    toePos_ = toePos;
    state_other = state_vec;
    // cout << "OTHER AGENT's STATE (" << log_ID << "):   " << state_other.transpose() << std::endl;
} 

void MPC_dist::footholdsPlanner() {
    Eigen::Matrix<double, 4, 2> initFootprint; initFootprint.setZero();
    initFootprint << 0.2188, -0.1320,    // FR 0 + -
                     0.2188,  0.1320,    // FL 1 + + 
                    -0.1472, -0.1320,    // RR 2 - -
                    -0.1472,  0.1320;    // RL 3 - +

    if(gaitDomain_ <= 1) // Before and During Stand Up, loco starts at gaitDomain > 1
    {
        toePos_ << initFootprint(0, 0) + agent_Initial_(0), initFootprint(1, 0) + agent_Initial_(0), initFootprint(2, 0) + agent_Initial_(0), initFootprint(3, 0) + agent_Initial_(0),
                   initFootprint(0, 1) + agent_Initial_(1), initFootprint(1, 1) + agent_Initial_(1), initFootprint(2, 1) + agent_Initial_(1), initFootprint(3, 1) + agent_Initial_(1),
                        0.0,      0.0,       0.0,       0.0;
    }

    // No need to do footstep planning, get the current footholds directly from KinematicsInfo and then use them to plan over current domain
    // std::cout << "toePose_ from MPC_dist is: \n" << toePos_ << std::endl; 
    // std::cout << "gaitDomain? = " << gaitDomain_; std::cin.get();

    Eigen::Matrix<double, 8, 1> currFoothold;
    for (size_t i = 0; i < 4; i++)
    {
        currFoothold(2*i, 0) = (contactInd[i] == 1) ? toePos_(0, i) : inf;
        currFoothold(2*i+1, 0) = (contactInd[i] == 1) ? toePos_(1, i) : inf;
    }
    // std::cout << "currFoothold? = \n" << currFoothold;

    int contacts = contactInd[0]+contactInd[1]+contactInd[2]+contactInd[3];

    Eigen::MatrixXd footPrintDiagTmp;
    Eigen::MatrixXd footPrintDiagTmpOnes;
    footPrintDiagTmp.setZero(8, contacts*NDOMAIN);
    footPrintDiagTmpOnes.setZero(8, contacts*NDOMAIN);
    Eigen::MatrixXd tmpFootHold; tmpFootHold.setZero(2, contacts);

    if (contacts == 4)
    {
        tmpFootHold << toePos_(0, 0), toePos_(0, 1), toePos_(0, 2), toePos_(0, 3),
                       toePos_(1, 0), toePos_(1, 1), toePos_(1, 2), toePos_(1, 3);
    }
    else if(contacts == 2 && contactInd[0] == 0) // 0 1 1 0
    {
        tmpFootHold << toePos_(0, 1), toePos_(0, 2),
                       toePos_(1, 1), toePos_(1, 2);
    }
    else if(contacts == 2 && contactInd[0] == 1) // 1 0 0 1
    {
        tmpFootHold << toePos_(0, 0), toePos_(0, 3),
                       toePos_(1, 0), toePos_(1, 3);
    }

    // std::cout << "tmpFootHold? = \n" << tmpFootHold; //std::cin.get();

    for (size_t i = 0; i < NDOMAIN; i++)
    {
        footPrintDiagTmp.block(2*i, contacts*i, 2, contacts) << tmpFootHold;
        footPrintDiagTmpOnes.block(2*i, contacts*i, 2, contacts) << Eigen::MatrixXd::Ones(2, contacts);
    }

    // std::cout << "footPrintDiagTmp? = \n" << footPrintDiagTmp; //std::cin.get();
    footPrintGlobalTruncated_.setZero(footPrintDiagTmp.rows(), footPrintDiagTmp.cols());
    footPrintGlobalTruncated_ = footPrintDiagTmp;
    footPrintGlobalOnesTruncated_ = footPrintDiagTmpOnes;
}

void MPC_dist::getStateOtherAgent(Eigen::Vector4d state_vec) {
    state_other = state_vec;
}

Eigen::Vector4d MPC_dist::get_lastState() {
    Eigen::Vector4d last_state; 
    last_state << q[0], q[1], dq[0], dq[1];
    return last_state;
}

MatrixXd MPC_dist::bezier2d(double s, MatrixXd a)
{
    MatrixXd p; p.setZero(2, 1);
    p =  pow(1-s, 3)*a.block(0, 0, 2, 1) + 3*pow(1-s, 2)*s*a.block(0, 1, 2, 1) + 3*pow(1-s, 1)*s*s*a.block(0, 2, 2, 1)  + s*s*s*a.block(0, 3, 2, 1);
    //cout << "DBG 99900 " << p.transpose(); cin.get();
    return p;
}

MatrixXd MPC_dist::bezierd2d(double s, MatrixXd a)
{
    MatrixXd dp; dp.setZero(2, 1);
    dp =  (-3*pow(s-1, 2))*a.block(0, 0, 2, 1) + (s*(2*s-2)*3+3*pow(s-1, 2))*a.block(0, 1, 2, 1) + (-s*(3*s-3)*2-3*pow(s, 2))*a.block(0, 2, 2, 1)  + 3*pow(s, 2)*a.block(0, 3, 2, 1);
    return dp;
}