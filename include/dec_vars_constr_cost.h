/******************************************************************************
Copyright (c) 2017, Alexander W Winkler. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
******************************************************************************/

/**
 *  @file test_vars_constr_cost.h
 *
 *  @brief Example to generate a solver-independent formulation for the problem, taken
 *  from the IPOPT cpp_example.
 *
 *  The example problem to be solved is given as:
 *
 *      min_x f(x) = -(x1-2)^2
 *      s.t.
 *           0 = x0^2 + x1 - 1
 *           -1 <= x0 <= 1
 *
 * In this simple example we only use one set of variables, constraints and
 * cost. However, most real world problems have multiple different constraints
 * and also different variable sets representing different quantities. This
 * framework allows to define each set of variables or constraints absolutely
 * independently from another and correctly stitches them together to form the
 * final optimization problem.
 *
 * For a helpful graphical overview, see:
 * http://docs.ros.org/api/ifopt/html/group__ProblemFormulation.html
 */

#include <ifopt/variable_set.h>
#include <ifopt/constraint_set.h>
#include <ifopt/cost_term.h>
#include "pack_nlp_data.h"

#define USE_AS_CONSTR 4
#define VEL_CONSTR 8

namespace ifopt {
using Eigen::Vector2d;
using Eigen::VectorXd;
using std::cin;
using std::cout;

class ExVariables : public VariableSet {
public:
  // Every variable set has a name, here "var_set1". this allows the constraints
  // and costs to define values and Jacobians specifically w.r.t this variable set.
  ExVariables() : ExVariables("var_set1") {};
  ExVariables(const std::string& name) : VariableSet(72, name)    // Not invoked in my implementation, no need to worry about 72
  {
    // cout << "DEBUG: 000030 ENTER constructor ExVariables" << std::endl;
    dec_vars_.setZero(72); // Initialize dec_vars_ to zero
    // cout << "DEBUG: 000031 EXIT constructor ExVariables" << std::endl;
  }

  ExVariables(std::shared_ptr<nlpTuple> nlp_data, int num_dec_vars, std::string name) : VariableSet(num_dec_vars, name) 
  {

    // cout << "DEBUG: 000022 ENTER CONSTRUCTOR ExVariables()" << std::endl;
    // the initial values where the NLP starts iterating from
    //cout << "Dec Vars: #0"; cin.get();
    n_dec_vars_ = num_dec_vars;

    //cout << "Dec Vars: #001"; cin.get();
    dec_vars_.setZero(n_dec_vars_);
    //cout << "Dec Vars: #001"; cin.get();
    Q_qp_ = nlp_data->Q_qp;
    f_qp_ = nlp_data->f_qp;
    Aeq_ = nlp_data->Aeq;
    beq_ = nlp_data->beq;
    Gineq_ = nlp_data->Gineq;
    hineq_ = nlp_data->hineq;
    x0_ = nlp_data->x0;
    //cout << x0 << "\n"; cin.get();
    dec_vars_ = x0_;
    // cout << "DECISION VARIABLES INITIAL CONDITION" << dec_vars_ << "\n"; cin.get();
    //cout << "Dec_vars1: " << dec_vars_ << std::endl; std::cin.get();
    // cout << "DEBUG: 000023 EXIT CONSTRUCTOR ExVariables()" << std::endl;
  }

  void SetVariables(const VectorXd& x) override
  {
    // cout << "DEBUG: 000024 ENTER SetVariables()" << std::endl;
    dec_vars_ = x;   //remove this comment
    // cout << "DEBUG: 000025 EXIT SetVariables()" << std::endl;
  };

  // Here is the reverse transformation from the internal representation to
  // to the Eigen::Vector
  VectorXd GetValues() const override
  {
    // cout << "DEBUG: 000026 ENTER GetValues() ExVariables" << std::endl;
    // cout << "DEBUG: 000027 EXIT GetValues() ExVariables" << std::endl;
    return dec_vars_;    //remove this comment
    
  };

  // Each variable has an upper and lower bound set here
  VecBound GetBounds() const override
  {
    // cout << "DEBUG: 000028 Enter GetBounds() ExVariables" << std::endl;
    VecBound bounds(GetRows());
    for (size_t i = 0; i < n_dec_vars_; i++)
    {
      bounds.at(i) = NoBound;
    }

    // cout << "DEBUG: 000029 EXIT GetValues() ExVariables" << std::endl;
    return bounds;
  }

private:
  VectorXd dec_vars_;
  int n_dec_vars_;
  MatrixXd Q_qp_;
  MatrixXd f_qp_;
  MatrixXd Aeq_;
  MatrixXd beq_; 
  MatrixXd Gineq_;
  MatrixXd hineq_;
  VectorXd x0_;
};

class EqConstraints : public ConstraintSet {
public:
  EqConstraints() : EqConstraints("constraint1") {}

  EqConstraints(const std::string& name) : ConstraintSet(1, name) {}

  EqConstraints(std::shared_ptr<nlpTuple> nlp_data, int num_dec_vars, std::string name) : ConstraintSet(nlp_data->beq.rows(), name) 
  {
    // cout << "DEBUG: 000001EQ \n" << std::endl;
    //cout << "Dec Vars: #004"; cin.get();
    n_dec_vars_ = num_dec_vars;
    Q_qp_ = nlp_data->Q_qp;
    f_qp_ = nlp_data->f_qp;
    Aeq_ = nlp_data->Aeq;
    beq_ = nlp_data->beq;
    Gineq_ = nlp_data->Gineq;
    hineq_ = nlp_data->hineq;
    // cout << "DEBUG: 000002EQ \n" << std::endl;
  }

  VectorXd GetValues() const override
  {
    // cout << "DEBUG: 000011 ENTER GetValues() EQ" << std::endl;
    VectorXd g(GetRows());
    VectorXd x = GetVariables()->GetComponent("var_set1")->GetValues();
    g = Aeq_*x;    //Remove this comment
    // cout << "DEBUG: 000012 EXIT GetValues() EQ" << std::endl;
    return g;
  };

  VecBound GetBounds() const override
  {
    // cout << "DEBUG: 000013 ENTER GetBounds() EQ" << std::endl;
    //cout << "Dec Vars (1): #005"; //cin.get();  
    VecBound b(GetRows()); 
    for (size_t i = 0; i < beq_.rows(); i++)
    {
      b.at(i) = Bounds(beq_(i), beq_(i));
    }
    // cout << "DEBUG: 000014 EXIT GetBounds() EQ" << std::endl;
    return b;
  }

  void FillJacobianBlock (std::string var_set, Jacobian& jac_block) const override
  {
    // cout << "DEBUG: 000009 ENTER JACBLOCK EQ" << std::endl;
    //if (var_set == "var_set1") {
      //Vector2d x = GetVariables()->GetComponent("var_set1")->GetValues();
      for (size_t i = 0; i < Aeq_.rows(); i++)
      {
        for (size_t j = 0; j < Aeq_.cols(); j++)
        {
          // cout << "DEBUG:  JAC EQ CNSTR\n";
          jac_block.coeffRef(i, j) = Aeq_(i,j);
        }
      }
      // cout << "DEBUG: 000010 EXIT JACBLOCK EQ" << std::endl;
    //}
  }
private:
  int n_dec_vars_;
  MatrixXd Q_qp_;
  MatrixXd f_qp_;
  MatrixXd Aeq_;
  MatrixXd beq_; 
  MatrixXd Gineq_;
  MatrixXd hineq_;
};

class NeqConstraints : public ConstraintSet {
public:
  NeqConstraints() : NeqConstraints("constraint2") {}

  NeqConstraints(const std::string& name) : ConstraintSet(1, name) {}

  NeqConstraints(std::shared_ptr<nlpTuple> nlp_data, int num_dec_vars, std::string name) : ConstraintSet(nlp_data->hineq.rows()+USE_AS_CONSTR+VEL_CONSTR, name) 
  {
    // cout << "DEBUG: 000006 ENTER Constructor NEQ" << std::endl;
    //cout << "Dec Vars: #004"; cin.get();
    n_dec_vars_ = num_dec_vars;
    cout << "0  DEBUGGING THE CONSTR of NEQ_CONSTR\n"; //cin.get();
    Gineq_.setZero(nlp_data->Gineq.rows()+USE_AS_CONSTR+VEL_CONSTR, nlp_data->Gineq.cols());
    hineq_.setZero(nlp_data->hineq.rows()+USE_AS_CONSTR+VEL_CONSTR, 1);

    Q_qp_ = nlp_data->Q_qp;
    f_qp_ = nlp_data->f_qp;
    Aeq_ = nlp_data->Aeq;
    beq_ = nlp_data->beq;
    obs_x = nlp_data->closest_obs(0);
    obs_y = nlp_data->closest_obs(1);
    
    Gineq_.block(0,0,nlp_data->Gineq.rows(),nlp_data->Gineq.cols()) = nlp_data->Gineq;
    hineq_.block(0,0,nlp_data->Gineq.rows(),1) = nlp_data->hineq;
    cout << "0  EXIT DEBUG THE CONSTR of NEQ_CONSTR\n"; //cin.get();
    // cout << "DEBUG: 000007 EXIT Constructor NEQ" << std::endl;
  }

  VectorXd GetValues() const override
  {
    // cout << "DEBUG: 000005 ENTER GetValues NEQ" << std::endl;
    // cout << "5   DEBG calculate constr value\n";
    VectorXd g(GetRows());
    // cout << "5.01\n";
    VectorXd x = GetVariables()->GetComponent("var_set1")->GetValues();
    // cout << "5.02\n";
    //g.block(0,0,Gineq_.rows()-USE_AS_CONSTR,1) = Gineq_.block(0,0,Gineq_.rows()-USE_AS_CONSTR,Gineq_.cols())*x;    //Remove this comment
    g.block(0,0,Gineq_.rows()-USE_AS_CONSTR-VEL_CONSTR,1) = Gineq_.block(0,0,Gineq_.rows()-USE_AS_CONSTR-VEL_CONSTR,Gineq_.cols())*x;
    // cout << "5.03\n";

    if (USE_AS_CONSTR-3) { 
      // g(Gineq_.rows()-USE_AS_CONSTR) = sqrt(pow(x(0)-obs_x,2)+pow(x(2)-obs_y,2)) + x(n_dec_vars_-1);
      // g(Gineq_.rows()-USE_AS_CONSTR+1) = sqrt(pow(x(4)-obs_x,2)+pow(x(6)-obs_y,2)) + x(n_dec_vars_-1);
      // g(Gineq_.rows()-USE_AS_CONSTR+2) = sqrt(pow(x(8)-obs_x,2)+pow(x(10)-obs_y,2)) + x(n_dec_vars_-1);
      // g(Gineq_.rows()-USE_AS_CONSTR+3) = sqrt(pow(x(12)-obs_x,2)+pow(x(14)-obs_y,2)) + x(n_dec_vars_-1);
      g(Gineq_.rows()-USE_AS_CONSTR-VEL_CONSTR) = pow(x(0)-obs_x,2)+pow(x(2)-obs_y,2)*1 + x(n_dec_vars_-1);
      g(Gineq_.rows()-USE_AS_CONSTR-VEL_CONSTR+1) = pow(x(4)-obs_x,2)+pow(x(6)-obs_y,2)*1 + x(n_dec_vars_-1);
      g(Gineq_.rows()-USE_AS_CONSTR-VEL_CONSTR+2) = pow(x(8)-obs_x,2)+pow(x(10)-obs_y,2)*1 + x(n_dec_vars_-1);
      g(Gineq_.rows()-USE_AS_CONSTR-VEL_CONSTR+3) = pow(x(12)-obs_x,2)+pow(x(14)-obs_y,2)*1 + x(n_dec_vars_-1);

      if(VEL_CONSTR == 8)
      {
      // Velocity saturation constraints - x
      g(Gineq_.rows()-VEL_CONSTR)   = x(1);
      g(Gineq_.rows()-VEL_CONSTR+1) = x(5);
      g(Gineq_.rows()-VEL_CONSTR+2) = x(9); // 8 10
      g(Gineq_.rows()-VEL_CONSTR+3) = x(13); // 12 14

      // Velocity saturation constraints - y
      g(Gineq_.rows()-VEL_CONSTR+4) = x(3);
      g(Gineq_.rows()-VEL_CONSTR+5) = x(7);
      g(Gineq_.rows()-VEL_CONSTR+6) = x(11); // 8 10
      g(Gineq_.rows()-VEL_CONSTR+7) = x(15); // 12 14
      }
    }

    return g;
  };

  VecBound GetBounds() const override
  {
    VecBound b(GetRows());
    int last_i = 0;
    for (size_t i = 0; i < hineq_.rows()-USE_AS_CONSTR-VEL_CONSTR; i++)
    {
      b.at(i) = Bounds(-inf, hineq_(i));
      last_i = i;
    }
    // ADD THE BOUND FOR DISTANCE CONSTRAINT -------------------------------------------------------- NEW
    // cout << "2   DEBUGGING THE CONSTR of NEQ_CONSTR"; cin.get();
    if(USE_AS_CONSTR-3)
    {
      b.at(last_i+1) = Bounds(epsilon, inf);
      b.at(last_i+2) = Bounds(epsilon, inf);
      b.at(last_i+3) = Bounds(epsilon, inf);
      b.at(last_i+4) = Bounds(epsilon, inf);

      if(VEL_CONSTR == 8)
      {
      float x_vel_sat = 0.35;
      float y_vel_sat = 0.35;

      // Vel saturation
      b.at(last_i+5) = Bounds(-x_vel_sat, x_vel_sat);
      b.at(last_i+6) = Bounds(-x_vel_sat, x_vel_sat);
      b.at(last_i+7) = Bounds(-x_vel_sat, x_vel_sat);
      b.at(last_i+8) = Bounds(-x_vel_sat, x_vel_sat);
      b.at(last_i+9) = Bounds(-y_vel_sat, y_vel_sat);
      b.at(last_i+10) = Bounds(-y_vel_sat, y_vel_sat);
      b.at(last_i+11) = Bounds(-y_vel_sat, y_vel_sat);
      b.at(last_i+12) = Bounds(-y_vel_sat, y_vel_sat);
      }
    }
    cout << "3  EXIT DEBG calculate bound value\n";
    // cout << "2   DEBUGGING THE CONSTR of NEQ_CONSTR"; cin.get();
    // cout << "DEBUG: 000004 EXIT GetBounds NEQ " << std::endl;
    return b;
  }

  void FillJacobianBlock (std::string var_set, Jacobian& jac_block) const override
  {
      // cout << "DEBUG: 000020 ENTER FillJacBlock INEQ" << std::endl;
    //if (var_set == "var_set1") {
      cout << "1  Jacob\n";

      VectorXd x = GetVariables()->GetComponent("var_set1")->GetValues();
      int i_last = 0;
      for (size_t i = 0; i < Gineq_.rows()-USE_AS_CONSTR-VEL_CONSTR; i++)
      {
        for (size_t j = 0; j < Gineq_.cols(); j++)
        {
          // cout << "DEBUG:  JAC INEQ CNSTR\n";
          jac_block.coeffRef(i, j) = Gineq_(i,j); // Derivative of ith constraint with respect to jth dec var
        }
        i_last = i;
      }
      
      // ADD THE DERIVATIVE for the NEW SAFE DIST CONSTRAINT HERE
      if (USE_AS_CONSTR-3) { 
        // jac_block.coeffRef(i_last+1, 0) = (x(0)-obs_x)/(sqrt(pow(x(0)-obs_x,2)+pow(x(2)-obs_y,2)));
        // jac_block.coeffRef(i_last+1, 2) = (x(2)-obs_y)/(sqrt(pow(x(0)-obs_x,2)+pow(x(2)-obs_y,2)));
        // jac_block.coeffRef(i_last+1, n_dec_vars_-1) = 1; 

        // jac_block.coeffRef(i_last+2, 4) = -(x(4)-obs_x)/(sqrt(pow(x(4)-obs_x,2)+pow(x(6)-obs_y,2)));
        // jac_block.coeffRef(i_last+2, 6) = -(x(6)-obs_y)/(sqrt(pow(x(4)-obs_x,2)+pow(x(6)-obs_y,2)));
        // jac_block.coeffRef(i_last+2, n_dec_vars_-1) = 1; 

        // jac_block.coeffRef(i_last+3, 8) = -(x(8)-obs_x)/(sqrt(pow(x(8)-obs_x,2)+pow(x(10)-obs_y,2)));
        // jac_block.coeffRef(i_last+3, 10) = -(x(10)-obs_y)/(sqrt(pow(x(8)-obs_x,2)+pow(x(10)-obs_y,2)));
        // jac_block.coeffRef(i_last+3, n_dec_vars_-1) = 1; 

        // jac_block.coeffRef(i_last+4, 12) = (x(12)-obs_x)/(sqrt(pow(x(12)-obs_x,2)+pow(x(14)-obs_y,2)));
        // jac_block.coeffRef(i_last+4, 14) = (x(14)-obs_y)/(sqrt(pow(x(12)-obs_x,2)+pow(x(14)-obs_y,2)));
        // jac_block.coeffRef(i_last+4, n_dec_vars_-1) = 1; 

        // using squared distance
        jac_block.coeffRef(i_last+1, 0) = 2*(x(0)-obs_x);
        jac_block.coeffRef(i_last+1, 2) = 2*(x(2)-obs_y);
        jac_block.coeffRef(i_last+1, n_dec_vars_-1) = 1; 

        jac_block.coeffRef(i_last+2, 4) = 2*(x(4)-obs_x);
        jac_block.coeffRef(i_last+2, 6) = 2*(x(6)-obs_y);
        jac_block.coeffRef(i_last+2, n_dec_vars_-1) = 1; 

        jac_block.coeffRef(i_last+3, 8) = 2*(x(8)-obs_x);
        jac_block.coeffRef(i_last+3, 10) = 2*(x(10)-obs_y);
        jac_block.coeffRef(i_last+3, n_dec_vars_-1) = 1; 

        jac_block.coeffRef(i_last+4, 12) = 2*(x(12)-obs_x);
        jac_block.coeffRef(i_last+4, 14) = 2*(x(14)-obs_y);
        jac_block.coeffRef(i_last+4, n_dec_vars_-1) = 1;

        if(VEL_CONSTR == 8)
        {
          jac_block.coeffRef(i_last+5, 1) = 1;
          jac_block.coeffRef(i_last+6, 5) = 1; 
          jac_block.coeffRef(i_last+7, 9) = 1; 
          jac_block.coeffRef(i_last+8, 13) = 1; 

          jac_block.coeffRef(i_last+9, 3) = 1; 
          jac_block.coeffRef(i_last+10, 7) = 1; 
          jac_block.coeffRef(i_last+11, 11) = 1; 
          jac_block.coeffRef(i_last+12, 15) = 1; 
        }
      }
      cout << "1  EXIT  Jacob\n";// cin.get();
      // cout << "DEBUG: 000021 EXIT FillJacBlock INEQ" << std::endl;
    //}
  }
private:
  double obs_x;
  double obs_y;
  // const float obs_x =  1.0;
  // const float obs_y = -0.0;
  const float epsilon = 1.9; //for robot to obstacle // 2.55 for sim1
  // const float epsilon = 2.2; //for robot to robot
  int n_dec_vars_;
  MatrixXd Q_qp_;
  MatrixXd f_qp_;
  MatrixXd Aeq_;
  MatrixXd beq_; 
  MatrixXd Gineq_;
  MatrixXd hineq_;
};

class ExCost: public CostTerm {
public:
  ExCost() : ExCost("cost_term1") {}
  ExCost(const std::string& name) : CostTerm(name) {}
  ExCost(std::shared_ptr<nlpTuple> nlp_data, int num_dec_vars, std::string name) : CostTerm(name) 
  {
    n_dec_vars_ = num_dec_vars;
    Q_qp_ = nlp_data->Q_qp;
    f_qp_ = nlp_data->f_qp;
  }

  double GetCost() const override
  {
    VectorXd x = GetVariables()->GetComponent("var_set1")->GetValues();
    VectorXd tmp = 0.5*x.transpose()*Q_qp_*x+f_qp_.transpose()*x; // adding the nonlinear cost
    return tmp(0);
  };

  void FillJacobianBlock (std::string var_set, Jacobian& jac) const override
  {
    VectorXd x = GetVariables()->GetComponent("var_set1")->GetValues();
    VectorXd tmp = Q_qp_*x+f_qp_;
    for (size_t i = 0; i < n_dec_vars_; i++)
    {
      jac.coeffRef(0,i) = tmp(i);   // CHECK IF CORRECT
    }
  }

private:
  int n_dec_vars_;
  MatrixXd Q_qp_;
  MatrixXd f_qp_;
  MatrixXd Aeq_;
  MatrixXd beq_; 
  MatrixXd Gineq_;
  MatrixXd hineq_;
};

class AugCost: public CostTerm {
public:
  AugCost() : AugCost("cost_term2") {}
  AugCost(const std::string& name) : CostTerm(name) {}
  AugCost(std::shared_ptr<nlpTuple> nlp_data, int num_dec_vars, std::string name) : CostTerm(name) 
  {
    n_dec_vars_ = num_dec_vars;
    Q_qp_ = nlp_data->Q_qp;
    f_qp_ = nlp_data->f_qp;
  }

  double GetCost() const override
  {
    VectorXd x = GetVariables()->GetComponent("var_set1")->GetValues();
    // x(0) = x(0); x(0+4) = x(0+4); x(0+8) = x(0+8); x(0+12) = x(0+12);
    float d0 = sqrt( pow(x(0)    - ox, 2) + pow( x(2)    - oy, 2) );
    float d1 = sqrt( pow(x(0+4)  - ox, 2) + pow( x(2+4)  - oy, 2) );
    float d2 = sqrt( pow(x(0+8)  - ox, 2) + pow( x(2+8)  - oy, 2) );
    float d3 = sqrt( pow(x(0+12) - ox, 2) + pow( x(2+12) - oy, 2) );

    double nonlinear_cost = 0.0;
    // nonlinear_cost =  mu*0.5* pow(MAX(0, eps - d0), 2) + mu*0.5* pow(MAX(0, eps - d1), 2) + mu*0.5* pow(MAX(0, eps - d2), 2) + mu*0.5* pow(MAX(0, eps - d3), 2);
    nonlinear_cost =  mu*0.5* pow(MAX(0, eps - d0), 2)+mu*0.5* pow(MAX(0, eps - d1), 2)+mu*0.5* pow(MAX(0, eps - d2), 2)+mu*0.5* pow(MAX(0, eps - d3), 2);
    return nonlinear_cost;
  };

  void FillJacobianBlock (std::string var_set, Jacobian& jac) const override
  {
    VectorXd x = GetVariables()->GetComponent("var_set1")->GetValues();
    x(0) = x(0); x(0+4) = x(0+4); x(0+8) = x(0+8); x(0+12) = x(0+12);
    float d0 = sqrt( pow(x(0)    - ox, 2) + pow( x(2)    - oy, 2) );
    float d1 = sqrt( pow(x(0+4)  - ox, 2) + pow( x(2+4)  - oy, 2) );
    float d2 = sqrt( pow(x(0+8)  - ox, 2) + pow( x(2+8)  - oy, 2) );
    float d3 = sqrt( pow(x(0+12) - ox, 2) + pow( x(2+12) - oy, 2) );

    jac.coeffRef(0,0)  = mu*(eps-d0)*(x(0) - ox)/d0;
    jac.coeffRef(0,2)  = mu*(eps-d0)*(x(2) - oy)/d0;
    
    // jac.coeffRef(0,4)  = mu*(eps-d0)*(x(4) - ox)/d1;
    // jac.coeffRef(0,6)  = mu*(eps-d0)*(x(6) - oy)/d1;

    // jac.coeffRef(0,8)  = mu*(eps-d0)*(x(8) - ox)/d2;
    // jac.coeffRef(0,10) = mu*(eps-d0)*(x(10) - oy)/d2;

    // jac.coeffRef(0,12) = mu*(eps-d3)*(x(12) - ox)/d3;
    // jac.coeffRef(0,14) = mu*(eps-d3)*(x(14) - oy)/d3;
  }

private:
  //Eigen::VectorXd dec_vars_;
  float ox = 1.0;
  float oy =-0;
  double mu = 1e6; 
  float eps = 1;
  int n_dec_vars_;
  MatrixXd Q_qp_;
  MatrixXd f_qp_;
  MatrixXd Aeq_;
  MatrixXd beq_; 
  MatrixXd Gineq_;
  MatrixXd hineq_;
};

} // namespace opt