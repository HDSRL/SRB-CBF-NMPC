%% Swift - Light Weight Interior Point Solver
% ---------------------------------------------------
% ---------------------------------------------------
%
%   [x,info] = Swift_cmex(P,c,A,b,G,h,sigma_d,Permut)
% 
%        minimize    0.5*x'Px + c'x
%        subject to  Ax = b
%                    Gx <= h
%
% ---------------------------------------------------
%  [x,info] = Swift_cmex(P,c,G,h,sigma_d,Permut) Solves a Quadratic of the form
%
%       minimize    0.5*x'Px + c'x
%       subject to  Gx <= h    
%
% ---------------------------------------------------
%
%     INPUT arguments:
% 
%         P is a sparse matrix of dimension (n,n)
%
%         c is a dense column vector of size n
%
%         A is a sparse matrix of size (p,n) ; p represents the number of
%         equality constraints
%  
%         b is a dense column vector of size p
%
%         G is a sparse matrix of size (m,n) ; m represents the number of
%         inequality constraints
% 
%         h is a dense column vector of size m
%         
%         sigma_d is a scalar value
%
%         Permut is permutation matrix obtained as
%         
%         KKT = [P A' G';
%                A 0  0;
%                G 0  -I];
%        Permut = symamd(KKT) - 1;
%        
%   Note: Permutation matrix shold be in zero based notation; otherwise the
%   solver doesnot work
%
%   [x,info] = Swift_cmex(P,c,A,b,G,h,Permut);
%   
%   x represents the solution 
%   info has five fileds
%       -> Exit Flag : 0 : Optimal Solution Found
%                    : 1 : Failure in factorising KKT matrix
%                    : 2 : Maximum Number of Iterations Reached
%                    : 3 : Unkwon Problem in Solver
%      -> Iterations : Number of Iterations
%      -> Setup Time : Invloves setting up QP; solving for initial guess
%      -> Solve Time : Solution Time in ms
%      -> KKT Time   : Time taken for Matrix Factorizations
%
%
%
% Copyright (C) Abhishek Pandala [pandala2@illinois.edu], Hae Won Park [haewon@illinois.edu], Yanran Ding [yding35@illinois.edu]
% Dynamic Robotics Lab, Department of Mechanical Science and Engineering, University of Illinois at Urbana - Champaign, USA
