-> The matlab mex interface doesnot perform error checking on the input arguments.
-> Please make sure the arguments are proper(in dimension)
-> The solver solves QP of the form

	0.5*x'Px + q'x
	Ax = b
	Gx<=h

-> You can invoke the solver using the following arguments

	[X,info] = Swift_cmex(sparse(P),c',sparse(A),beq',sparse(G),h',sigma_d,int64(Permut)');
	
	where Permut = symamd(Phi) - 1;
	
	Phi = [P A' G'; A zeros(p,p+m);G zeros(m,p) -1*eye(m,m)];

-> The result of the solver is the solution and info structure with following details
	info -> EXITCODE
	info -> Iterations
	info -> SetupTime
	info -> SolveTime
-> Total RunTime = info->SetupTime + info->SolveTime

-> Although the solver is capable of handling QP with no equality constraints, the interface is not developed so.

-> The file Ordering_Generator.m in $...\matlab\Ordering\$ shows how to generate a zero-based Permutation matrix using amd in matlab