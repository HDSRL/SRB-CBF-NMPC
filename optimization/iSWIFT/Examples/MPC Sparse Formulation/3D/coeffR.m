function [C1,C2,C3] = coeffR(Rop,wop,dt)


M3 = Rop*my_hat(wop);
% M3 = M3(:);


P = [1,0,0,0,0,0,0,0,0;0,0,0,1,0,0,0,0,0;0,0,0,0,0,0,1,0,0;0,1,0,0,0,0,0,0,0;0,0,0,0,1,0,0,0,0;0,0,0,0,0,0,0,1,0;0,0,1,0,0,0,0,0,0;0,0,0,0,0,1,0,0,0;0,0,0,0,0,0,0,0,1];
L = 0.5*(kron(eye(3),Rop')-(kron(Rop',eye(3))*P));

F_Rop_wop = calculate_F(Rop*wop);
N = [0 0 0;0 0 1;0 -1 0;0 0 -1;0 0 0;1 0 0;0 1 0;-1 0 0;0 0 0];

% M1 = (kron(hatMap(wop)',Rop) + kron(eye(3),Rop*hatMap(wop)))*L - (kron(eye(3),Rop)*N*F_Rop_wop);

M1 = (kron(my_hat(wop)',Rop) + kron(eye(3),M3))*L - (kron(eye(3),Rop)*N*F_Rop_wop);

M2 = kron(eye(3),Rop)*N;

C1 = eye(9) + M1*dt;
C2 = M2*dt;
C3 = M3(:)*dt;

% C1 = simplify(C1);
% C2 = simplify(C2);
% C3 = simplify(C3);
end

