function [d1 d2 d3 d4 d5] = coeffw(Rop,wop,pf,dt,J,fop,xop)

rop = pf - repmat(xop,[1,4]);

M_op = [my_hat(rop(:,1)) my_hat(rop(:,2)) my_hat(rop(:,3)) my_hat(rop(:,4))]*fop;


P = [1,0,0,0,0,0,0,0,0;0,0,0,1,0,0,0,0,0;0,0,0,0,0,0,1,0,0;0,1,0,0,0,0,0,0,0;0,0,0,0,1,0,0,0,0;0,0,0,0,0,0,0,1,0;0,0,1,0,0,0,0,0,0;0,0,0,0,0,1,0,0,0;0,0,0,0,0,0,0,0,1];
L = 0.5*(kron(eye(3),Rop')-(kron(Rop',eye(3))*P));

M4 = (- my_hat(wop)*J*wop) + (Rop'*M_op);
M8 = my_hat(J*wop)-(my_hat(wop)*J);
M5 = calculate_F(M_op)*L - (M8*calculate_F(Rop*wop));
% 
% inv_J = inv(J);
% d_M_I = zeros(3,1);
% %d_r =  repmat(d_x,[1,4]);
% 
% % C6 = zeros(3,1);
% % C7 = zeros(3,1);
% % 
% % for ct = 1:4
% %     C6 = C6 + my_hat(f_op(:,ct)) * d_x;
% %     C7 = C7 + my_hat(rop(:,ct)) * d_f(:,ct);
% % end
M6 = Rop'*[my_hat(rop(:,1)) my_hat(rop(:,2)) my_hat(rop(:,3)) my_hat(rop(:,4))];
M7 = Rop'*my_hat([sum(fop([1,4,7,10])),sum(fop([2,5,8,11])),sum(fop([3,6,9,12]))]);
% 
% C7 = Rop'*C7;
% 
% C8 = my_hat(J*wop)-(my_hat(wop)*J);
% 
% 
% wk_1 = (eye(3) + inv_J*C8*dt)*wk + (inv_J*C5*dt)*Rk(:) + (inv_J*C6*dt)*d_x + (inv_J*C7*dt)*d_f + inv_J*C4*dt;

in_J = inv(J);


d3 = (eye(3) + (in_J*M8)*dt);

d2 = in_J*M5*dt;

d1 = in_J*M7*dt;

d5 = in_J*M4*dt - d1*xop;

d4 = in_J*M6*dt;


end