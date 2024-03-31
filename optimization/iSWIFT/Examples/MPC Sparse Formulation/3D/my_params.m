function [g_param,m_param,J_param,p_foot] = my_params()
g_param = 9.81;m_param = 4.5;
Jy = 4.5*0.15^2;
Jx = 4.5*0.1^2;
Jz = 4.5*0.15^2;
J_param = diag([Jx,Jy,Jz]);
p_foot = [0.15 0.15 -0.15 -0.15;-0.07 0.07 -0.07 0.07;0 0 0 0];
