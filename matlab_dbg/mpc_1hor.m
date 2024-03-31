clear all; close all;

figure(1)
MPC_sol = dlmread('MPC_loco.txt');
COM_des_traj = dlmread('COM_loco.txt');


X = reshape(MPC_sol(1:16), 4, 4);
U = reshape(MPC_sol(17:17+7), 2, 4);
plot(X(1,:), X(3,:), 'LineWidth', 2, 'Marker', 's')
hold on
plot(U(1,:), U(2,:), 'Marker', 'd')
plot(X(1,1), X(3,1), 'Marker', 'o', 'MarkerSize', 20)
plot(COM_des_traj(1,:), COM_des_traj(3,:), 'LineWidth', 2)
plot(COM_des_traj(1,1), COM_des_traj(3,1), 'Marker', 'o')
legend('MPC','COP', 'MPC_0','DES', 'DES_0')

figure(2)
MPC_sol = dlmread('MPC_dist.txt');
COM_des_traj = dlmread('COM_dist.txt');


X = reshape(MPC_sol(1:16), 4, 4);
U = reshape(MPC_sol(17:17+7), 2, 4);
plot(X(1,:), X(3,:), 'LineWidth', 2, 'Marker', 's')
hold on
plot(U(1,:), U(2,:), 'Marker', 'd')
plot(X(1,1), X(3,1), 'Marker', 'o', 'MarkerSize', 20)
plot(COM_des_traj(1,:), COM_des_traj(3,:), 'LineWidth', 2)
plot(COM_des_traj(1,1), COM_des_traj(3,1), 'Marker', 'o')
legend('MPC','COP', 'MPC_0','DES', 'DES_0')