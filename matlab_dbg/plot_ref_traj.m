clearvars; close all;
MPC = readmatrix('ref_taj_MPC.txt')
% LOCO = readmatrix('ref_taj_LOCO.txt')
figure(1)
clf
plot(MPC(1, :), MPC(2, :), 'linestyle', 'none', 'marker', 'o');
hold on
% plot(LOCO(1, :), LOCO(2, :), 'linestyle', 'none', 'marker', 'd');
Pobs = [ 1,  1,  2.5,    100,  100,  100,  100,  100,  100
        -0, -1.5, -0.35,   -1, -3, -3.5, -4.0, -2.5, 0];
plot(Pobs(1,:), Pobs(2,:), 'x')


figure(2)
clf
vel = readmatrix('vel_traj_MPC.txt')
time = 1:size(vel,2);
plot(time(20:end-20), vel(1, 20:end-20));
hold on
plot(time(20:end-20), vel(2, 20:end-20));
% plot(time, vel(2, :));

% figure(3)
% clf
% plot(time(20:end-20), vecnorm(vel(1:2, 20:end-20)),2,1);