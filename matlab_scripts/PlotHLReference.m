clc; clear all; close all;
% Define file paths
addpath(genpath(pwd))
pathFile = '../Sim_Outputs/HLPath.txt';
velocityFile = '../Sim_Outputs/HLVelocity.txt';

% Load data
Pr_refined = load(pathFile);
Prd_refined = load(velocityFile);

% Assuming there are 4 agents and each has 2 rows (one for X and one for Y)
numAgents = size(Pr_refined, 1)/2; % number of agents

% Plot positions for each agent
figure;
hold on;
for i = 1:numAgents
    % Assuming the first row is X and the second row is Y for each agent
    plot(Pr_refined(2*i-1, 1:end-10), Pr_refined(2*i, 1:end-10), 'LineWidth', 2, 'DisplayName', ['Agent ' num2str(i) ' Position']);
end
title('Agent Positions');
xlabel('X Position');
ylabel('Y Position');
legend('show');
hold off;

% Plot velocities for each agent
figure;
hold on;
for i = 1:numAgents
    % Assuming the first row is X velocity and the second row is Y velocity for each agent
    % Calculate velocity magnitude as sqrt(vx^2 + vy^2)
    vx = Prd_refined(2*i-1, 1:end-10);
    vy = Prd_refined(2*i, 1:end-10);
    velocityMagnitude = sqrt(vx.^2 + vy.^2);
    plot(1:length(velocityMagnitude), velocityMagnitude, 'LineWidth', 2, 'DisplayName', ['Agent ' num2str(i) ' Velocity']);
end
title('Agent Velocities');
xlabel('Time Step');
ylabel('Velocity Magnitude');
legend('show');
hold off;

autoArrangeFigures();