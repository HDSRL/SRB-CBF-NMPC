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
figure(1);
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
figure(2);
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

path = computePath(Pr_refined, Prd_refined)

%% Assuming computePath has been called and 'path' matrix is available
numAgents = size(path, 1) / 12; % Assuming 12 states per agent
numSteps = size(path, 2);

agentWidth = 0.3;
agentHeight = 0.2;

figure(3);
hold on;
axis equal;
grid on;
xlabel('X Position');
ylabel('Y Position');
title('Agent Path Animation');
autoArrangeFigures();

% Calculate XY limits based on the entire path
xLimits = [min(path(1:12:end, :), [], 'all'), max(path(1:12:end, :), [], 'all')];
yLimits = [min(path(2:12:end, :), [], 'all'), max(path(2:12:end, :), [], 'all')];
xlim(xLimits + [-0.5, 0.5]); % Adding some padding
ylim(yLimits + [-0.5, 0.5]);

% Plot paths and animate agents
for i = 1:numSteps
    cla; % Clear for redrawing
    figure(3);
    
    % Redraw paths for clarity
    for agent = 1:numAgents
        plot(path(1 + (agent-1)*12, 1:i), path(2 + (agent-1)*12, 1:i), 'b--');
    % end
    % 
    % % Draw each agent with updated position and heading
    % for agent = 1:1
        agentCenter = [path(1 + (agent-1)*12, i), path(2 + (agent-1)*12, i)];
        agentAngle = path(7 + (agent-1)*12, i);
        colors = ['r', 'g', 'b', 'k', 'm', 'y', 'c']; % Extend as needed
        colorIndex = mod(agent, length(colors)) + 1;
        drawRotatedRect(agentCenter, agentWidth, agentHeight, agentAngle, colors(colorIndex));
    end
    
    drawnow;
    pause(0.01); % Adjust for desired animation speed
end



%%

function drawRotatedRect(center, width, height, angle, color)
    % Define the rectangle corners relative to the center (0,0)
    rectCorners = [-width/2, -height/2;
                    width/2, -height/2;
                    width/2,  height/2;
                   -width/2,  height/2]';
    
    % Rotation matrix
    R = [cos(angle), -sin(angle); sin(angle), cos(angle)];
    
    % Rotate the rectangle corners
    rotatedCorners = R * rectCorners;
    
    % Translate the rectangle corners to the specified center
    rotatedCorners(1, :) = rotatedCorners(1, :) + center(1);
    rotatedCorners(2, :) = rotatedCorners(2, :) + center(2);
    
    % Close the rectangle shape by repeating the first corner at the end
    rotatedCorners(:, end+1) = rotatedCorners(:, 1);
    
    % Plot the rectangle
    plot(rotatedCorners(1, :), rotatedCorners(2, :), color, 'LineWidth', 2);
end


function path = computePath(Pr_refined_, Prd_refined_)
    numAgents = size(Pr_refined_, 1) / 2; % Each agent has 2 rows (x and y)
    numSteps = size(Pr_refined_, 2); % Columns represent time steps
    path = zeros(12 * numAgents, numSteps); % Adjusted for multiple agents
    velocityChangeThreshold = 0.1; % Threshold for considering velocity change significant

    for agent = 1:numAgents
        % Calculate the row index offset for the current agent in the path matrix
        offset = (agent - 1) * 12;

        for i = 1:numSteps
            % Set position and velocity states
            path(1 + offset:6 + offset, i) = [Pr_refined_((agent - 1) * 2 + (1:2), i); 0; Prd_refined_((agent - 1) * 2 + (1:2), i); 0];

            if i > 1
                dx = Prd_refined_(1 + (agent - 1) * 2, i);
                dy = Prd_refined_(2 + (agent - 1) * 2, i);
                velocityChange = sqrt(dx^2 + dy^2);

                if velocityChange > velocityChangeThreshold
                    theta = atan2(dy, dx);
                else
                    % If the change is below the threshold, use the previous theta
                    % This avoids unnecessary theta changes for minor adjustments
                    theta = path(7 + offset, i-1);
                end
            else
                theta = 0; % Initial theta
            end
            path(7 + offset, i) = theta; % Update theta

            % Placeholder for gamma, phi, and their derivatives (set to zero)
            path(8 + offset:11 + offset, i) = 0; % gamma, phi, dot{gamma}, dot{phi}
        end
    end
end

