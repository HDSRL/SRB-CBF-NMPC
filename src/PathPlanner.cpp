#include "PathPlanner.hpp"

PathPlanner::PathPlanner() {
    // Constructor implementation (if needed)
}

void PathPlanner::setHLReference(const Eigen::MatrixXd& positionCommands, const Eigen::MatrixXd& velocityCommands) {
    hlPositionCommands = positionCommands;
    hlVelocityCommands = velocityCommands;
}

void PathPlanner::planPath() {
    computePath();
}

Eigen::MatrixXd PathPlanner::getPath() const {
    return path;
}

void PathPlanner::computePath() {
    size_t numSteps = Pr_refined_.cols(); // Columns represent time steps
    size_t numAgents = Pr_refined_.rows() / 2; // Each agent has 2 rows (x and y)
    path.resize(12 * numAgents, numSteps); // Adjusted for multiple agents

    for (size_t agent = 0; agent < numAgents; ++agent) {
        double theta = 0.0; // Initial heading angle for each agent

        for (size_t i = 0; i < numSteps; ++i) {
            // Indexing for each agent's x and y positions and velocities
            size_t x_idx = agent * 2;
            size_t y_idx = agent * 2 + 1;

            // Set position states
            path(0 + agent * 12, i) = Pr_refined_(x_idx, i); // x_pos
            path(1 + agent * 12, i) = Pr_refined_(y_idx, i); // y_pos
            path(2 + agent * 12, i) = 0; // z_pos, assuming flat terrain

            // Set velocity states
            path(3 + agent * 12, i) = Prd_refined_(x_idx, i); // x_vel
            path(4 + agent * 12, i) = Prd_refined_(y_idx, i); // y_vel
            path(5 + agent * 12, i) = 0; // z_vel, assuming flat terrain

            // Compute theta (heading angle) based on velocity direction
            if (i > 0) {
                theta = std::atan2(Prd_refined_(y_idx, i), Prd_refined_(x_idx, i));
            }
            path(6 + agent * 12, i) = theta; // theta

            // Compute angular velocity (omega) as change in theta over time
            if (i > 0) {
                path(11 + agent * 12, i) = (path(6 + agent * 12, i) - path(6 + agent * 12, i-1)) / 0.001; // Assuming time step of 0.001s
            } else {
                path(11 + agent * 12, i) = 0; // Initial angular velocity is zero
            }

            // Placeholder for gamma, phi, and their derivatives (set to zero)
            for (int j = 7; j <= 10; ++j) {
                path(j + agent * 12, i) = 0; // gamma, phi, dot{gamma}, dot{phi}
            }
        }
    }
}

