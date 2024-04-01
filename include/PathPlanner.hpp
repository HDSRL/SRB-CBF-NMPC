#ifndef PATHPLANNER_HPP
#define PATHPLANNER_HPP

#include <Eigen/Dense>
#include <vector>

class PathPlanner {
public:
    PathPlanner();
    void setHLReference(const Eigen::MatrixXd& positionReference, const Eigen::MatrixXd& velocityReference);
    void planPath();
    Eigen::MatrixXd getPath() const;

private:
    Eigen::MatrixXd hlPositionCommands;
    Eigen::MatrixXd hlVelocityCommands;
    Eigen::MatrixXd path;

    void computePath();
};

#endif // PATHPLANNER_HPP
