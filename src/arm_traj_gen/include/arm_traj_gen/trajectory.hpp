#pragma once

#include <Eigen/Dense>


class RobotTrajectoryGeneration
{
    public:
        explicit RobotTrajectoryGeneration(const Eigen::VectorXd& q_start, const Eigen::VectorXd& q_end, double duration);
        Eigen::VectorXd getPosition(double t);
        Eigen::VectorXd getVelocity(double t);
        Eigen::VectorXd getAcceleration(double t);
    
    private:
        void computeCoefficients();
        Eigen::VectorXd q_start_;
        Eigen::VectorXd q_end_;

        double duration_;
        Eigen::MatrixXd coefficients_;
        double clampTime(double t);
};
        
        