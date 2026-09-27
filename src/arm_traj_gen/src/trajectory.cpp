#include "arm_traj_gen/trajectory.hpp"
#include <algorithm>
#include <stdexcept>

void RobotTrajectoryGeneration::computeCoefficients()
{
    Eigen::Index n_joints = q_start_.size();
    coefficients_.resize(n_joints, 6);

    for (Eigen::Index i = 0; i < n_joints; ++i)
    {
        double q0 = q_start_(i);
        double qf = q_end_(i);
        double T = duration_;

        coefficients_(i, 0) = q0;
        coefficients_(i, 1) = 0.0;
        coefficients_(i, 2) = 0.0;
        coefficients_(i, 3) = (10 * (qf - q0)) / (T * T * T);
        coefficients_(i, 4) = (-15 * (qf - q0)) / (T * T * T * T);
        coefficients_(i, 5) = (6 * (qf - q0)) / (T * T * T * T * T);
    }
}

RobotTrajectoryGeneration::RobotTrajectoryGeneration(const Eigen::VectorXd& q_start, const Eigen::VectorXd& q_end, double duration)
    : q_start_(q_start), q_end_(q_end), duration_(duration)
{
    if (q_start_.size() != q_end_.size())
    {
        throw std::invalid_argument("Start and end configurations must have the same size.");
    }
    if (duration_ <= 0.0)
    {
        throw std::invalid_argument("Duration must be positive.");
    }

    computeCoefficients();

}

double RobotTrajectoryGeneration::clampTime(double t)
{
    if (t>0.0)
    {
        return std::min(t, duration_);
    }
    else
    {
        return 0.0;
    }
}

Eigen::VectorXd RobotTrajectoryGeneration::getPosition(double t)
{
    t=clampTime(t);
    Eigen::Index output = q_start_.size();
    Eigen::VectorXd q(output);
    for (Eigen::Index i=0;i<output;++i)
    {
        q(i) = coefficients_(i,0) + coefficients_(i,1)*t + coefficients_(i,2)*t*t + coefficients_(i,3)*t*t*t + coefficients_(i,4)*t*t*t*t + coefficients_(i,5)*t*t*t*t*t;
    }
    return q;
}

Eigen::VectorXd RobotTrajectoryGeneration::getVelocity(double t)
{
    t=clampTime(t);
    Eigen::Index output = q_start_.size();
    Eigen::VectorXd dq(output);
    for (Eigen::Index i=0;i<output;++i)
    {
        dq(i) = coefficients_(i,1) + 2*coefficients_(i,2)*t + 3*coefficients_(i,3)*t*t + 4*coefficients_(i,4)*t*t*t + 5*coefficients_(i,5)*t*t*t*t;
    }
    return dq;
}

Eigen::VectorXd RobotTrajectoryGeneration::getAcceleration(double t)
{
    t=clampTime(t);
    Eigen::Index output = q_start_.size();
    Eigen::VectorXd ddq(output);
    for (Eigen::Index i=0;i<output;++i)
    {
        ddq(i) = 2*coefficients_(i,2) + 6*coefficients_(i,3)*t + 12*coefficients_(i,4)*t*t + 20*coefficients_(i,5)*t*t*t;
    }
    return ddq;
}