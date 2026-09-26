#pragma once

#include <string>

#include <Eigen/Dense>

#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>


struct IKResult
{
    Eigen::VectorXd q;
    bool converged;
    int n_iter;
    double pos_err;
    double rot_err;
};

class RobotKinematics
{
public:
    explicit RobotKinematics(const std::string& urdf_path);

    Eigen::Vector3d logSO3(const Eigen::Matrix3d& R);

    // Forward kinematics
    Eigen::Vector3d getPosition(const Eigen::VectorXd& q);

    Eigen::Matrix3d getRotation(const Eigen::VectorXd& q);

    // Geometric Jacobian
    Eigen::MatrixXd getJacobian(const Eigen::VectorXd& q);

    // Damped Least-Squares inverse kinematics
    IKResult solveIK(const Eigen::VectorXd& q0,const Eigen::Vector3d& p_des,const Eigen::Matrix3d& R_des);

private:

    // Compute Jacobian internally for the IK solver
    Eigen::MatrixXd computeJacobian(const Eigen::VectorXd& q);

    pinocchio::Model model_;
    pinocchio::Data data_;
    pinocchio::FrameIndex ee_frame_;

    // DLS IK parameters
    double k_;
    double lambda_max_;
    double epsilon_;
    int max_iter_;
    double tol_;
};