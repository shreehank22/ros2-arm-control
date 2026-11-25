#include "arm_kinodynamics/kinematics.hpp"
#include "arm_kinodynamics/arm_model.hpp"

#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <iostream>

#include <algorithm>
#include <cmath>
#include <stdexcept>


RobotKinematics::RobotKinematics(const std::string& urdf_path)
{
    // Load the arm model: FR3 URDF with the finger joints locked (7 DoF, hand mass kept)
    model_ = loadArmModel(urdf_path, {"fr3_finger_joint1", "fr3_finger_joint2"}).model;

    // Create data associated with the model
    data_ = pinocchio::Data(model_);

    // Find the end-effector frame
    if (model_.existFrame("fr3_hand_tcp"))
    {
        ee_frame_ = model_.getFrameId("fr3_hand_tcp");
    }
    else
    {
        throw std::runtime_error("Frame 'fr3_hand_tcp' not found in the model.");
    }

    // DLS IK parameters
    k_ = 0.1;
    lambda_max_ = 0.1;
    epsilon_ = 1e-6;
    max_iter_ = 1000;
    tol_ = 1e-4;
}


Eigen::Vector3d RobotKinematics::getPosition(const Eigen::VectorXd& q)
{
    pinocchio::forwardKinematics(model_,data_,q);
    pinocchio::updateFramePlacement(model_,data_,ee_frame_);
    return data_.oMf[ee_frame_].translation();
}


Eigen::Matrix3d RobotKinematics::getRotation(const Eigen::VectorXd& q)
{
    pinocchio::forwardKinematics(model_,data_,q);
    pinocchio::updateFramePlacement(model_,data_,ee_frame_);
    return data_.oMf[ee_frame_].rotation();
}


Eigen::MatrixXd RobotKinematics::getJacobian(const Eigen::VectorXd& q)
{
    return computeJacobian(q);
}


Eigen::MatrixXd RobotKinematics::computeJacobian(const Eigen::VectorXd& q)
{
    Eigen::MatrixXd J(6, model_.nv);
    pinocchio::computeJointJacobians(model_,data_,q);
    pinocchio::updateFramePlacement(model_,data_,ee_frame_);
    pinocchio::getFrameJacobian(model_,data_,ee_frame_,pinocchio::LOCAL_WORLD_ALIGNED,J);
    return J;
}

Eigen::Vector3d RobotKinematics::logSO3(const Eigen::Matrix3d& R)
{
    double trace_val = (R.trace() - 1.0) / 2.0;

    double theta = std::acos(std::clamp(trace_val, -1.0, 1.0));


    // Small-angle case
    if (std::abs(theta) < 1e-7)
    {
        Eigen::Matrix3d skew = 0.5 * (R - R.transpose());

        Eigen::Vector3d e;
        e << skew(2, 1),skew(0, 2),skew(1, 0);

        return e;
    }

    // Near-pi case
    else if (std::abs(theta - M_PI) < 1e-4)
    {
        // The symmetric part of R gives A = n * n^T, which fixes the axis up to its sign:
        // take the column with the largest diagonal entry (the best-conditioned one).
        const double c = std::cos(theta);
        Eigen::Matrix3d A = (0.5 * (R + R.transpose()) - c * Eigen::Matrix3d::Identity()) / (1.0 - c);

        Eigen::Index k = 0;
        A.diagonal().maxCoeff(&k);

        Eigen::Vector3d n = A.col(k) / std::sqrt(std::max(A(k, k), 1e-12));
        n.normalize();

        // The skew part of R is 2 sin(theta) [n]x: it gives the sign of n when theta < pi.
        const Eigen::Vector3d skew_vec(R(2, 1) - R(1, 2), R(0, 2) - R(2, 0), R(1, 0) - R(0, 1));
        if (n.dot(skew_vec) < 0.0)
            n = -n;

        return theta * n;
    }

    // General case
    else
    {
        Eigen::Matrix3d skew = (theta / (2.0 * std::sin(theta))) * (R - R.transpose());

        Eigen::Vector3d e;

        e << skew(2, 1),skew(0, 2),skew(1, 0);

        return e;
    }
}

IKResult RobotKinematics::solveIK(const Eigen::VectorXd& q0,const Eigen::Vector3d& p_des,const Eigen::Matrix3d& R_des)
{
    IKResult result;

    Eigen::VectorXd q = q0;

    bool converged = false;

    double pos_err = 0.0;
    double rot_err = 0.0;

    int n_iter = 0;

    for (int iter = 0; iter < max_iter_; ++iter)
    {
        n_iter = iter + 1;

        // Current end-effector pose
        Eigen::Vector3d p_curr = getPosition(q);

        // Current end-effector orientation
        Eigen::Matrix3d R_curr = getRotation(q);

        // Position error
        Eigen::Vector3d e_pos = p_des - p_curr;

        // Orientation error
        Eigen::Vector3d e_rot = logSO3(R_des * R_curr.transpose());

        // Combined task-space error
        Eigen::VectorXd e(6);
        e.head<3>() = e_pos;
        e.tail<3>() = e_rot;

        const double w_p = 1.0;
        const double w_r = 0.85;

        Eigen::VectorXd weighted_e(6);
        weighted_e.head<3>() = w_p * e_pos;
        weighted_e.tail<3>() = w_r * e_rot;

        pos_err = e_pos.norm();
        rot_err = e_rot.norm();

        if (iter%50==0)
        {
            std::cout << "Iteration: " << iter
                      << " | Position error: " << pos_err
                      << " | Rotation error: " << rot_err
                      << std::endl;
        }

        // Check convergence
        if (pos_err < tol_ && rot_err < tol_)
        {
            converged = true;
            break;
        }

        // Compute Jacobian
        Eigen::MatrixXd J = computeJacobian(q);

        // Singular Value Decomposition
        Eigen::JacobiSVD<Eigen::MatrixXd> svd(J,Eigen::ComputeThinU |Eigen::ComputeThinV);
        Eigen::VectorXd S = svd.singularValues();

        // Adaptive damping
        Eigen::VectorXd lambda(S.size());

        for (int i = 0; i < S.size(); ++i)
        {
            lambda(i) =lambda_max_*std::exp(-std::pow(S(i)/epsilon_,2.0));
        }

        // Damped inverse singular values
        Eigen::VectorXd S_inv(S.size());

        for (int i = 0; i < S.size(); ++i)
        {
            S_inv(i) =S(i)/(S(i) * S(i) + lambda(i) * lambda(i));
        }

        // Damped least-squares solution
        Eigen::MatrixXd J_dls = svd.matrixV()*S_inv.asDiagonal()*svd.matrixU().transpose();
        Eigen::VectorXd dq = J_dls * weighted_e;


        // Step scaling
        dq *= k_;

        // Limit maximum joint-space step
        double dq_norm = dq.norm();

        if (dq_norm>0.5)
        {
            dq *= 0.5/dq_norm;
        }

        // Update configuration
        q += dq;
    }


    result.q = q;
    result.converged = converged;
    result.n_iter = n_iter;
    result.pos_err = pos_err;
    result.rot_err = rot_err;

    return result;
}