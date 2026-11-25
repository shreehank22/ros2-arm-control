#include "arm_kinodynamics/dynamics.hpp"
#include "arm_kinodynamics/arm_model.hpp"

#include <pinocchio/algorithm/crba.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <iostream>

RobotDynamics::RobotDynamics(const std::string& urdf_path)
{
    // Load the arm model: FR3 URDF with the finger joints locked (7 DoF, hand mass kept)
    model_ = loadArmModel(urdf_path, {"fr3_finger_joint1", "fr3_finger_joint2"}).model;
    data_=pinocchio::Data(model_);
}

Eigen::MatrixXd RobotDynamics::getMassMatrix(Eigen::VectorXd const& q)
{
    pinocchio::crba(model_,data_,q);
    data_.M.triangularView<Eigen::Lower>() = data_.M.transpose().triangularView<Eigen::Lower>();
    return data_.M;
}

Eigen::VectorXd RobotDynamics::getCoriolis(Eigen::VectorXd const& q, Eigen::VectorXd const& dq)
{
    pinocchio::computeCoriolisMatrix(model_,data_,q,dq);
    return data_.C * dq;
}   

Eigen::VectorXd RobotDynamics::getGravity(Eigen::VectorXd const& q)
{
    pinocchio::computeGeneralizedGravity(model_,data_,q);
    return data_.g;
}   
