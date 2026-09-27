#include "arm_kinodynamics/dynamics.hpp"

#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/crba.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <iostream>

RobotDynamics::RobotDynamics(const std::string& urdf_path)
{
    // Load the urdf model
    pinocchio::urdf::buildModel(urdf_path, model_);
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
