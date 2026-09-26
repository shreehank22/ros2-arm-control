#pragma once

#include <string>

#include <Eigen/Dense>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>

class RobotDynamics
{
public:
      explicit RobotDynamics(const std::string& urdf_path);

      // mass matrix
      Eigen::MatrixXd getMassMatrix(const Eigen::VectorXd& q);

      // Coriolis terms
      Eigen::VectorXd getCoriolis(const Eigen::VectorXd& q, const Eigen::VectorXd& dq);

      // gravity terms
      Eigen::VectorXd getGravity(const Eigen::VectorXd& q);

private:
        pinocchio::Model model_;
        pinocchio::Data data_;
};







