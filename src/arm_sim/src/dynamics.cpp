#include "arm_sim/dynamics.hpp"

namespace arm_sim {

Dynamics::Dynamics() {
  // TODO: initialize link lengths / masses / CoM offsets / inertias
}

Eigen::Matrix3d Dynamics::massMatrix(const Eigen::Vector3d & /*q*/) {
  // TODO: port from your verified P3 RNEA implementation
  return Eigen::Matrix3d::Identity();
}

Eigen::Vector3d Dynamics::coriolis(const Eigen::Vector3d & /*q*/, const Eigen::Vector3d & /*qdot*/) {
  // TODO
  return Eigen::Vector3d::Zero();
}

Eigen::Vector3d Dynamics::gravity(const Eigen::Vector3d & /*q*/) {
  // TODO
  return Eigen::Vector3d::Zero();
}

}  // namespace arm_sim
