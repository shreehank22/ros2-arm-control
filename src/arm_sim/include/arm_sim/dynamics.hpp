#pragma once
#include <Eigen/Dense>

// Port of your P3 RNEA work (Study-of-Robot-Dynamics) to C++/Eigen.
// Fill these in using the analytical ground truth you already verified
// (M[0,0]=6.57, M[1,1]=1.3366, M[2,2]=0.0833 at q=0).

namespace arm_sim {

class Dynamics {
public:
  Dynamics();

  // Mass matrix M(q), 3x3 for the 3R planar arm
  Eigen::Matrix3d massMatrix(const Eigen::Vector3d & q);

  // Coriolis/centrifugal vector C(q, qdot) * qdot, 3x1
  Eigen::Vector3d coriolis(const Eigen::Vector3d & q, const Eigen::Vector3d & qdot);

  // Gravity vector G(q), 3x1
  Eigen::Vector3d gravity(const Eigen::Vector3d & q);

private:
  // link lengths, masses, CoM offsets, inertias -- copy from
  // Study-of-Robot-Dynamics/models/three_r_planar.py
};

}  // namespace arm_sim
