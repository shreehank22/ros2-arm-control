#include <Eigen/Dense>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <iostream>


// testing Jacobian computation

int main(){
    pinocchio::Model model;
    pinocchio::urdf::buildModel("/home/shreehank1906/ros2-arm-control/src/arm_sim/urdf/ur5e.urdf",model);
    pinocchio::Data data(model);
    Eigen::VectorXd q0(6);
    Eigen::MatrixXd J(6,model.nv);

    q0 << 0, M_PI/2, 0, -M_PI/2, 0, 0;
    pinocchio::computeJointJacobians(model,data,q0);
    pinocchio::updateFramePlacement(model,data,model.getFrameId("tool0"));
    pinocchio::getFrameJacobian(model,data,model.getFrameId("tool0"),pinocchio::LOCAL_WORLD_ALIGNED,J);
    std::cout << "Jacobian:\n" << J << std::endl;
}