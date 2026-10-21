
#include <iostream>
#include <string>
#include <array>
#include <utility>

#include <Eigen/Dense>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/rnea.hpp>

int main()
{
    const std::string urdf_path =
        "/home/shreehank1906/ros2-arm-control/src/arm_sim/urdf/ur5e.urdf";

    const std::array<std::string, 6> joint_names = {
        "shoulder_pan_joint",
        "shoulder_lift_joint",
        "elbow_joint",
        "wrist_1_joint",
        "wrist_2_joint",
        "wrist_3_joint"
    };

    pinocchio::Model model;
    pinocchio::urdf::buildModel(urdf_path, model);
    pinocchio::Data data(model);

    std::cout << "Model gravity vector: "
              << model.gravity.linear().transpose() << " m/s^2\n\n";

    const Eigen::VectorXd q_home =
        (Eigen::VectorXd(6) << 0.0, -1.571, 0.0, -1.571, 0.0, 0.0).finished();

    const Eigen::VectorXd q_zero = Eigen::VectorXd::Zero(6);

    for (const auto &test :
         std::array<std::pair<std::string, Eigen::VectorXd>, 2>{
             std::make_pair("Home", q_home),
             std::make_pair("All-zero", q_zero)}) {

        pinocchio::computeGeneralizedGravity(model, data, test.second);

        std::cout << test.first << " configuration:\n";

        for (std::size_t i = 0; i < joint_names.size(); ++i) {
            std::cout << "  " << joint_names[i] << ": "
                      << data.g[static_cast<Eigen::Index>(i)] << " Nm\n";
        }

        std::cout << '\n';
    }

    std::cout << "Read-only test complete; no effort commands sent.\n";
    return 0;
}
