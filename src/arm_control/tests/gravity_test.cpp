
#include <iostream>
#include <string>
#include <array>
#include <utility>
#include <vector>

#include <Eigen/Dense>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <pinocchio/algorithm/rnea.hpp>

#include "arm_kinodynamics/arm_model.hpp"

// Prints the gravity torques of the FR3 arm model (fingers locked, hand mass included)
// at a few configurations. Optional argument: path to the URDF (default: arm_sim's fr3.urdf).
int main(int argc, char **argv)
{
    const std::string urdf_path = (argc > 1)
        ? std::string(argv[1])
        : ament_index_cpp::get_package_share_directory("arm_sim") + "/urdf/fr3.urdf";

    const ArmModel arm = loadArmModel(urdf_path, {"fr3_finger_joint1", "fr3_finger_joint2"});
    const std::vector<std::string> &joint_names = arm.joint_names;

    const pinocchio::Model &model = arm.model;
    pinocchio::Data data(model);

    std::cout << "URDF: " << urdf_path << "\n";
    std::cout << "Model gravity vector: "
              << model.gravity.linear().transpose() << " m/s^2\n\n";

    const Eigen::VectorXd q_home =
        (Eigen::VectorXd(7) << 0.0, -0.7854, 0.0, -2.3562, 0.0, 1.5708, 0.7854).finished();

    // outside the FR3 joint limits (joint4 and joint6); shown only as a reference configuration
    const Eigen::VectorXd q_zero = Eigen::VectorXd::Zero(7);

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
