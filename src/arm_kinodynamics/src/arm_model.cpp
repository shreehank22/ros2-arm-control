#include "arm_kinodynamics/arm_model.hpp"

#include <pinocchio/parsers/urdf.hpp>                   // urdf::buildModel
#include <pinocchio/algorithm/model.hpp>                // buildReducedModel
#include <pinocchio/algorithm/joint-configuration.hpp>  // neutral

#include <stdexcept>

ArmModel loadArmModel(const std::string& urdf_path, const std::vector<std::string>& locked_joints)
{
    // 1. full model, every joint in the URDF (FR3 + hand: 9)
    pinocchio::Model full;
    pinocchio::urdf::buildModel(urdf_path, full);

    // 2. names of the joints to lock -> joint indices
    std::vector<pinocchio::JointIndex> ids;
    for (const auto& name : locked_joints)
    {
        if (!full.existJointName(name))
        {
            throw std::runtime_error("Cannot lock unknown joint: " + name);
        }
        ids.push_back(full.getJointId(name));
    }

    // 3. configuration the locked joints are frozen at (fingers closed)
    const Eigen::VectorXd q_lock = pinocchio::neutral(full);

    // 4. reduced model; the locked links' inertia stays attached to their parents
    ArmModel arm;
    pinocchio::buildReducedModel(full, ids, q_lock, arm.model);

    // 5. remaining joint names; index 0 is "universe"
    for (pinocchio::JointIndex i = 1; i < static_cast<pinocchio::JointIndex>(arm.model.njoints); ++i)
    {
        if (arm.model.joints[i].nq() != 1)
        {
            throw std::runtime_error("Joint '" + arm.model.names[i] + "' is not 1-DoF.");
        }
        arm.joint_names.push_back(arm.model.names[i]);
    }

    return arm;
}
