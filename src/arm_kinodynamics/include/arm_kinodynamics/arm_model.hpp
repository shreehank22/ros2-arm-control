#pragma once

#include <string>
#include <vector>

#include <pinocchio/multibody/model.hpp>

// The arm's Pinocchio model with some joints locked (e.g. the gripper fingers),
// plus the remaining joint names in model order.
struct ArmModel
{
    pinocchio::Model model;                // reduced model: only the unlocked (arm) joints
    std::vector<std::string> joint_names;  // joint names in model order, "universe" excluded
};

// Loads the URDF and locks the named joints at their neutral position, keeping their mass in the model.
// Throws std::runtime_error if a locked joint does not exist or a remaining joint is not 1-DoF.
ArmModel loadArmModel(const std::string& urdf_path, const std::vector<std::string>& locked_joints);
