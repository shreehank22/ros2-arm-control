#include <gtest/gtest.h>

#include <Eigen/Dense>
#include <stdexcept>
#include <string>
#include <vector>

#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/rnea.hpp>

#include "arm_kinodynamics/arm_model.hpp"

// FR3_URDF_PATH is set by CMake and points at arm_sim/urdf/fr3.urdf

namespace
{
const std::vector<std::string> kLocked = {"fr3_finger_joint1", "fr3_finger_joint2"};

Eigen::VectorXd home()
{
    Eigen::VectorXd q(7);
    q << 0.0, -0.7854, 0.0, -2.3562, 0.0, 1.5708, 0.7854;
    return q;
}
}  // namespace

TEST(ArmModelTest, ReducesToSevenArmJoints)
{
    const ArmModel arm = loadArmModel(FR3_URDF_PATH, kLocked);

    EXPECT_EQ(arm.model.nq, 7);
    EXPECT_EQ(arm.model.nv, 7);
    ASSERT_EQ(arm.joint_names.size(), 7u);
    for (std::size_t i = 0; i < arm.joint_names.size(); ++i)
    {
        EXPECT_EQ(arm.joint_names[i], "fr3_joint" + std::to_string(i + 1));
    }
}

TEST(ArmModelTest, KeepsEffortLimits)
{
    const ArmModel arm = loadArmModel(FR3_URDF_PATH, kLocked);

    ASSERT_EQ(arm.model.effortLimit.size(), 7);
    for (int i = 0; i < 4; ++i)
    {
        EXPECT_DOUBLE_EQ(arm.model.effortLimit[i], 87.0) << "joint index " << i;
    }
    for (int i = 4; i < 7; ++i)
    {
        EXPECT_DOUBLE_EQ(arm.model.effortLimit[i], 12.0) << "joint index " << i;
    }
}

// Locking the fingers must keep their (and the hand's) mass: gravity torques on the
// arm joints must equal those of the full 9-joint model with the fingers at 0.
TEST(ArmModelTest, GravityIncludesHandMass)
{
    const ArmModel arm = loadArmModel(FR3_URDF_PATH, kLocked);
    pinocchio::Data data_reduced(arm.model);
    const Eigen::VectorXd g_reduced =
        pinocchio::computeGeneralizedGravity(arm.model, data_reduced, home());

    pinocchio::Model full;
    pinocchio::urdf::buildModel(FR3_URDF_PATH, full);
    ASSERT_EQ(full.nq, 9);
    pinocchio::Data data_full(full);
    Eigen::VectorXd q_full(9);
    q_full << home(), 0.0, 0.0;
    const Eigen::VectorXd g_full =
        pinocchio::computeGeneralizedGravity(full, data_full, q_full);

    EXPECT_TRUE(g_reduced.isApprox(g_full.head(7), 1e-9))
        << "reduced: " << g_reduced.transpose() << "\nfull:    " << g_full.head(7).transpose();
    EXPECT_TRUE(g_reduced.allFinite());
}

TEST(ArmModelTest, KeepsTcpFrame)
{
    const ArmModel arm = loadArmModel(FR3_URDF_PATH, kLocked);
    EXPECT_TRUE(arm.model.existFrame("fr3_hand_tcp"));
}

TEST(ArmModelTest, UnknownLockedJointThrows)
{
    EXPECT_THROW(loadArmModel(FR3_URDF_PATH, {"fr3_finger_jiont1"}), std::runtime_error);
}
