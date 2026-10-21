#include <gtest/gtest.h>
#include <Eigen/Dense>
#include "arm_traj_gen/joint_limits.hpp"
#include <cmath>
#include <stdexcept>
#include <string>
#include <vector>

namespace
{
const std::vector<std::string> kNames = {
    "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
    "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"};

Eigen::VectorXd home()
{
    Eigen::VectorXd q(6);
    q << 0.0, -1.571, 0.0, -1.571, 0.0, 0.0;
    return q;
}

Eigen::VectorXd withPanOffset(Eigen::VectorXd q, double dq)
{
    q[0] += dq;
    return q;
}
}  // namespace

class JointLimitsTest : public ::testing::Test
{
protected:
    JointLimits limits = JointLimits::fromUrdf(UR5E_URDF_PATH);
    std::string reason;
};

TEST_F(JointLimitsTest, LoadsAllSixJointsFromUrdf)
{
    EXPECT_EQ(limits.size(), 6u);
}

TEST_F(JointLimitsTest, HomePoseWithPanOffsetIsAccepted)
{
    EXPECT_TRUE(limits.check(kNames, home(), withPanOffset(home(), 0.1), reason)) << reason;
}

TEST_F(JointLimitsTest, StartAndTargetExactlyAtLimitsAreAccepted)
{
    Eigen::VectorXd q = home();
    q[0] = 2.0 * M_PI;
    q[2] = M_PI;  // elbow limit is +/- pi
    EXPECT_TRUE(limits.check(kNames, q, q, reason)) << reason;
    q[0] = -2.0 * M_PI;
    q[2] = -M_PI;
    EXPECT_TRUE(limits.check(kNames, q, q, reason)) << reason;
}

TEST_F(JointLimitsTest, StartBeyondLimitIsRejected)
{
    Eigen::VectorXd q = home();
    q[0] = 2.0 * M_PI + 0.01;
    EXPECT_FALSE(limits.check(kNames, q, q, reason));
    EXPECT_NE(reason.find("shoulder_pan_joint start"), std::string::npos) << reason;

    q = home();
    q[2] = M_PI + 0.01;  // elbow has the tighter +/- pi range
    EXPECT_FALSE(limits.check(kNames, q, q, reason));
    EXPECT_NE(reason.find("elbow_joint start"), std::string::npos) << reason;

    q = home();
    q[3] = -2.0 * M_PI - 0.01;
    EXPECT_FALSE(limits.check(kNames, q, q, reason));
    EXPECT_NE(reason.find("wrist_1_joint start"), std::string::npos) << reason;
}

TEST_F(JointLimitsTest, TargetBeyondLimitIsRejectedEvenIfStartIsValid)
{
    Eigen::VectorXd q = home();
    q[0] = 2.0 * M_PI - 0.05;  // start valid, +0.1 pushes the target over
    EXPECT_FALSE(limits.check(kNames, q, withPanOffset(q, 0.1), reason));
    EXPECT_NE(reason.find("shoulder_pan_joint target"), std::string::npos) << reason;
}

TEST_F(JointLimitsTest, NonFiniteValuesAreRejected)
{
    Eigen::VectorXd q = home();
    q[1] = std::nan("");
    EXPECT_FALSE(limits.check(kNames, q, q, reason));
    q[1] = INFINITY;
    EXPECT_FALSE(limits.check(kNames, q, q, reason));
}

TEST_F(JointLimitsTest, UnknownJointIsRejected)
{
    std::vector<std::string> names = kNames;
    names[5] = "bogus_joint";
    EXPECT_FALSE(limits.check(names, home(), home(), reason));
    EXPECT_NE(reason.find("bogus_joint"), std::string::npos) << reason;
}

TEST_F(JointLimitsTest, SizeMismatchIsRejected)
{
    Eigen::VectorXd five = home().head(5);
    EXPECT_FALSE(limits.check(kNames, five, five, reason));
    EXPECT_FALSE(limits.check(kNames, home(), five, reason));
}

TEST(JointLimitsLoadTest, MissingUrdfThrows)
{
    EXPECT_ANY_THROW(JointLimits::fromUrdf("/nonexistent/path/ur5e.urdf"));
}

TEST(JointLimitsLoadTest, ModelWithoutJointsThrows)
{
    pinocchio::Model empty;  // only the universe joint, so no limits to load
    EXPECT_THROW(JointLimits::fromModel(empty), std::runtime_error);
}
