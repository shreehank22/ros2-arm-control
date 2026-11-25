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
    "fr3_joint1", "fr3_joint2", "fr3_joint3", "fr3_joint4",
    "fr3_joint5", "fr3_joint6", "fr3_joint7"};

Eigen::VectorXd home()
{
    Eigen::VectorXd q(7);
    q << 0.0, -0.7854, 0.0, -2.3562, 0.0, 1.5708, 0.7854;
    return q;
}

Eigen::VectorXd withJoint1Offset(Eigen::VectorXd q, double dq)
{
    q[0] += dq;
    return q;
}
}  // namespace

class JointLimitsTest : public ::testing::Test
{
protected:
    JointLimits limits = JointLimits::fromUrdf(FR3_URDF_PATH);
    std::string reason;
};

TEST_F(JointLimitsTest, LoadsAllNineJointsFromUrdf)
{
    // the full URDF: 7 arm joints + 2 finger joints
    EXPECT_EQ(limits.size(), 9u);
}

TEST_F(JointLimitsTest, HomePoseWithJoint1OffsetIsAccepted)
{
    EXPECT_TRUE(limits.check(kNames, home(), withJoint1Offset(home(), 0.1), reason)) << reason;
}

TEST_F(JointLimitsTest, StartAndTargetExactlyAtLimitsAreAccepted)
{
    Eigen::VectorXd q = home();
    q[0] = 2.9007;
    q[3] = -0.1169;  // joint4 range is asymmetric: [-3.077, -0.1169]
    q[5] = 4.6216;   // joint6 range is asymmetric: [0.4398, 4.6216]
    EXPECT_TRUE(limits.check(kNames, q, q, reason)) << reason;
    q[0] = -2.9007;
    q[3] = -3.077;
    q[5] = 0.4398;
    EXPECT_TRUE(limits.check(kNames, q, q, reason)) << reason;
}

TEST_F(JointLimitsTest, StartBeyondLimitIsRejected)
{
    Eigen::VectorXd q = home();
    q[0] = 2.9007 + 0.01;
    EXPECT_FALSE(limits.check(kNames, q, q, reason));
    EXPECT_NE(reason.find("fr3_joint1 start"), std::string::npos) << reason;

    q = home();
    q[3] = 0.0;  // joint4 can never reach 0 (upper limit -0.1169)
    EXPECT_FALSE(limits.check(kNames, q, q, reason));
    EXPECT_NE(reason.find("fr3_joint4 start"), std::string::npos) << reason;

    q = home();
    q[5] = 0.4398 - 0.01;  // joint6 has a positive lower limit
    EXPECT_FALSE(limits.check(kNames, q, q, reason));
    EXPECT_NE(reason.find("fr3_joint6 start"), std::string::npos) << reason;
}

TEST_F(JointLimitsTest, TargetBeyondLimitIsRejectedEvenIfStartIsValid)
{
    Eigen::VectorXd q = home();
    q[0] = 2.9007 - 0.05;  // start valid, +0.1 pushes the target over
    EXPECT_FALSE(limits.check(kNames, q, withJoint1Offset(q, 0.1), reason));
    EXPECT_NE(reason.find("fr3_joint1 target"), std::string::npos) << reason;
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
    names[6] = "bogus_joint";
    EXPECT_FALSE(limits.check(names, home(), home(), reason));
    EXPECT_NE(reason.find("bogus_joint"), std::string::npos) << reason;
}

TEST_F(JointLimitsTest, SizeMismatchIsRejected)
{
    Eigen::VectorXd six = home().head(6);
    EXPECT_FALSE(limits.check(kNames, six, six, reason));
    EXPECT_FALSE(limits.check(kNames, home(), six, reason));
}

TEST(JointLimitsLoadTest, MissingUrdfThrows)
{
    EXPECT_ANY_THROW(JointLimits::fromUrdf("/nonexistent/path/fr3.urdf"));
}

TEST(JointLimitsLoadTest, ModelWithoutJointsThrows)
{
    pinocchio::Model empty;  // only the universe joint, so no limits to load
    EXPECT_THROW(JointLimits::fromModel(empty), std::runtime_error);
}
