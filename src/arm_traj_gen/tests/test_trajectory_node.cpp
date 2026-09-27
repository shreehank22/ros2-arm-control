#include <gtest/gtest.h>
#include <Eigen/Dense>
#include "arm_traj_gen/trajectory.hpp"
#include <cmath>
#include <iostream>

// testing position for trajectory generation
TEST(RobotTrajectoryGenerationTest, TestPosition)
{
    Eigen::VectorXd q_start(6);
    q_start << 0.0, -M_PI / 2.0, 0.0, -M_PI / 2.0, 0.0, 0.0;

    Eigen::VectorXd q_end(6);
    q_end << M_PI / 2.0, -M_PI / 4.0, M_PI / 4.0, -M_PI / 4.0, M_PI / 4.0, M_PI / 2.0;

    double duration = 5.0;

    RobotTrajectoryGeneration trajectory(q_start, q_end, duration);

    double t = 2.5; // Midpoint of the trajectory
    Eigen::VectorXd q_mid = trajectory.getPosition(t);

    // Check if the midpoint is approximately halfway between start and end
    Eigen::VectorXd expected_mid = (q_start + q_end) / 2.0;
    EXPECT_TRUE(q_mid.isApprox(expected_mid, 1e-5));

    // Check if the start and end positions are correct
    Eigen::VectorXd q_start_check = trajectory.getPosition(0.0);
    Eigen::VectorXd q_end_check = trajectory.getPosition(duration);
    EXPECT_TRUE(q_start_check.isApprox(q_start, 1e-5));
    EXPECT_TRUE(q_end_check.isApprox(q_end, 1e-5));

    // check for negative time and time greater than duration
    Eigen::VectorXd q_neg_time = trajectory.getPosition(-1.0);
    Eigen::VectorXd q_over_time = trajectory.getPosition(duration + 1.0);
    EXPECT_TRUE(q_neg_time.isApprox(q_start, 1e-5));
    EXPECT_TRUE(q_over_time.isApprox(q_end, 1e-5));

}

TEST(RobotTrajectoryGenerationTest, TestAcceleration)
{
    Eigen::VectorXd q_start(6);
    q_start << 0.0, -M_PI / 2.0, 0.0, -M_PI / 2.0, 0.0, 0.0;

    Eigen::VectorXd q_end(6);
    q_end << M_PI / 2.0, -M_PI / 4.0, M_PI / 4.0, -M_PI / 4.0, M_PI / 4.0, M_PI / 2.0;

    double duration = 5.0;

    RobotTrajectoryGeneration trajectory(q_start, q_end, duration);

    Eigen::VectorXd ddq_start = trajectory.getAcceleration(0.0);
    Eigen::VectorXd ddq_end = trajectory.getAcceleration(duration);

    EXPECT_TRUE(ddq_start.norm() < 1e-5);
    EXPECT_TRUE(ddq_end.norm() < 1e-5);

    Eigen::VectorXd ddq_mid = trajectory.getAcceleration(duration / 2.0);

    EXPECT_TRUE(ddq_mid.norm() < 1e-5);
}

TEST(RobotTrajectoryGenerationTest, TestVelocity)
{
    Eigen::VectorXd q_start(6);
    q_start << 0.0, -M_PI / 2.0, 0.0, -M_PI / 2.0, 0.0, 0.0;

    Eigen::VectorXd q_end(6);
    q_end << M_PI / 2.0, -M_PI / 4.0, M_PI / 4.0, -M_PI / 4.0, M_PI / 4.0, M_PI / 2.0;

    double duration = 5.0;

    RobotTrajectoryGeneration trajectory(q_start, q_end, duration);

    Eigen::VectorXd dq_start = trajectory.getVelocity(0.0);
    Eigen::VectorXd dq_end = trajectory.getVelocity(duration);

    EXPECT_TRUE(dq_start.norm() < 1e-5);
    EXPECT_TRUE(dq_end.norm() < 1e-5);

    Eigen::VectorXd expected_dq_mid = 0.375 * (q_end - q_start);
    Eigen::VectorXd dq_mid = trajectory.getVelocity(duration / 2.0);

    EXPECT_TRUE(dq_mid.isApprox(expected_dq_mid, 1e-5));
}