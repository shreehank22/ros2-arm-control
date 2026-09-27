#include <gtest/gtest.h>

#include <Eigen/Dense>

#include <cmath>
#include <string>
#include <memory>
#include "arm_kinodynamics/dynamics.hpp"
#include "arm_kinodynamics/kinematics.hpp"
#include "vector"


// test scripts for dynamics and kinematics of the robot arm

TEST(RobotDynamicsTest, TestMassMatrix){
    std::string urdf_path = "/home/shreehank1906/ros2-arm-control/src/arm_sim/urdf/ur5e.urdf";
    RobotDynamics robot_dynamics(urdf_path);
    Eigen::VectorXd q(6);
    q << 0.0, -M_PI / 2.0, 0.0, -M_PI / 2.0, 0.0, 0.0;

    Eigen::MatrixXd mass_matrix = robot_dynamics.getMassMatrix(q);

    // Check if the mass matrix is symmetric
    EXPECT_TRUE(mass_matrix.isApprox(mass_matrix.transpose(), 1e-5));

    // test for positive definiteness of the mass matrix through eigenvalues
    Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eigensolver(mass_matrix);
    ASSERT_EQ(eigensolver.info(), Eigen::Success);
    EXPECT_TRUE((eigensolver.eigenvalues().array() > 0).all());

    // test for correct dimensions
    EXPECT_EQ(mass_matrix.rows(), q.size());
    EXPECT_EQ(mass_matrix.cols(), q.size());

    // finiteness of all values
    EXPECT_TRUE(mass_matrix.allFinite());    
}

TEST(RobotDynamicsTest, TestCoriolisAndGravity){
    std::string urdf_path = "/home/shreehank1906/ros2-arm-control/src/arm_sim/urdf/ur5e.urdf";
    RobotDynamics robot_dynamics(urdf_path);
    Eigen::VectorXd q(6);
    q << 0.2, -1.0, 0.4, -0.8, 0.3, 0.2;
    Eigen::VectorXd dq(6);
    dq << 0.1, 0.2, 0.3, 0.4, 0.5, 0.6;
    Eigen::VectorXd zero_velocity(6);
    zero_velocity.setZero();

    Eigen::VectorXd coriolis = robot_dynamics.getCoriolis(q, dq);
    Eigen::VectorXd gravity = robot_dynamics.getGravity(q);

    // check for zero velocity, Coriolis should be zero
    Eigen::VectorXd coriolis_zero = robot_dynamics.getCoriolis(q, zero_velocity);
    EXPECT_TRUE(coriolis_zero.isZero(1e-5));

    // Check if the Coriolis and Gravity vector is of the correct size
    EXPECT_EQ(coriolis.size(), q.size());
    EXPECT_EQ(gravity.size(), q.size());

    // Check if the Coriolis and Gravity vector contains finite values
    EXPECT_TRUE(coriolis.allFinite());
    EXPECT_TRUE(gravity.allFinite());

   

    // Coriolis vector reversal
    Eigen::VectorXd coriolis_reversed = robot_dynamics.getCoriolis(q, -dq);
    EXPECT_TRUE(coriolis_reversed.isApprox(coriolis, 1e-5));
}

TEST(RobotKinematicsTest, TestJacobian)
{
    std::string urdf_path = "/home/shreehank1906/ros2-arm-control/src/arm_sim/urdf/ur5e.urdf";
    RobotKinematics robot_kinematics(urdf_path);

    Eigen::VectorXd q(6);
    q << 0.0, -M_PI / 2.0, 0.0, -M_PI / 2.0, 0.0, 0.0;

    const double epsilon = 1e-6;

    // Analytical Jacobian from Pinocchio
    Eigen::MatrixXd jacobian = robot_kinematics.getJacobian(q);

    // Check dimensions
    EXPECT_EQ(jacobian.rows(), 6);
    EXPECT_EQ(jacobian.cols(), 6);

    // Check finite values
    EXPECT_TRUE(jacobian.allFinite());

    // Numerical linear-velocity Jacobian
    Eigen::MatrixXd numerical_jacobian(3, 6);

    for (int i = 0; i < 6; ++i)
    {
        Eigen::VectorXd q_plus = q;
        Eigen::VectorXd q_minus = q;

        q_plus(i) += epsilon;
        q_minus(i) -= epsilon;

        Eigen::Vector3d p_plus = robot_kinematics.getPosition(q_plus);
        Eigen::Vector3d p_minus =robot_kinematics.getPosition(q_minus);
        numerical_jacobian.col(i) = (p_plus - p_minus) / (2.0 * epsilon);
    }

    // Analytical linear-velocity Jacobian
    Eigen::MatrixXd analytical_jacobian = jacobian.topRows(3);

    // Compare analytical and numerical Jacobians
    EXPECT_TRUE(analytical_jacobian.isApprox(numerical_jacobian,1e-5));
}

TEST(RobotKinematicsTest, TestFKIK)
{
    std::string urdf_path = "/home/shreehank1906/ros2-arm-control/src/arm_sim/urdf/ur5e.urdf";
    RobotKinematics robot_kinematics(urdf_path);

    // Fixed target configuration
    Eigen::VectorXd q_test(6);
    q_test << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;

    // Forward kinematics -> target pose
    Eigen::Vector3d p_test = robot_kinematics.getPosition(q_test);
    Eigen::Matrix3d R_test = robot_kinematics.getRotation(q_test);

    // Deliberately different initial guess
    Eigen::VectorXd q_initial(6);
    q_initial << 0.0, -M_PI / 2.0, 0.0, -M_PI / 2.0, 0.0, 0.0;

    // Inverse kinematics: solve for q_test's pose starting from q_initial
    IKResult ik_result = robot_kinematics.solveIK(q_initial, p_test, R_test);
    ASSERT_TRUE(ik_result.converged);

    Eigen::VectorXd q_result = ik_result.q;

    // Recompute FK at the IK solution
    Eigen::Vector3d p_result = robot_kinematics.getPosition(q_result);
    Eigen::Matrix3d R_result = robot_kinematics.getRotation(q_result);

    // Separate, physically meaningful error metrics
    double position_error = (p_test - p_result).norm();                                   
    double rotation_error = robot_kinematics.logSO3(R_test * R_result.transpose()).norm();;          

    std::cout << "Position error: " << position_error << " m\n";
    std::cout << "Rotation error: " << rotation_error << " rad\n";

    // Thresholds — tune to what your solver/application actually needs
    constexpr double epsilon_p = 1e-4;  
    constexpr double epsilon_R = 1e-3;  

    EXPECT_LT(position_error, epsilon_p);
    EXPECT_LT(rotation_error, epsilon_R);
}

TEST(RobotKinematicsTest, IKMultipleConfigurations)
{
    std::string urdf_path = "/home/shreehank1906/ros2-arm-control/src/arm_sim/urdf/ur5e.urdf";
    RobotKinematics robot_kinematics(urdf_path);

    struct Config { Eigen::VectorXd q_test, q_initial; std::string name; };

    std::vector<Config> configs;

    Config c1; c1.name = "MildBend";
    c1.q_test = (Eigen::VectorXd(6) << 0.3, -1.2, 0.8, -1.0, 0.5, 0.2).finished();
    c1.q_initial = (Eigen::VectorXd(6) << 0.0, -1.5, 0.5, -1.0, 0.0, 0.0).finished();
    configs.push_back(c1);

    Config c2; c2.name = "ElbowBentWristRotated";
    c2.q_test = (Eigen::VectorXd(6) << -0.5, -0.8, 1.8, -2.0, -0.7, 1.0).finished();
    c2.q_initial = (Eigen::VectorXd(6) << 0.0, -1.0, 1.0, -1.5, 0.0, 0.0).finished();
    configs.push_back(c2);

    Config c3; c3.name = "NearFullExtension";
    c3.q_test = (Eigen::VectorXd(6) << 0.0, -1.57, 0.05, -1.57, 0.0, 0.0).finished();
    c3.q_initial = (Eigen::VectorXd(6) << 0.5, -1.0, 0.5, -1.0, 0.5, 0.5).finished();
    configs.push_back(c3);

    Config c4; c4.name = "WristAxesAligned";
    c4.q_test = (Eigen::VectorXd(6) << 0.2, -1.0, 1.0, -1.57, 0.0, 0.3).finished();
    c4.q_initial = (Eigen::VectorXd(6) << -0.3, -1.3, 0.6, -0.8, 0.4, -0.2).finished();
    configs.push_back(c4);

    Config c5; c5.name = "LargeExcursion";
    c5.q_test = (Eigen::VectorXd(6) << 1.5, -2.0, 1.5, -1.0, 1.0, -1.0).finished();
    c5.q_initial = (Eigen::VectorXd(6) << -1.5, -0.5, 0.2, -2.0, -1.0, 1.0).finished();
    configs.push_back(c5);

    Config c6; c6.name = "Zero";
    c6.q_test = (Eigen::VectorXd(6) << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0).finished();
    c6.q_initial = (Eigen::VectorXd(6) << 0.0, -M_PI / 2.0, 0.0, -M_PI / 2.0, 0.0, 0.0).finished();
    configs.push_back(c6);

    constexpr double epsilon_p = 1e-4;
    constexpr double epsilon_R = 1e-3;

    for (const auto& cfg : configs)
    {
        Eigen::Vector3d p_test = robot_kinematics.getPosition(cfg.q_test);
        Eigen::Matrix3d R_test = robot_kinematics.getRotation(cfg.q_test);

        IKResult ik_result = robot_kinematics.solveIK(cfg.q_initial, p_test, R_test);
        EXPECT_TRUE(ik_result.converged) << "Failed to converge for case: " << cfg.name;
        if (!ik_result.converged) 
        {
            std::cerr << "Failed to converge for case: " << cfg.name << std::endl;
            continue;
        }

        Eigen::Vector3d p_result = robot_kinematics.getPosition(ik_result.q);
        Eigen::Matrix3d R_result = robot_kinematics.getRotation(ik_result.q);

        double position_error = (p_test - p_result).norm();
        double rotation_error = robot_kinematics.logSO3(R_test * R_result.transpose()).norm();

        std::cout << "[" << cfg.name << "] pos_err: " << position_error
                   << " | rot_err: " << rotation_error
                   << " | iters: " << ik_result.n_iter << "\n";

        EXPECT_LT(position_error, epsilon_p) << "Position error too large for case: " << cfg.name;
        EXPECT_LT(rotation_error, epsilon_R) << "Rotation error too large for case: " << cfg.name;
    }
}

TEST (RobotKinematicsTest, TestJacobianAtMultipleConfigurations)
{
    std::string urdf_path = "/home/shreehank1906/ros2-arm-control/src/arm_sim/urdf/ur5e.urdf";
    RobotKinematics robot_kinematics(urdf_path);

    std::vector<Eigen::VectorXd> test_configs = {
        (Eigen::VectorXd(6) << 0.0, -M_PI / 2.0, 0.0, -M_PI / 2.0, 0.0, 0.0).finished(),
        (Eigen::VectorXd(6) << 0.3, -1.2, 0.8, -1.0, 0.5, 0.2).finished(),
        (Eigen::VectorXd(6) << -0.5, -0.8, 1.8, -2.0, -0.7, 1.0).finished(),
        (Eigen::VectorXd(6) << 1.5, -2.0, 1.5, -1.0, 1.0, -1.0).finished(),
        (Eigen::VectorXd(6) << 0.2, -1.0, 1.0, -1.57, 0.0, 0.3).finished()
    };

    const double epsilon = 1e-6;

    for (const auto& q : test_configs)
    {
        Eigen::MatrixXd jacobian = robot_kinematics.getJacobian(q);
        EXPECT_EQ(jacobian.rows(), 6);
        EXPECT_EQ(jacobian.cols(), 6);
        EXPECT_TRUE(jacobian.allFinite());

        Eigen::MatrixXd numerical_jacobian(3, 6);
        for (int i = 0; i < 6; ++i)
        {
            Eigen::VectorXd q_plus = q;
            Eigen::VectorXd q_minus = q;

            q_plus(i) += epsilon;
            q_minus(i) -= epsilon;

            Eigen::Vector3d p_plus = robot_kinematics.getPosition(q_plus);
            Eigen::Vector3d p_minus = robot_kinematics.getPosition(q_minus);
            numerical_jacobian.col(i) = (p_plus - p_minus) / (2.0 * epsilon);
        }

        Eigen::MatrixXd analytical_jacobian = jacobian.topRows(3);
        EXPECT_TRUE(analytical_jacobian.isApprox(numerical_jacobian, 1e-5));
    }
}   

TEST (RobotKinematicsTest, IKRoundTripMultipleConfigurations)
{
    std::string urdf_path = "/home/shreehank1906/ros2-arm-control/src/arm_sim/urdf/ur5e.urdf";
    RobotKinematics robot_kinematics(urdf_path);

    Eigen::VectorXd q_test(6);
    q_test << 0.0, -M_PI / 2.0, 0.0, -M_PI / 2.0, 0.0, 0.0;

    Eigen::Vector3d p_test = robot_kinematics.getPosition(q_test);
    Eigen::Matrix3d R_test = robot_kinematics.getRotation(q_test);

    std::vector<Eigen::VectorXd> initial_guesses = {
        (Eigen::VectorXd(6) << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0).finished(),
        (Eigen::VectorXd(6) << 1.5, -1.0, 1.5, -0.5, 1.0, -1.0).finished(),
        (Eigen::VectorXd(6) << -1.0, -1.8, 0.3, -1.2, -0.8, 0.5).finished(),
        (Eigen::VectorXd(6) << 0.5, -0.3, 2.0, -2.0, 0.2, 1.5).finished()
    };

    const double epsilon_p = 1e-4;
    const double epsilon_R = 1e-3;

    for (size_t i = 0; i < initial_guesses.size(); ++i)
    {
        IKResult ik_result = robot_kinematics.solveIK(initial_guesses[i], p_test, R_test);
        EXPECT_TRUE(ik_result.converged) << "Failed to converge for initial guess index: " << i;
        if (!ik_result.converged) 
        {
            std::cerr << "Failed to converge for initial guess index: " << i << std::endl;
            continue;
        }

        Eigen::Vector3d p_result = robot_kinematics.getPosition(ik_result.q);
        Eigen::Matrix3d R_result = robot_kinematics.getRotation(ik_result.q);

        double position_error = (p_test - p_result).norm();
        double rotation_error = robot_kinematics.logSO3(R_test * R_result.transpose()).norm();

        std::cout << "[Initial Guess " << i << "] pos_err: " << position_error
                  << " | rot_err: " << rotation_error
                  << " | iters: " << ik_result.n_iter << "\n";

        EXPECT_LT(position_error, epsilon_p) << "Position error too large for initial guess index: " << i;
        EXPECT_LT(rotation_error, epsilon_R) << "Rotation error too large for initial guess index: " << i;
    }
}

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}