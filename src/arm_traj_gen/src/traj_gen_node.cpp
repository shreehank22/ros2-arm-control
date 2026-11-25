
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include "arm_kinodynamics/arm_model.hpp"
#include "arm_traj_gen/joint_limits.hpp"
#include "arm_traj_gen/trajectory.hpp"
#include <algorithm>
#include <cmath>
#include <exception>
#include <filesystem>

#include <Eigen/Dense>
#include <chrono>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>
#include <cmath>
#include <iterator>

class TrajectoryGeneratorNode : public rclcpp::Node
{
public:
    TrajectoryGeneratorNode()
    : Node("traj_gen_node")
    {
        // Joint names and limits come from the same arm model the controller uses
        // (FR3 URDF with the finger joints locked).
        const std::string urdf_path = resolveUrdfPath();
        const ArmModel arm = loadArmModel(
            urdf_path, {"fr3_finger_joint1", "fr3_finger_joint2"});
        joint_names_ = arm.joint_names;
        joint_limits_ = std::make_unique<JointLimits>(
            JointLimits::fromModel(arm.model));
        RCLCPP_INFO(
            get_logger(), "Loaded limits for %zu joints from %s",
            joint_limits_->size(), urdf_path.c_str());

        joint_state_subscriber_ =
            create_subscription<sensor_msgs::msg::JointState>(
                "/joint_states",
                10,
                std::bind(
                    &TrajectoryGeneratorNode::jointStateCallback,
                    this,
                    std::placeholders::_1));

        desired_state_publisher_ =
            create_publisher<sensor_msgs::msg::JointState>(
                "/desired_joint_states", 10);

        trajectory_timer_ = create_wall_timer(
            std::chrono::milliseconds(10),
            std::bind(&TrajectoryGeneratorNode::publishTrajectory, this));

        RCLCPP_INFO(
            get_logger(),
            "Trajectory generator started; waiting for joint states.");
    }

private:
    // Resolves the URDF: the urdf_path parameter if set, otherwise the installed arm_sim share directory.
    std::string resolveUrdfPath()
    {
        std::string path = declare_parameter<std::string>("urdf_path", "");
        if (path.empty()) {
            try {
                path = ament_index_cpp::get_package_share_directory("arm_sim") +
                    "/urdf/fr3.urdf";
            } catch (const std::exception &e) {
                throw std::runtime_error(
                    std::string("Cannot locate package 'arm_sim' to find fr3.urdf "
                                "(is it built and sourced?): ") + e.what());
            }
        }
        if (!std::filesystem::exists(path)) {
            throw std::runtime_error("URDF file not found: " + path);
        }
        return path;
    }

    void jointStateCallback(
        const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        if (trajectory_started_) {
            return;
        }

        // /joint_states may carry more joints than the arm (e.g. the fingers),
        // so only the name/position pairing is checked here.
        if (msg->position.size() != msg->name.size()) {
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 2000,
                "Invalid joint state dimensions");
            return;
        }

        for (const double position : msg->position) {
            if (!std::isfinite(position)) {
                RCLCPP_WARN_THROTTLE(
                    get_logger(), *get_clock(), 2000,
                    "Non-finite joint position received; ignoring message.");
                return;
            }
        }

        // Pick the arm joints out of the message by name, in model order (joint_names_).
        q_start_.resize(static_cast<Eigen::Index>(joint_names_.size()));
        for (std::size_t i = 0; i < joint_names_.size(); ++i) {
            const auto joint = std::find(
                msg->name.begin(), msg->name.end(), joint_names_[i]);

            if (joint == msg->name.end()) {
                RCLCPP_WARN_THROTTLE(
                    get_logger(), *get_clock(), 2000,
                    "%s not found in /joint_states", joint_names_[i].c_str());
                return;
            }
            q_start_[static_cast<Eigen::Index>(i)] =
                msg->position[static_cast<std::size_t>(std::distance(msg->name.begin(), joint))];
        }

        // The target joint is picked by name.
        const auto it = std::find(
            joint_names_.begin(), joint_names_.end(), "fr3_joint1");

        if (it == joint_names_.end()) {
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 2000,
                "fr3_joint1 not found in the arm model");
            return;
        }

        q_end_ = q_start_;
        q_end_[std::distance(joint_names_.begin(), it)] += 0.1;

        std::string limit_error;
        if (!joint_limits_->check(joint_names_, q_start_, q_end_, limit_error)) {
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 2000,
                "%s Refusing trajectory.", limit_error.c_str());
            return;
        }

        try {
            trajectory_ =
                std::make_unique<RobotTrajectoryGeneration>(
                    q_start_, q_end_, duration_);
        } catch (const std::exception &e) {
            RCLCPP_ERROR(
                get_logger(),
                "Failed to create trajectory: %s", e.what());
            return;
        }

        start_time_ = now();
        trajectory_started_ = true;

        RCLCPP_INFO(
            get_logger(),
            "Trajectory target: fr3_joint1 +0.1 rad, duration = %.1f s.",
            duration_);
    }

    void publishTrajectory()
    {
        if (!trajectory_started_ || !trajectory_) {
            return;
        }

        const double elapsed =
            (now() - start_time_).seconds();

        const double t = std::min(elapsed, duration_);

        const Eigen::VectorXd q_desired =
            trajectory_->getPosition(t);

        const Eigen::VectorXd qdot_desired =
            trajectory_->getVelocity(t);

        sensor_msgs::msg::JointState msg;
        msg.header.stamp = now();
        msg.name = joint_names_;

        msg.position.resize(joint_names_.size());
        msg.velocity.resize(joint_names_.size());

        for (std::size_t i = 0; i < joint_names_.size(); ++i) {
            msg.position[i] = q_desired[i];
            msg.velocity[i] = qdot_desired[i];
        }

        desired_state_publisher_->publish(msg);

        if (!completion_logged_ && elapsed >= duration_) {
            completion_logged_ = true;
            RCLCPP_INFO(
                get_logger(),
                "Trajectory complete; holding final position.");
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr
        joint_state_subscriber_;

    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr
        desired_state_publisher_;

    rclcpp::TimerBase::SharedPtr trajectory_timer_;

    std::unique_ptr<RobotTrajectoryGeneration> trajectory_;

    Eigen::VectorXd q_start_;
    Eigen::VectorXd q_end_;

    std::vector<std::string> joint_names_;
    std::unique_ptr<JointLimits> joint_limits_;

    rclcpp::Time start_time_{0, 0, RCL_ROS_TIME};

    const double duration_ = 5.0;

    bool trajectory_started_ = false;
    bool completion_logged_ = false;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TrajectoryGeneratorNode>());
    rclcpp::shutdown();
    return 0;
}
