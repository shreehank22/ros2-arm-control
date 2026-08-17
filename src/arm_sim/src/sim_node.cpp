#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/rnea.hpp>  
#include <iostream>
#include <memory>
#include <Eigen/Dense>
#include <pinocchio/algorithm/crba.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <chrono>
#include <algorithm>

class SimNode : public rclcpp::Node {
    public:
        SimNode() : Node("sim_node") {
            RCLCPP_INFO(this->get_logger(), "SimNode constructor started");
            std::string urdf_path = "/home/shreehank1906/ros2-arm-control/src/arm_sim/urdf/ur5e.urdf";
            pinocchio::urdf::buildModel(urdf_path, model_);
            data_ = std::make_unique<pinocchio::Data>(model_);
            q_ = Eigen::VectorXd::Constant(model_.nq, 0.3);
            q_dot_ = Eigen::VectorXd::Constant(model_.nv, 0.0);
            joint_state_publisher = this->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);
            joint_effort_subscriber = this->create_subscription<std_msgs::msg::Float64MultiArray>(
                "/joint_efforts", 10, std::bind(&SimNode::jointEffortCallback, this, std::placeholders::_1));
            timer_ = this->create_wall_timer(std::chrono::microseconds(100),[this]() {this->onTimer(); });
            RCLCPP_INFO(this->get_logger(), "SimNode constructor finished");
        }

        void onTimer() {
            log_counter_++;
            Eigen::VectorXd nle = pinocchio::nonLinearEffects(model_, *data_, q_, q_dot_);
            pinocchio::crba(model_, *data_, q_);
            data_->M.triangularView<Eigen::StrictlyLower>() = data_->M.transpose().triangularView<Eigen::StrictlyLower>();
            Eigen::VectorXd damping = 0.5 * q_dot_;
            Eigen::VectorXd qddot = data_->M.ldlt().solve(tau_commanded - nle - damping);
            q_dot_ += qddot * dt;
            q_ += q_dot_ * dt;
            sensor_msgs::msg::JointState msg;
            msg.header.stamp = this->now();
            for (size_t i = 1; i < model_.names.size(); i++) {
                msg.name.push_back(model_.names[i]);
            }
            msg.position.assign(q_.data(), q_.data() + q_.size());
            msg.velocity.assign(q_dot_.data(), q_dot_.data() + q_dot_.size());
            joint_state_publisher->publish(msg);

            if (log_counter_ % 1000 == 0) {
                RCLCPP_INFO(this->get_logger(), "q_ = [%f, %f, %f, %f, %f, %f]",q_[0], q_[1], q_[2], q_[3], q_[4], q_[5]);
                RCLCPP_INFO(this->get_logger(), "qdot_ = [%f, %f, %f, %f, %f, %f]",q_dot_[0], q_dot_[1], q_dot_[2], q_dot_[3], q_dot_[4], q_dot_[5]);
            }
        }
        void jointEffortCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
            Eigen::Map<Eigen::VectorXd> tau_map(msg->data.data(), msg->data.size());
            tau_commanded = tau_map.cwiseMax(-150.0).cwiseMin(150.0);
            effort_log_counter_++;
            if (effort_log_counter_ % 1000 == 0) {
                RCLCPP_INFO(this->get_logger(), "tau_commanded = [%f, %f, %f, %f, %f, %f]",
                    tau_commanded[0], tau_commanded[1], tau_commanded[2],
                    tau_commanded[3], tau_commanded[4], tau_commanded[5]);
            }
        }
    private:
        pinocchio::Model model_;
        std::unique_ptr<pinocchio::Data> data_;
        Eigen::VectorXd q_;
        Eigen::VectorXd q_dot_;
        int log_counter_ = 0;
        int effort_log_counter_ = 0;
        rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_publisher;
        rclcpp::TimerBase::SharedPtr timer_;
        Eigen::VectorXd tau_commanded = Eigen::VectorXd::Zero(6);
        rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr joint_effort_subscriber;
        double dt = 0.0001;
};

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SimNode>());
    rclcpp::shutdown();
    return 0;
}