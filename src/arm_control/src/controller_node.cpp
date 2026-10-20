#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <Eigen/Dense>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <memory>
#include <algorithm>
#include <array>
#include <cmath>
#include <iterator>
#include <string>
#include <chrono>
#include <functional>

class ControllerNode : public rclcpp::Node {

    public:
        ControllerNode() : Node("controller_node") {

            RCLCPP_INFO(this->get_logger(), "Controller Node initiated");

            // Initialize desired joint positions and velocities

            joint_state_subscriber = this->create_subscription<sensor_msgs::msg::JointState>("/joint_states", 10, std::bind(&ControllerNode::jointStateCallback, this, std::placeholders::_1));
            desired_state_subscriber = this->create_subscription<sensor_msgs::msg::JointState>("/desired_joint_states", 10, std::bind(&ControllerNode::desiredStateCallback, this, std::placeholders::_1));

            joint_effort_publisher = this->create_publisher<std_msgs::msg::Float64MultiArray>("/effort_controller/commands", 10);
            std::string urdf_path = "/home/shreehank1906/ros2-arm-control/src/arm_sim/urdf/ur5e.urdf";
            pinocchio::urdf::buildModel(urdf_path, model_);
            data_ = std::make_unique<pinocchio::Data>(model_);
            RCLCPP_INFO(this->get_logger(), "Loaded URDF. nq = %d, nv = %d", model_.nq, model_.nv);

            control_timer = this->create_wall_timer(
                std::chrono::milliseconds(10),
                std::bind(&ControllerNode::ComputeControlEffort, this)
            );
        }
        
        // Callback function to update the measured states

        void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg) {
            // Check if the received message has the correct size

            if (msg->position.size() != 6 || msg->velocity.size() != 6) {
                RCLCPP_ERROR(this->get_logger(),"Received joint state message with incorrect size: q size = %zu, qdot size = %zu", msg->position.size(),msg->velocity.size()
            );
            return; }

            // reorder the message into the Pinocchio model (URDF) joint order by name
            if (!reorderToModelOrder(*msg, q_current, qdot_current)) {
                return;
            }

            last_measured_time = this->now();
            has_measured_state = true;
        }

        // receives desired joint targets from the trajectory generator node
        void desiredStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
        {

            if (msg->position.size() != 6 || msg->velocity.size() != 6) {
                RCLCPP_ERROR(this->get_logger(),"Received joint state message with incorrect size: q size = %zu, qdot size = %zu", msg->position.size(),msg->velocity.size()
            );
            return; }
            
            // reorder the message into the Pinocchio model (URDF) joint order by name
            if (!reorderToModelOrder(*msg, q_desired, qdot_desired)) {
                return;
            }

            last_desired_time = this->now();
            has_desired_state = true;
        }

        // compute the control effort using PD control and publish it
        void ComputeControlEffort(){

            if (!has_measured_state || !has_desired_state) {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Waiting for states: measured = %d, desired = %d", has_measured_state, has_desired_state);
                return;
            }

            // do not command effort from stale measured or desired states
            const rclcpp::Time current_time = this->now();
            if ((current_time - last_measured_time).seconds() > state_timeout || (current_time - last_desired_time).seconds() > state_timeout) {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Stale state: measured age = %.3f s, desired age = %.3f s (timeout %.3f s); publishing gravity compensation only", (current_time - last_measured_time).seconds(), (current_time - last_desired_time).seconds(), state_timeout);

                // fallback: hold against gravity at the last measured pose, with no PD terms
                pinocchio::computeGeneralizedGravity(model_, *data_, q_current);
                Eigen::VectorXd fallback_effort = data_->g.cwiseMax(-effort_limit).cwiseMin(effort_limit);
                publishEffort(fallback_effort);
                return;
            }

             if (q_current.size() != 6 || qdot_current.size() != 6 || q_desired.size() != 6 || qdot_desired.size() != 6) {
                RCLCPP_ERROR(this->get_logger(),"Stored state has incorrect size: q_current = %zu, qdot_current = %zu, q_desired = %zu, qdot_desired = %zu", (size_t)q_current.size(), (size_t)qdot_current.size(), (size_t)q_desired.size(), (size_t)qdot_desired.size()
            );
            return; }

            Eigen::VectorXd position_error = q_desired - q_current;
            Eigen::VectorXd velocity_error = qdot_desired - qdot_current;

            // compute generalized gravity for gravity compensation

            pinocchio::computeGeneralizedGravity(model_, *data_, q_current);
            

            // PD control with gravity compensation

            Eigen::VectorXd control_effort = Kp.cwiseProduct(position_error) + Kd.cwiseProduct(velocity_error) + data_->g;

            // limit control effort to a reasonable range
            control_effort = control_effort.cwiseMax(-effort_limit).cwiseMin(effort_limit);
        
            publishEffort(control_effort);

            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Published control effort: [%f, %f, %f, %f, %f, %f]", control_effort[0], control_effort[1], control_effort[2], control_effort[3], control_effort[4], control_effort[5]);

        }

    private:
        // publishes a joint effort vector (URDF joint order) to the effort controller
        void publishEffort(const Eigen::VectorXd &effort) {
            std_msgs::msg::Float64MultiArray effort_msg;
            effort_msg.data.resize(effort.size());

            for (Eigen::Index i = 0; i < effort.size(); ++i) {
                effort_msg.data[i] = effort[i];
            }

            joint_effort_publisher->publish(effort_msg);
        }

        // Reorders a JointState message into the fixed joint order used by the URDF and the effort controller.
        // Returns false and logs an error if a joint is missing or any value is not finite.
        bool reorderToModelOrder(const sensor_msgs::msg::JointState &msg, Eigen::VectorXd &q_out, Eigen::VectorXd &qdot_out) {
            const std::array<std::string, 6> joint_names = {"shoulder_pan_joint","shoulder_lift_joint","elbow_joint","wrist_1_joint","wrist_2_joint","wrist_3_joint"};

            if (msg.name.size() != msg.position.size() || msg.name.size() != msg.velocity.size()) {
                RCLCPP_ERROR(this->get_logger(), "JointState array sizes differ: names=%zu, positions=%zu, velocities=%zu", msg.name.size(), msg.position.size(), msg.velocity.size());
                return false;
            }

            Eigen::VectorXd q_ordered(6);
            Eigen::VectorXd qdot_ordered(6);

            for (std::size_t i = 0; i < joint_names.size(); ++i) {
                const auto it = std::find(msg.name.begin(), msg.name.end(), joint_names[i]);

                if (it == msg.name.end()) {
                    RCLCPP_ERROR(this->get_logger(), "Missing joint: %s", joint_names[i].c_str());
                    return false;
                }

                const auto index = static_cast<std::size_t>(std::distance(msg.name.begin(), it));

                if (!std::isfinite(msg.position[index]) || !std::isfinite(msg.velocity[index])) {
                    RCLCPP_ERROR(this->get_logger(), "Non-finite state for joint: %s", joint_names[i].c_str());
                    return false;
                }

                q_ordered[i] = msg.position[index];
                qdot_ordered[i] = msg.velocity[index];
            }

            q_out = q_ordered;
            qdot_out = qdot_ordered;
            return true;
        }

        Eigen::VectorXd q_current = Eigen::VectorXd::Zero(6);
        Eigen::VectorXd qdot_current = Eigen::VectorXd::Zero(6);
        Eigen::VectorXd q_desired = Eigen::VectorXd::Zero(6);
        Eigen::VectorXd qdot_desired = Eigen::VectorXd::Zero(6);


        // per-joint effort limits, matching the URDF <limit effort=...> values
        Eigen::VectorXd effort_limit = (Eigen::VectorXd(6) << 150.0, 150.0, 150.0, 28.0, 28.0, 28.0).finished();
        bool has_measured_state = false;
        bool has_desired_state = false;
        rclcpp::Time last_measured_time{0, 0, RCL_ROS_TIME};
        rclcpp::Time last_desired_time{0, 0, RCL_ROS_TIME};

        
        // maximum allowed age of the measured/desired state before effort output stops
        const double state_timeout = 0.2;
        Eigen::VectorXd Kp = (Eigen::VectorXd(6) << 30.0, 30.0, 20.0, 5.0, 5.0, 2.0).finished();
        Eigen::VectorXd Kd = (Eigen::VectorXd(6) << 10.0, 10.0, 8.0, 3.0, 3.0, 1.0).finished();
        rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_subscriber;
        rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr desired_state_subscriber;
        rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr joint_effort_publisher;
        rclcpp::TimerBase::SharedPtr control_timer;
        pinocchio::Model model_;
        std::unique_ptr<pinocchio::Data> data_;

};


int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ControllerNode>());
    rclcpp::shutdown();
    return 0;
}