#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <Eigen/Dense>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <memory>

class ControllerNode : public rclcpp::Node {
    public:
        ControllerNode() : Node("controller_node") {
            RCLCPP_INFO(this->get_logger(), "Controller Node initiated");
            joint_state_subscriber = this->create_subscription<sensor_msgs::msg::JointState>("/joint_states", 10, std::bind(&ControllerNode::jointStateCallback, this, std::placeholders::_1));
            joint_effort_publisher = this->create_publisher<std_msgs::msg::Float64MultiArray>("/effort_controller/commands", 10);
            std::string urdf_path = "/home/shreehank1906/ros2-arm-control/src/arm_sim/urdf/ur5e.urdf";
            pinocchio::urdf::buildModel(urdf_path, model_);
            data_ = std::make_unique<pinocchio::Data>(model_);
            RCLCPP_INFO(this->get_logger(), "Loaded URDF. nq = %d, nv = %d", model_.nq, model_.nv);
        }

        void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg) {
            log_counter++;
            Eigen::Map<Eigen::VectorXd> q_map(msg->position.data(), msg->position.size());
            Eigen::Map<Eigen::VectorXd> qdot_map(msg->velocity.data(), msg->velocity.size());
            q_current = q_map;
            qdot_current = qdot_map;
            pinocchio::computeGeneralizedGravity(model_, *data_, q_current);
            Eigen::VectorXd pos_error = q_desired - q_current;
            Eigen::VectorXd vel_error = qdot_desired - qdot_current;

            // Control law: PD control with gravity compensation along with torque saturation.
            Eigen::VectorXd tau = (Kp.array() * pos_error.array() + Kd.array() * vel_error.array()).matrix() + data_->g;
            tau = tau.cwiseMax(-150.0).cwiseMin(150.0);
            std_msgs::msg::Float64MultiArray effort_msg;
            effort_msg.data.resize(tau.size());
            Eigen::Map<Eigen::VectorXd>(effort_msg.data.data(), tau.size()) = tau;
            joint_effort_publisher->publish(effort_msg);
            if (log_counter % 1000 == 0) {
                RCLCPP_INFO(this->get_logger(), "q = [%f, %f, %f, %f, %f, %f]",q_current[0], q_current[1], q_current[2], q_current[3], q_current[4], q_current[5]);
                RCLCPP_INFO(this->get_logger(), "tau = [%f, %f, %f, %f, %f, %f]",tau[0], tau[1], tau[2], tau[3], tau[4], tau[5]);
            }
        }
    private:
        Eigen::VectorXd q_current = Eigen::VectorXd::Zero(6);
        Eigen::VectorXd qdot_current = Eigen::VectorXd::Zero(6);
        Eigen::VectorXd q_desired = Eigen::VectorXd::Zero(6);
        Eigen::VectorXd qdot_desired = Eigen::VectorXd::Zero(6);
        Eigen::VectorXd Kp = (Eigen::VectorXd(6) << 30.0, 30.0, 20.0, 5.0, 5.0, 2.0).finished();
        Eigen::VectorXd Kd = (Eigen::VectorXd(6) << 10.0, 10.0, 8.0, 3.0, 3.0, 1.0).finished();
        int log_counter = 0;
        rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_subscriber;
        rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr joint_effort_publisher;
        pinocchio::Model model_;
        std::unique_ptr<pinocchio::Data> data_;
};
int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ControllerNode>());
    rclcpp::shutdown();
    return 0;
}