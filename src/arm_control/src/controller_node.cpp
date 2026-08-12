#include <rclcpp/rclcpp.hpp>
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "iostream"
#include "Eigen/Dense"

class ControllerNode : public rclcpp::Node{
    public:
        ControllerNode() : Node("controller_node") {
            RCLCPP_INFO(this->get_logger(),"Controller Node initiated");
            joint_state_subscriber = this->create_subscription<sensor_msgs::msg::JointState>(
                "/joint_states", 10, std::bind(&ControllerNode::jointStateCallback, this, std::placeholders::_1));
                RCLCPP_INFO(this->get_logger(),"Subscribed to /joint_states");
        }
        void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg) {
            Eigen::Map<Eigen::VectorXd> q_map(msg->position.data(), msg->position.size());
            Eigen::Map<Eigen::VectorXd> qdot_map(msg->velocity.data(), msg->velocity.size());
            q_current = q_map;
            qdot_current = qdot_map;
            RCLCPP_INFO(this->get_logger(), "Received joint state: q[0] = %f, qdot[0] = %f", q_current[0], qdot_current[0]);
        }
    private:
        Eigen::VectorXd q_current = Eigen::VectorXd::Zero(6);
        Eigen::VectorXd qdot_current = Eigen::VectorXd::Zero(6);
        rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_subscriber;

        
};

int main(int args,char ** argv) {
    std::cout << "main() started, calling rclcpp::init" << std::endl;
    rclcpp::init(args, argv);

    std::cout <<"creating ControllerNode and calling spin"<<std::endl;
    rclcpp::spin(std::make_shared<ControllerNode>());

    std::cout <<"spin() returned, calling shutdown"<<std::endl;
    rclcpp::shutdown();
    return 0;
}