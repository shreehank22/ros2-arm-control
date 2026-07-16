#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

// TODO:
// - declare parameters kp, kd, gravity_compensation (load from gains.yaml)
// - subscribe /joint_states (actual q, qdot) and /joint_trajectory (desired)
// - each tick: tau = Kp*(q_des - q) + Kd*(qdot_des - qdot) + G(q) if enabled
// - publish /joint_effort (std_msgs/Float64MultiArray)

class ControllerNode : public rclcpp::Node {
public:
  ControllerNode() : Node("controller_node") {
    RCLCPP_INFO(this->get_logger(), "controller_node started");
    // TODO: declare_parameter, create publisher, subscribers, timer
  }
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControllerNode>());
  rclcpp::shutdown();
  return 0;
}
