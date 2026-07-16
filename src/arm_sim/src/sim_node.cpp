#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include "arm_sim/dynamics.hpp"

// TODO:
// - subscribe to /joint_effort (commanded torque, std_msgs/Float64MultiArray)
// - each timer tick: qddot = M(q).inverse() * (tau - C(q,qdot) - G(q))
// - semi-implicit Euler: qdot += qddot*dt; q += qdot*dt
// - publish /joint_states (sensor_msgs/JointState) with q, qdot

class SimNode : public rclcpp::Node {
public:
  SimNode() : Node("sim_node") {
    RCLCPP_INFO(this->get_logger(), "sim_node started");
    // TODO: create publisher, subscriber, timer
  }

private:
  arm_sim::Dynamics dynamics_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SimNode>());
  rclcpp::shutdown();
  return 0;
}
