#include <rclcpp/rclcpp.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

// TODO:
// - quintic polynomial q(t) from q0 -> qf over duration T (zero vel/accel at endpoints)
// - publish /joint_trajectory at a fixed rate, or as one JointTrajectory with
//   multiple JointTrajectoryPoints and let controller_node interpolate

class TrajGenNode : public rclcpp::Node {
public:
  TrajGenNode() : Node("traj_gen_node") {
    RCLCPP_INFO(this->get_logger(), "traj_gen_node started");
    // TODO: declare_parameter for q0, qf, duration; create publisher, timer
  }
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TrajGenNode>());
  rclcpp::shutdown();
  return 0;
}
