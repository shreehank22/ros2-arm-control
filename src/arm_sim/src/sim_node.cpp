#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/rnea.hpp>  
#include <iostream>
#include <memory>
class SimNode : public rclcpp::Node {
    public:
        SimNode() : Node("sim_node") {
            RCLCPP_INFO(this->get_logger(), "SimNode constructor started");
            std::string urdf_path = "/home/shreehank1906/ros2-arm-control/src/arm_sim/urdf/ur5e.urdf";
            pinocchio::urdf::buildModel(urdf_path, model_);
            data_ = std::make_unique<pinocchio::Data>(model_);
            q_ = Eigen::VectorXd::Constant(model_.nq, 0.3);
            q_dot_ = Eigen::VectorXd::Constant(model_.nv, 0.0);
            RCLCPP_INFO(this->get_logger(), "size of q: %ld, size of q_dot: %ld", q_.size(), q_dot_.size());
            RCLCPP_INFO(this->get_logger(), "Data allocated. M rows = %ld", data_->M.rows());
            RCLCPP_INFO(this->get_logger(), "SimNode constructor finished");
            RCLCPP_INFO(this->get_logger(), "Loaded URDF. nq = %d, nv = %d", model_.nq, model_.nv);
        }
    private:
        pinocchio::Model model_;
        std::unique_ptr<pinocchio::Data> data_;
        Eigen::VectorXd q_;
        Eigen::VectorXd q_dot_;
};


int main(int args,char ** argv) {
    std::cout << "main() started, calling rclcpp::init" << std::endl;
    rclcpp::init(args, argv);

    std::cout <<"creating SimNode and calling spin"<<std::endl;
    rclcpp::spin(std::make_shared<SimNode>());

    std::cout <<"spin() returned, calling shutdown"<<std::endl;
    rclcpp::shutdown();
    return 0;
}