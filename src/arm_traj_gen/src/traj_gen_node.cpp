#include <rclcpp/rclcpp.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <cstddef>

int main(int argc,char** argv)
{
    rclcpp::init(argc,argv);
    rclcpp::executors::SingleThreadedExecutor executor;

    auto node = std::make_shared<rclcpp::Node>("traj_gen_node");
    RCLCPP_INFO(node->get_logger(),"Trajectory generator node started");

    // Load pinocchio model
    pinocchio::Model model;
    pinocchio::urdf::buildModel("/home/shreehank1906/ros2-arm-control/src/arm_sim/urdf/ur5e.urdf", model);
    RCLCPP_INFO(node->get_logger(),"Model loaded successfully");

    // get frames for each joint
    std::size_t i;
    for (std::size_t i=0;i<model.frames.size();i++)
    {
        const pinocchio::Frame& frame = model.frames[i];
        RCLCPP_INFO(node->get_logger(),"Frame index: %d", frame.id);
        RCLCPP_INFO(node->get_logger(),"Frame name: %s",frame.name.c_str());
    }


    executor.add_node(node);
    executor.spin();

    RCLCPP_INFO(node->get_logger(),"Trajectory generator node stopped");
    rclcpp::shutdown();
    return 0;
}