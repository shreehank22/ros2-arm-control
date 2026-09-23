#include <rclcpp/rclcpp.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <cstddef>
#include <Eigen/Dense>

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
    pinocchio::FrameIndex ee_frameID = model.getFrameId("tool0");

    // forward kinematics test
    Eigen::VectorXd q0(6);
    q0 << 0.0, -M_PI/2, 0.0, -M_PI/2, 0.0, 0.0;
    pinocchio::Data data(model);
    pinocchio::forwardKinematics(model, data, q0);
    pinocchio::updateFramePlacements(model, data);
    pinocchio::SE3 ee_pose = data.oMf[ee_frameID];
    Eigen::Matrix3d ee_rotation = ee_pose.rotation();
    Eigen::Vector3d ee_position = ee_pose.translation();

    RCLCPP_INFO(node->get_logger(),"Frame index of end-effector: %zu", ee_frameID);
    RCLCPP_INFO(node->get_logger(),"End-effector position: [%f %f %f]", ee_position.x(), ee_position.y(), ee_position.z());
    RCLCPP_INFO(node->get_logger(),"End-effector rotation matrix:\n[%f %f %f]\n[%f %f %f]\n[%f %f %f]",
                ee_rotation(0,0), ee_rotation(0,1), ee_rotation(0,2),
                ee_rotation(1,0), ee_rotation(1,1), ee_rotation(1,2),
                ee_rotation(2,0), ee_rotation(2,1), ee_rotation(2,2));
    
                



    executor.add_node(node);
    executor.spin();

    RCLCPP_INFO(node->get_logger(),"Trajectory generator node stopped");
    rclcpp::shutdown();
    return 0;
}