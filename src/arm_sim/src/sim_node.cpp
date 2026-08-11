#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/rnea.hpp>  
#include <iostream>
#include <memory>
#include <Eigen/Dense>
#include <pinocchio/algorithm/crba.hpp>

#include <chrono>

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
            timer_ = this->create_wall_timer(std::chrono::microseconds(100),[this]() {this->onTimer(); });
            RCLCPP_INFO(this->get_logger(),"Timer created with period %f seconds", dt);
            RCLCPP_INFO(this->get_logger(), "Loaded URDF. nq = %d, nv = %d", model_.nq, model_.nv);
            RCLCPP_INFO(this->get_logger(), "Data allocated. M rows = %ld", data_->M.rows());
            RCLCPP_INFO(this->get_logger(), "size of q: %ld, size of q_dot: %ld", q_.size(), q_dot_.size());
            RCLCPP_INFO(this->get_logger(), "Publisher created on /joint_states");
            RCLCPP_INFO(this->get_logger(), "SimNode constructor finished");
            
        }
        void onTimer(){
            Eigen::VectorXd nle = pinocchio::nonLinearEffects(model_, *data_, q_, q_dot_);
            pinocchio::crba(model_, *data_, q_);
            data_->M.triangularView<Eigen::StrictlyLower>() = data_->M.transpose().triangularView<Eigen::StrictlyLower>();
            Eigen::VectorXd tau=Eigen::VectorXd::Zero(model_.nv);
            Eigen::VectorXd qddot = data_->M.inverse() * (tau - nle);
            q_dot_ += qddot * dt;
            q_ += q_dot_ * dt;
            sensor_msgs::msg::JointState msg;
            msg.header.stamp = this->now();
            for (size_t i=1; i<model_.names.size(); i++) {
                msg.name.push_back(model_.names[i]);
            }
            msg.position.assign(q_.data(), q_.data() + q_.size());
            msg.velocity.assign(q_dot_.data(), q_dot_.data() + q_dot_.size());
            joint_state_publisher->publish(msg);
            RCLCPP_INFO(this->get_logger(), "q_[0] = %f, qdot_[0] = %f", q_[0], q_dot_[0]);
        }
    private:
        pinocchio::Model model_;
        std::unique_ptr<pinocchio::Data> data_;
        Eigen::VectorXd q_;
        Eigen::VectorXd q_dot_;
        rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_publisher;
        rclcpp::TimerBase::SharedPtr timer_;
        double dt = 0.0001;
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