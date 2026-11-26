#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include "arm_kinodynamics/arm_model.hpp"
#include <Eigen/Dense>
#include <algorithm>
#include <chrono>
#include <exception>
#include <filesystem>
#include <functional>
#include <iterator>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>


using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
using GoalHandle = rclcpp_action::ServerGoalHandle<FollowJointTrajectory>;



class TrajectoryActionServer : public rclcpp::Node
{
public:
      TrajectoryActionServer()
        : Node("trajectory_action_server")
        {
            const std::string urdf_path = resolveUrdfPath();
            const ArmModel arm = loadArmModel(
                urdf_path, {"fr3_finger_joint1", "fr3_finger_joint2"});
            joint_names_ = arm.joint_names;

            q_hold_ = Eigen::VectorXd::Zero(static_cast<Eigen::Index>(joint_names_.size()));
            q_actual_ = Eigen::VectorXd::Zero(static_cast<Eigen::Index>(joint_names_.size()));

            // creating a subscriber 
            joint_states_subscriber_ = create_subscription<sensor_msgs::msg::JointState>("/joint_states", 10, std::bind(&TrajectoryActionServer::jointStateCallback, this, std::placeholders::_1));

            joint_state_publisher_ = create_publisher<sensor_msgs::msg::JointState>("/desired_joint_states", 10);

            wall_timer_ = create_wall_timer(
                std::chrono::milliseconds(1),
                std::bind(&TrajectoryActionServer::publishJointState, this));

            q_desired_ = Eigen::VectorXd::Zero(static_cast<Eigen::Index>(joint_names_.size()));
            qd_desired_ = Eigen::VectorXd::Zero(static_cast<Eigen::Index>(joint_names_.size()));

            // action server MoveIt sends trajectories to (name must match moveit_controllers.yaml)
            action_server_ = rclcpp_action::create_server<FollowJointTrajectory>(
                this,
                "/fr3_arm_controller/follow_joint_trajectory",
                std::bind(&TrajectoryActionServer::handleGoal, this, std::placeholders::_1, std::placeholders::_2),
                std::bind(&TrajectoryActionServer::handleCancel, this, std::placeholders::_1),
                std::bind(&TrajectoryActionServer::handleAccepted, this, std::placeholders::_1));

            RCLCPP_INFO(
                get_logger(), "Trajectory Action Server initialized.");
        }

private:
    std::string resolveUrdfPath()
    {
        std::string path = declare_parameter<std::string>("urdf_path", "");
        if (path.empty()) {
            try {
                path = ament_index_cpp::get_package_share_directory("arm_sim") +
                    "/urdf/fr3.urdf";
            } catch (const std::exception &e) {
                throw std::runtime_error(
                    std::string("Cannot locate package 'arm_sim' to find fr3.urdf "
                                "(is it built and sourced?): ") + e.what());
            }
        }
        if (!std::filesystem::exists(path)) {
            throw std::runtime_error("URDF file not found: " + path);
        }
        return path;
    }

    void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msgs)
    {
        // /joint_states also carries the finger joints, so only check the message against itself
        if (msgs->position.size() != msgs->name.size()) {
            RCLCPP_ERROR_THROTTLE(
                get_logger(), *get_clock(), 1000,
                "Received joint state with %zu positions for %zu names.",
                msgs->position.size(), msgs->name.size());
            return;
        }

        // pick the arm joints out of the message by name, in model order (joint_names_)
        Eigen::VectorXd q = Eigen::VectorXd::Zero(static_cast<Eigen::Index>(joint_names_.size()));
        for (std::size_t i = 0; i < joint_names_.size(); ++i) {
            const auto it = std::find(msgs->name.begin(), msgs->name.end(), joint_names_[i]);
            if (it == msgs->name.end()) {
                RCLCPP_ERROR_THROTTLE(
                    get_logger(), *get_clock(), 1000,
                    "%s not found in /joint_states", joint_names_[i].c_str());
                return;
            }
            q[static_cast<Eigen::Index>(i)] =
                msgs->position[static_cast<std::size_t>(std::distance(msgs->name.begin(), it))];
        }
        q_actual_ = q;

        // hold the first measured pose until a trajectory arrives
        if (!has_hold_) {
            q_hold_ = q_actual_;
            has_hold_ = true;
            RCLCPP_INFO(get_logger(), "Holding initial position.");
        }
    }

    // publishes a desired position and velocity (model joint order) to /desired_joint_states
    void publishDesired(const Eigen::VectorXd &q_desired, const Eigen::VectorXd &qd_desired)
    {
        sensor_msgs::msg::JointState msg;
        msg.header.stamp = now();
        msg.name = joint_names_;
        msg.position.resize(joint_names_.size());
        msg.velocity.resize(joint_names_.size());
        for (std::size_t i = 0; i < joint_names_.size(); ++i) {
            msg.position[i] = q_desired[static_cast<Eigen::Index>(i)];
            msg.velocity[i] = qd_desired[static_cast<Eigen::Index>(i)];
        }
        joint_state_publisher_->publish(msg);
    }

    void publishJointState()
    {
        if (!has_hold_) {
            return;
        }

        const Eigen::VectorXd zeros =
            Eigen::VectorXd::Zero(static_cast<Eigen::Index>(joint_names_.size()));

        if (active_goal_ && !active_goal_->is_active()) {
            active_goal_ = nullptr;
        }

        if (!active_goal_) {
            q_desired_ = q_hold_;
            publishDesired(q_hold_, zeros);
            return;
        }

        if (active_goal_->is_canceling()) {
            RCLCPP_INFO(get_logger(), "Canceling active trajectory goal.");
            q_hold_ = q_desired_;
            auto result = std::make_shared<FollowJointTrajectory::Result>();
            result->error_string = "canceled";
            active_goal_->canceled(result);
            active_goal_ = nullptr;
            publishDesired(q_hold_, zeros);
            return;
        }

        const double t = (now() - start_time_).seconds();

        if (t >= trajectory_times_.back()) {
            RCLCPP_INFO(get_logger(), "Trajectory goal completed.");
            q_hold_ = trajectory_positions_.back();
            q_desired_ = q_hold_;
            publishDesired(q_hold_, zeros);
            auto result = std::make_shared<FollowJointTrajectory::Result>();
            result->error_code = FollowJointTrajectory::Result::SUCCESSFUL;
            active_goal_->succeed(result);
            active_goal_ = nullptr;
            return;
        }

        std::size_t segment_index = 0;
        while (segment_index + 1 < trajectory_times_.size() - 1 &&
               t > trajectory_times_[segment_index + 1]) {
            ++segment_index;
        }

        SampleSegment(
            trajectory_positions_[segment_index], trajectory_velocities_[segment_index],
            trajectory_positions_[segment_index + 1], trajectory_velocities_[segment_index + 1],
            trajectory_times_[segment_index], trajectory_times_[segment_index + 1], t,
            q_desired_, qd_desired_);

        publishDesired(q_desired_, qd_desired_);

        if (feedback_counter_++ % 10 == 0) {
            auto feedback_msg = std::make_shared<FollowJointTrajectory::Feedback>();
            feedback_msg->header.stamp = now();
            feedback_msg->joint_names = joint_names_;
            feedback_msg->desired.positions.resize(joint_names_.size());
            feedback_msg->desired.velocities.resize(joint_names_.size());
            feedback_msg->actual.positions.resize(joint_names_.size());
            feedback_msg->error.positions.resize(joint_names_.size());
            for (std::size_t i = 0; i < joint_names_.size(); ++i) {
                const auto idx = static_cast<Eigen::Index>(i);
                feedback_msg->desired.positions[i] = q_desired_[idx];
                feedback_msg->desired.velocities[i] = qd_desired_[idx];
                feedback_msg->actual.positions[i] = q_actual_[idx];
                feedback_msg->error.positions[i] = q_desired_[idx] - q_actual_[idx];
            }
            active_goal_->publish_feedback(feedback_msg);
        }
    }

    void SampleSegment(
        const Eigen::VectorXd &q_start, const Eigen::VectorXd &qd_start,
        const Eigen::VectorXd &q_end, const Eigen::VectorXd &qd_end,
        double t_start, double t_end, double t_sample,
        Eigen::VectorXd &q_sample, Eigen::VectorXd &qd_sample)
    {
        
        const double T = t_end - t_start;

        if (T <= 0.0) {
            q_sample = q_end;
            qd_sample = qd_end;
            return;
        }

       
        const double s = std::clamp((t_sample - t_start) / T, 0.0, 1.0);
        const double s2 = s * s;
        const double s3 = s2 * s;

        
        const double h00 = 2.0 * s3 - 3.0 * s2 + 1.0;
        const double h10 = s3 - 2.0 * s2 + s;
        const double h01 = -2.0 * s3 + 3.0 * s2;
        const double h11 = s3 - s2;

        
        const double dh00 = 6.0 * s2 - 6.0 * s;
        const double dh10 = 3.0 * s2 - 4.0 * s + 1.0;
        const double dh01 = -6.0 * s2 + 6.0 * s;
        const double dh11 = 3.0 * s2 - 2.0 * s;

      
        q_sample = h00 * q_start + h10 * T * qd_start + h01 * q_end + h11 * T * qd_end;
        qd_sample = (dh00 * q_start + dh10 * T * qd_start + dh01 * q_end + dh11 * T * qd_end) / T;
    }

    std::vector<std::string> joint_names_;
    Eigen::VectorXd q_hold_;
    Eigen::VectorXd q_actual_;
    bool has_hold_ = false;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_publisher_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_states_subscriber_;
    rclcpp::TimerBase::SharedPtr wall_timer_;

    rclcpp_action::Server<FollowJointTrajectory>::SharedPtr action_server_;
    std::shared_ptr<GoalHandle> active_goal_ = nullptr;
    std::vector<double> trajectory_times_;
    std::vector<Eigen::VectorXd> trajectory_positions_;
    std::vector<Eigen::VectorXd> trajectory_velocities_;
    bool has_velocities_ = true;
    rclcpp::Time start_time_;
    Eigen::VectorXd q_desired_;
    Eigen::VectorXd qd_desired_;
    int feedback_counter_ = 0;

    rclcpp_action::GoalResponse handleGoal(
        const rclcpp_action::GoalUUID &uuid,
        std::shared_ptr<const FollowJointTrajectory::Goal> goal)
    {
        (void)uuid;
        const auto &trajectory = goal->trajectory;
        RCLCPP_INFO(get_logger(), "Received trajectory goal request with %zu points.", trajectory.points.size());

        // no joint state yet, so there is no position to start the trajectory from
        if (!has_hold_) {
            RCLCPP_ERROR(get_logger(), "Rejecting goal: no joint state received yet.");
            return rclcpp_action::GoalResponse::REJECT;
        }

        if (trajectory.points.empty()) {
            RCLCPP_ERROR(get_logger(), "Rejecting goal: trajectory has no points.");
            return rclcpp_action::GoalResponse::REJECT;
        }

        // exactly the arm joints, in any order
        if (trajectory.joint_names.size() != joint_names_.size()) {
            RCLCPP_ERROR(get_logger(), "Rejecting goal: trajectory has %zu joints, expected %zu.",
                trajectory.joint_names.size(), joint_names_.size());
            return rclcpp_action::GoalResponse::REJECT;
        }
        for (const auto &name : joint_names_) {
            if (std::find(trajectory.joint_names.begin(), trajectory.joint_names.end(), name) ==
                trajectory.joint_names.end()) {
                RCLCPP_ERROR(get_logger(), "Rejecting goal: %s missing from the trajectory.", name.c_str());
                return rclcpp_action::GoalResponse::REJECT;
            }
        }

        // every point must have one position per joint, and either no velocities or one per joint
        for (const auto &point : trajectory.points) {
            if (point.positions.size() != joint_names_.size() ||
                (!point.velocities.empty() && point.velocities.size() != joint_names_.size())) {
                RCLCPP_ERROR(get_logger(), "Rejecting goal: point has %zu positions and %zu velocities, expected %zu.",
                    point.positions.size(), point.velocities.size(), joint_names_.size());
                return rclcpp_action::GoalResponse::REJECT;
            }
        }

        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handleCancel(
        const std::shared_ptr<GoalHandle> goal_handle)
    {
        (void)goal_handle;
        RCLCPP_INFO(get_logger(), "Received request to cancel trajectory goal.");
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handleAccepted(const std::shared_ptr<GoalHandle> goal_handle)
    {

        // to cancel the previous goal if it is still active, and hold the last commanded position
        if (active_goal_ && active_goal_->is_active()) {
            RCLCPP_WARN(get_logger(), "Cancelling previous active goal.");
            auto result = std::make_shared<FollowJointTrajectory::Result>();
            result->error_string = "preempted by a new goal";
            active_goal_->abort(result);
            q_hold_ = q_desired_;
        }

        const auto &trajectory = goal_handle->get_goal()->trajectory;

        // reordering. basically index[i] is where joint_names_[i] sits in the goal's joint_names
        // (handleGoal has already checked that every arm joint is present)
        std::vector<std::size_t> index(joint_names_.size());
        for (std::size_t i = 0; i < joint_names_.size(); ++i) {
            const auto it = std::find(
                trajectory.joint_names.begin(), trajectory.joint_names.end(), joint_names_[i]);
            index[i] = static_cast<std::size_t>(std::distance(trajectory.joint_names.begin(), it));
        }

        // fill the trajectory in model joint order
        trajectory_times_.clear();
        trajectory_positions_.clear();
        trajectory_velocities_.clear();
        has_velocities_ = std::all_of(trajectory.points.begin(), trajectory.points.end(),[](const auto &point) { return !point.velocities.empty(); });

        for (const auto &point : trajectory.points) {

            trajectory_times_.push_back(rclcpp::Duration(point.time_from_start).seconds());

            Eigen::VectorXd q = Eigen::VectorXd::Zero(static_cast<Eigen::Index>(joint_names_.size()));
            Eigen::VectorXd qd = Eigen::VectorXd::Zero(static_cast<Eigen::Index>(joint_names_.size()));
            
            for (std::size_t i = 0; i < joint_names_.size(); ++i) {
                q[static_cast<Eigen::Index>(i)] = point.positions[index[i]];
                
                if (has_velocities_) {
                    qd[static_cast<Eigen::Index>(i)] = point.velocities[index[i]];
                }
            }
            trajectory_positions_.push_back(q);
            trajectory_velocities_.push_back(qd);
        }

        // start from the held position: if the first point is later than t = 0, insert the held
        // position at t = 0 so the arm moves there smoothly instead of jumping to it
        if (trajectory_times_.front() > 0.0) {
            trajectory_times_.insert(trajectory_times_.begin(), 0.0);
            trajectory_positions_.insert(trajectory_positions_.begin(), q_hold_);
            trajectory_velocities_.insert(
                trajectory_velocities_.begin(),
                Eigen::VectorXd::Zero(static_cast<Eigen::Index>(joint_names_.size())));
        }

        active_goal_ = goal_handle;
        start_time_ = now();
        feedback_counter_ = 0;
    }
};

int main (int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TrajectoryActionServer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

