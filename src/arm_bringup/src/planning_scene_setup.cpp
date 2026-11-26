// Adds the robot platform and the ground to MoveIt's planning scene, then exits.
// The geometry comes from parameters (config/planning_scene.yaml), in MoveIt's planning frame.

#include <rclcpp/rclcpp.hpp>
#include <moveit_msgs/srv/apply_planning_scene.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include <chrono>
#include <string>
#include <vector>

// a box collision object of the given size (x, y, z), centred at position
moveit_msgs::msg::CollisionObject makeBox(const std::string &id, const std::string &frame,
                                          const std::vector<double> &size,
                                          const std::vector<double> &position)
{
    moveit_msgs::msg::CollisionObject obj;
    obj.header.frame_id = frame;
    obj.id = id;

    shape_msgs::msg::SolidPrimitive box;
    box.type = shape_msgs::msg::SolidPrimitive::BOX;
    box.dimensions = {size[0], size[1], size[2]};   // bounded vector of 3: x, y, z

    geometry_msgs::msg::Pose pose;
    pose.position.x = position[0];
    pose.position.y = position[1];
    pose.position.z = position[2];
    pose.orientation.w = 1.0;

    obj.primitives.push_back(box);
    obj.primitive_poses.push_back(pose);
    obj.operation = moveit_msgs::msg::CollisionObject::ADD;
    return obj;
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("planning_scene_setup");

    const auto frame_id = node->declare_parameter<std::string>("frame_id", "world");
    const auto platform_size = node->declare_parameter<std::vector<double>>("platform_size", {1.2, 1.2, 0.4});
    const auto platform_position = node->declare_parameter<std::vector<double>>("platform_position", {0.0, 0.0, -0.205});
    const auto ground_size = node->declare_parameter<std::vector<double>>("ground_size", {4.0, 4.0, 0.02});
    const auto ground_position = node->declare_parameter<std::vector<double>>("ground_position", {0.0, 0.0, -0.41});

    for (const auto *v : {&platform_size, &platform_position, &ground_size, &ground_position}) {
        if (v->size() != 3) {
            RCLCPP_ERROR(node->get_logger(), "Box sizes and positions need exactly 3 values (x, y, z).");
            rclcpp::shutdown();
            return 1;
        }
    }

    auto client = node->create_client<moveit_msgs::srv::ApplyPlanningScene>("/apply_planning_scene");

    // move_group provides the service once it is up
    while (!client->wait_for_service(std::chrono::seconds(2))) {
        if (!rclcpp::ok()) {
            rclcpp::shutdown();
            return 1;
        }
        RCLCPP_INFO(node->get_logger(), "Waiting for move_group (/apply_planning_scene)...");
    }

    auto request = std::make_shared<moveit_msgs::srv::ApplyPlanningScene::Request>();
    // a diff only adds these objects; a full scene would replace everything, robot state included
    request->scene.is_diff = true;
    request->scene.world.collision_objects.push_back(makeBox("platform", frame_id, platform_size, platform_position));
    request->scene.world.collision_objects.push_back(makeBox("ground", frame_id, ground_size, ground_position));

    auto future = client->async_send_request(request);
    const auto status = rclcpp::spin_until_future_complete(node, future, std::chrono::seconds(10));

    if (status == rclcpp::FutureReturnCode::SUCCESS && future.get()->success) {
        RCLCPP_INFO(node->get_logger(), "Added platform and ground to the planning scene.");
        rclcpp::shutdown();
        return 0;
    }

    RCLCPP_ERROR(node->get_logger(), "Failed to add platform and ground to the planning scene.");
    rclcpp::shutdown();
    return 1;
}
