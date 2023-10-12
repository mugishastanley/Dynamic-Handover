#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>

class RobotArmController
{
public:
    RobotArmController()
    {
        // Initialize your class members here, if necessary.
    }

    void moveRobotArmThroughPoses(const std::vector<geometry_msgs::msg::Pose>& target_poses, rclcpp::Node::SharedPtr node)
    {
        // Create a MoveGroupInterface instance for the robot arm
        moveit::planning_interface::MoveGroupInterface move_group_interface(node, "ur_manipulator");

        // Set the planning time and maximum velocity scaling factor
        move_group_interface.setPlanningTime(10.0);
        move_group_interface.setMaxVelocityScalingFactor(0.5);

        // Iterate through the target poses and move the robot arm
        for (const auto& target_pose : target_poses)
        {
            // Set the target pose for the move group
            move_group_interface.setPoseTarget(target_pose);

            // Plan and execute the motion
            moveit::planning_interface::MoveGroupInterface::Plan plan;
            if (move_group_interface.plan(plan))
            {
                move_group_interface.execute(plan);
            }
            else
            {
                RCLCPP_ERROR(node->get_logger(), "Failed to plan motion to target pose");
            }
        }
    }
};

void targetPoseCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg, RobotArmController& controller, rclcpp::Node::SharedPtr node)
{
    if (msg->data.size() % 7 != 0)
    {
        RCLCPP_ERROR(node->get_logger(), "Invalid pose message format");
        return;
    }

    std::vector<geometry_msgs::msg::Pose> target_poses;
    for (size_t i = 0; i < msg->data.size(); i += 7)
    {
        geometry_msgs::msg::Pose pose;
        pose.position.x = msg->data[i];
        pose.position.y = msg->data[i + 1];
        pose.position.z = msg->data[i + 2];
        pose.orientation.x = msg->data[i + 3];
        pose.orientation.y = msg->data[i + 4];
        pose.orientation.z = msg->data[i + 5];
        pose.orientation.w = msg->data[i + 6];
        target_poses.push_back(pose);
    }

    controller.moveRobotArmThroughPoses(target_poses, node);
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    // Create a ROS Node
    auto node = std::make_shared<rclcpp::Node>(
        "moveit_receive_move",
        rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

    RobotArmController controller;

    // Create a subscriber for receiving target poses
    auto target_pose_subscriber = node->create_subscription<std_msgs::msg::Float32MultiArray>(
        "target_poses", 10, [&](const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
            targetPoseCallback(msg, controller, node);
        });

    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}
