//This node subscribes to the /aruco_poses topic and prints the last received pose and
//the new transformed pose.
#include <rclcpp/rclcpp.hpp>
#include <hello_moveit_ur/srv/localize_part.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

class Localizer : public rclcpp::Node
{
public:
    Localizer() : Node("vision_node"), buffer_(this->get_clock()), listener_(buffer_)
    {
        ar_sub_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
            "/aruco_poses",
            rclcpp::QoS(1),
            std::bind(&Localizer::visionCallback, this, std::placeholders::_1));

        server_ = this->create_service<hello_moveit_ur::srv::LocalizePart>(
            "localize_part",
            std::bind(&Localizer::localizePart, this, std::placeholders::_1, std::placeholders::_2));
    }

    void visionCallback(geometry_msgs::msg::PoseArray::SharedPtr msg)
    {
        if (msg->poses.empty())
        {
            RCLCPP_WARN(this->get_logger(), "Received an empty PoseArray.");
            return;
        }

        // Access the last received pose from the PoseArray
        const geometry_msgs::msg::Pose& last_pose = msg->poses.back();

        // Access pose information (e.g., position and orientation)
        double x = last_pose.position.x;
        double y = last_pose.position.y;
        double z = last_pose.position.z;

        double qx = last_pose.orientation.x;
        double qy = last_pose.orientation.y;
        double qz = last_pose.orientation.z;
        double qw = last_pose.orientation.w;

        // Do something with the pose information
        // ...

        // Print the received pose
        RCLCPP_INFO(this->get_logger(), "Received pose from Cam: (%f, %f, %f), (%f, %f, %f, %f)",
                    x, y, z, qx, qy, qz, qw);

        // Store the last received pose for use in localizePart (not recommended, just for demonstration)
        last_received_pose_ = last_pose;
    }

    void localizePart(hello_moveit_ur::srv::LocalizePart::Request::SharedPtr req,
                      hello_moveit_ur::srv::LocalizePart::Response::SharedPtr res)
    {
        if (last_received_pose_.position.x == 0.0 && last_received_pose_.position.y == 0.0 && last_received_pose_.position.z == 0.0)
        {
            RCLCPP_ERROR(this->get_logger(), "No data received.");
            res->success = false;
            return;
        }

        try
        {
            // Define the target pose from the camera
            geometry_msgs::msg::PoseStamped target_pose_from_cam;
            target_pose_from_cam.header.frame_id = "camera_color_optical_frame"; // Replace with the actual frame ID of the camera
            target_pose_from_cam.pose = last_received_pose_;

            // Transform the target pose to the requested frame
            geometry_msgs::msg::PoseStamped target_pose_from_req;
            buffer_.transform(target_pose_from_cam, target_pose_from_req, req->base_frame);

            // Set the transformed pose in the response
            res->pose = target_pose_from_req.pose;
            res->success = true;
        }
        catch (const tf2::TransformException& ex)
        {
            RCLCPP_ERROR(this->get_logger(), "Transform failed: %s", ex.what());
            res->success = false;
        }
    }

private:
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr ar_sub_;
    rclcpp::Service<hello_moveit_ur::srv::LocalizePart>::SharedPtr server_;
    geometry_msgs::msg::Pose last_received_pose_;
    tf2_ros::Buffer buffer_;
    tf2_ros::TransformListener listener_;
};

int main(int argc, char* argv[])
{
    // This must be called before anything else ROS-related
    rclcpp::init(argc, argv);

    // The Localizer class provides this node's ROS interfaces
    auto node = std::make_shared<Localizer>();
    RCLCPP_INFO(node->get_logger(), "Vision node starting");

    // Don't exit the program.
    rclcpp::spin(node);
}
