#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include "geometry_msgs/msg/transform_stamped.hpp"
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/buffer.h>

using namespace std::chrono_literals;

class ObjectLocalizer : public rclcpp::Node
{
public:
  ObjectLocalizer()
    : Node("object_localizer")
  {
    publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/object_pose", 10);

    // Setup the transform listener
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // Subscribe to the camera pose topic
    subscription_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/aruco_poses", 10, [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        // Transform object pose from camera to world coordinates
        geometry_msgs::msg::TransformStamped cam_to_world;

                  // Lookup the transform from camera frame to world frame
          //harcoded values

          cam_to_world.header.frame_id = "Camera_to_world";
          cam_to_world.child_frame_id = "object_pose";
          cam_to_world.transform.translation.x = 0.2689;
          cam_to_world.transform.translation.y = -0.197;
          cam_to_world.transform.translation.z = 0.082;
          cam_to_world.transform.rotation.x = 0.6519802;
          cam_to_world.transform.rotation.y  = -0.2210103;
          cam_to_world.transform.rotation.z  = -0.2996899;
          cam_to_world.transform.rotation.w  = 0.6605015;

        geometry_msgs::msg::PoseStamped object_pose;
        try {


          geometry_msgs::msg::TransformStamped transformStamped = tf_buffer_->lookupTransform(
            "world_frame", msg->header.frame_id, msg->header.stamp, tf2::durationFromSec(1.0));
          
          // Perform the transformation
          //tf2::doTransform(*msg, object_pose, transformStamped);
          tf2::doTransform(*msg, object_pose, cam_to_world);


          tf2::doTransform(*msg, cam_to_world, transformStamped);
          object_pose.header.frame_id = "world_frame";
          
          // Publish the transformed object pose
          publisher_->publish(object_pose);
        } catch (const tf2::TransformException &ex) {
          RCLCPP_ERROR(this->get_logger(), "Transform lookup failed: %s", ex.what());
        }
      });
  }

private:
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr subscription_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ObjectLocalizer>();
  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}
