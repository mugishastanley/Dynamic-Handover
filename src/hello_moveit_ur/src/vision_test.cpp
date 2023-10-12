// A client which sends a pose request to vision node and recieves a transformed pose in the specified frame.

#include <rclcpp/rclcpp.hpp>
#include <hello_moveit_ur/srv/localize_part.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

using namespace std::chrono_literals;

class PoseTransformer : public rclcpp::Node
{
public:
    PoseTransformer() : Node("vision_test")
    {
        // Create a client to call the localize_part service
        client_ = this->create_client<hello_moveit_ur::srv::LocalizePart>("localize_part");

        // Wait for the service to become available
        while (!client_->wait_for_service(std::chrono::seconds(1)))
        {
            if (!rclcpp::ok())
            {
                RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
                return;
            }
            RCLCPP_INFO(this->get_logger(), "Waiting for the localize_part service to become available...");
        }

        // Create a timer to periodically request the transformed pose or just a publisher, but one of these
        //timer_ = this->create_wall_timer(std::chrono::seconds(1), std::bind(&PoseTransformer::requestTransformedPose, this));

        publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/ar_marker_1", 5); //queue size is 5
        timer_ = this->create_wall_timer(
        10ms, std::bind(&PoseTransformer::requestTransformedPose, this));
        
        // Initialize the pose request with the desired base frame
        pose_request_.base_frame = "base_link"; // Replace with your desired base frame
    }

    void requestTransformedPose()
    {
        auto request = std::make_shared<hello_moveit_ur::srv::LocalizePart::Request>(pose_request_);

        // Call the localize_part service asynchronously
        auto future = client_->async_send_request(request, [this](rclcpp::Client<hello_moveit_ur::srv::LocalizePart>::SharedFuture future)
        {
            if (future.get()->success)
            {
                geometry_msgs::msg::PoseStamped t;
                t.header.stamp = this->get_clock()->now();
                t.header.frame_id = "base_link";
                

                // Print the transformed pose
                const auto& transformed_pose = future.get()->pose;
                float x = transformed_pose.position.x;
                float y = transformed_pose.position.y;
                float z = transformed_pose.position.z;
                float qx = transformed_pose.orientation.x;
                float qy = transformed_pose.orientation.y;
                float qz = transformed_pose.orientation.z;
                float qw = transformed_pose.orientation.w;

               RCLCPP_INFO(this->get_logger(), "Publishing Transformed pose: (%f, %f, %f), (%f, %f, %f, %f)",
                          x, y, z, qx, qy, qz, qw);
            //Do something with the pose.
            // For now we shall just publish the transformed pose to a topic.
            t.pose=transformed_pose;
            publisher_->publish(t);
            }
            else
            {
                RCLCPP_ERROR(this->get_logger(), "Failed to request transformed pose.");
            }
        });
    }

private:
    rclcpp::Client<hello_moveit_ur::srv::LocalizePart>::SharedPtr client_;
    rclcpp::TimerBase::SharedPtr timer_;
    hello_moveit_ur::srv::LocalizePart::Request pose_request_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PoseTransformer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
