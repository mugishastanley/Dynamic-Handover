#include <rclcpp/rclcpp.hpp>
#include <workcell_core/srv/localize_part.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

class PoseTransformer : public rclcpp::Node
{
public:
    PoseTransformer() : Node("pose_transformer_node")
    {
        // Create a client to call the localize_part service
        client_ = this->create_client<workcell_core::srv::LocalizePart>("localize_part");

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

        // Create a timer to periodically request the transformed pose
        timer_ = this->create_wall_timer(std::chrono::seconds(5), std::bind(&PoseTransformer::requestTransformedPose, this));

        // Initialize the pose request with the desired base frame
        pose_request_.base_frame = "world"; // Replace with your desired base frame
    }

    void requestTransformedPose()
    {
        auto request = std::make_shared<workcell_core::srv::LocalizePart::Request>(pose_request_);

        // Call the localize_part service asynchronously
        auto future = client_->async_send_request(request, [this](rclcpp::Client<workcell_core::srv::LocalizePart>::SharedFuture future)
        {
            if (future.get()->success)
            {
                // Print the transformed pose
                const auto& transformed_pose = future.get()->pose;
                double x = transformed_pose.position.x;
                double y = transformed_pose.position.y;
                double z = transformed_pose.position.z;
                double qx = transformed_pose.orientation.x;
                double qy = transformed_pose.orientation.y;
                double qz = transformed_pose.orientation.z;
                double qw = transformed_pose.orientation.w;

                RCLCPP_INFO(this->get_logger(), "Transformed pose: (%f, %f, %f), (%f, %f, %f, %f)",
                            x, y, z, qx, qy, qz, qw);
            }
            else
            {
                RCLCPP_ERROR(this->get_logger(), "Failed to request transformed pose.");
            }
        });
    }

private:
    rclcpp::Client<workcell_core::srv::LocalizePart>::SharedPtr client_;
    rclcpp::TimerBase::SharedPtr timer_;
    workcell_core::srv::LocalizePart::Request pose_request_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PoseTransformer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
