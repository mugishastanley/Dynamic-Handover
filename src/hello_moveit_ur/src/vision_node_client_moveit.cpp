// This node moves the robot arm to a target pose received from the camera.
// In this case, the target pose is set to the dynamic object
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

using std::placeholders::_1;
using moveit::planning_interface::MoveGroupInterface;

class Point_to_point : public rclcpp::Node
{
public:
  Point_to_point() : Node("vision_node_client_moveit")
  {
  
      subscription_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/ar_marker_1", 10 , std::bind(&Point_to_point::poseArrayCallback, this, _1));
  }

  void poseArrayCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    geometry_msgs::msg::Pose target_pose;
    target_pose = msg->pose;
    moveCollisionBoxToPose(target_pose);
    auto const logger = rclcpp::get_logger("Object moved");

  }

  void moveCollisionBoxToPose(geometry_msgs::msg::Pose target_pose)
  {
    RCLCPP_ERROR(this->get_logger(), "Moving box to new pose");
    // Create a MoveGroupInterface instance for the planning scene
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    // Create collision object for the robot to avoid
    moveit_msgs::msg::CollisionObject collision_object;
    collision_object.header.frame_id = "base_link";
    collision_object.id = "box1";
    shape_msgs::msg::SolidPrimitive primitive;

     // Define the size of the box in meters
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[primitive.BOX_X] = 0.3;
    primitive.dimensions[primitive.BOX_Y] = 0.1;
    primitive.dimensions[primitive.BOX_Z] = 0.1;

    // Set the pose of the box to the first received pose
    geometry_msgs::msg::Pose box_pose = target_pose;
    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(box_pose);
    collision_object.operation = collision_object.ADD;

      // Add the collision object to the scene
      planning_scene_interface.applyCollisionObject(collision_object);
    }
    private:
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr subscription_;
}; //Ends class 1

class Robot_motion :public rclcpp::Node
{
  public:
    Robot_motion(float sleep_time):Node("move_robot")
    { 
          subscription3_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
           "/topic", 10 , std::bind(&Robot_motion::poseArrayCallback3, this, _1));
    }
    
  void poseArrayCallback3(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
      // Process the received PoseArray messages
      geometry_msgs::msg::Pose target_pose;
      target_pose = msg->pose;

      // Move the collision box to the received pose
      //moveCollisionBoxToPose(target_pose);

      // Move the robot arm to the target poses
      auto const node = std::make_shared<rclcpp::Node>(
      "moveit_receive_move",
      rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
      );

      // Create a ROS logger
      auto const logger = rclcpp::get_logger("hello_moveit");
      // Define a set of 10 points to move the robot arm through

      geometry_msgs::msg::Pose homepose;
      homepose.position.x = -0.431;
      homepose.position.y = -0.382;
      homepose.position.z = 0.344;
      homepose.orientation.x =0.233;
      homepose.orientation.y =0.685;
      homepose.orientation.z =-0.657;
      homepose.orientation.w =-0.211;    
      target_poses.push_back(homepose);
      moveRobotArmThroughPoses(node,target_poses);  
    }
      
    void moveRobotArmThroughPoses(auto const &node,const std::vector<geometry_msgs::msg::Pose>& target_poses)
    {
      using moveit::planning_interface::MoveGroupInterface;
      auto move_group_interface = MoveGroupInterface(node, "ur_manipulator");

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
          RCLCPP_ERROR(this->get_logger(), "Failed to plan motion to target pose");
        }
      }
    }
  private:
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr subscription3_;
    std::vector<geometry_msgs::msg::Pose> target_poses;

};// End Class


int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto arm_node = std::make_shared<Point_to_point>();
  auto robot_node = std::make_shared<Robot_motion>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(arm_node);
  executor.add_node(robot_node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}