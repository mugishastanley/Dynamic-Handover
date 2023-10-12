// This node moves the robot arm to a target pose receved from the camera.
//In this case, the target pose may be the localized object
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
  Point_to_point() : Node("Pointtopoint")
  {
    subscription_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
      "/aruco_poses", 100, std::bind(&Point_to_point::poseArrayCallback, this, _1));
  }
 

private:
  void poseArrayCallback(const geometry_msgs::msg::PoseArray::SharedPtr msg)
  {
    
    // Process the received PoseArray messages
    geometry_msgs::msg::Pose target_pose;
    for (const auto& pose : msg->poses)
    {
      // Access pose information (e.g., position and orientation)
      double x = pose.position.x;
      double y = pose.position.y;
      double z = pose.position.z;

      double qx = pose.orientation.x;
      double qy = pose.orientation.y;
      double qz = pose.orientation.z;
      double qw = pose.orientation.w;

      //1. Transform the pose information into robot pose
      //1.1 Get the robot transform from the robot state publisher
      //1.2 multiply the robot tranfrom with the robot to camera transform
      //1.3 multiply the camera transform with the aruco pose to get the aruco to robot base transform
      //1.4 send the aruco to base transform to move it.
      //Test your work.
      //Take a coffee and go home. 

      // Print the received pose
      // auto move_group_interface = MoveGroupInterface(node, "ur_manipulator");
      RCLCPP_INFO(this->get_logger(), "Received pose: (%f, %f, %f), (%f, %f, %f, %f)",
                  x, y, z, qx, qy, qz, qw);

      target_poses.push_back(pose);
    }

    auto const node = std::make_shared<rclcpp::Node>(
    "moveit_receive_move_robot",
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

void moveRobotArmThroughPoses(auto const &node, const std::vector<geometry_msgs::msg::Pose>& target_poses)
{
    // Create a MoveGroupInterface instance for the robot arm
        // Create the MoveIt MoveGroup Interface
    using moveit::planning_interface::MoveGroupInterface;
    auto move_group_interface = MoveGroupInterface(node, "ur_manipulator");
    //moveit::planning_interface::MoveGroupInterface move_group("ur_manipulator");

    // Set the planning time and maximum velocity scaling factor
    move_group_interface.setPlanningTime(10.0);
    move_group_interface.setMaxVelocityScalingFactor(0.5);

    // Create collision object for the robot to avoid
  auto const collision_object = [frame_id =
                                  move_group_interface.getPlanningFrame()] {
    moveit_msgs::msg::CollisionObject collision_object;
    collision_object.header.frame_id = frame_id;
    collision_object.id = "box1";
    shape_msgs::msg::SolidPrimitive primitive;

    // Define the size of the box in meters
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[primitive.BOX_X] = 1.0;
    primitive.dimensions[primitive.BOX_Y] = 1.0;
    primitive.dimensions[primitive.BOX_Z] = 1.0;

    // Define the pose of the box (relative to the frame_id)
    geometry_msgs::msg::Pose box_pose;
    box_pose.orientation.w = 1.0;
    box_pose.position.x = 0.0;
    box_pose.position.y = 0.0;
    box_pose.position.z = -0.5;

    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(box_pose);
    collision_object.operation = collision_object.ADD;

    return collision_object;
  }();

  // Add the collision object to the scene
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  planning_scene_interface.applyCollisionObject(collision_object);

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
        RCLCPP_ERROR(rclcpp::get_logger("moveit_example"), "Failed to plan motion to target pose");
      }
    }
}

rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr subscription_;
std::vector<geometry_msgs::msg::Pose> target_poses;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Point_to_point>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}