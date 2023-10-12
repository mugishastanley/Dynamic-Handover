// This node moves the robot to the start position.
// This node moves the robot arm to a target pose receved from the camera.

// This node moves the robot arm to a target pose predefined by the human.
// AN obstacle is deined in the scene as the human arm and evaluation for safety has to be made from this.


#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>


void moveRobotArmThroughPoses(auto const &node, const std::vector<geometry_msgs::msg::Pose>& target_poses)
{
  // Create a MoveGroupInterface instance for the robot arm
      // Create the MoveIt MoveGroup Interface
    using moveit::planning_interface::MoveGroupInterface;
    auto move_group_interface = MoveGroupInterface(node, "ur_manipulator");

  // Set the planning time and maximum velocity scaling factor
  move_group_interface.setPlanningTime(10.0);
  move_group_interface.setMaxVelocityScalingFactor(0.5);

  // Create collision object for the robot to act as a table
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


int main(int argc, char** argv)
{

    // Initialize ROS and create the Node
    rclcpp::init(argc, argv);
    auto const node = std::make_shared<rclcpp::Node>(
    "reset_node",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
    );

    // Create a ROS logger
    auto const logger = rclcpp::get_logger("Moving robot to start_pose");

  // Define a set of 10 points to move the robot arm through
  std::vector<geometry_msgs::msg::Pose> target_poses;


  // Example target poses
  geometry_msgs::msg::Pose init_pose;
  init_pose.position.x = -0.431;
  init_pose.position.y = -0.382;
  init_pose.position.z = 0.344;
  init_pose.orientation.x =0.233;
  init_pose.orientation.y =0.685;
  init_pose.orientation.z =-0.657;
  init_pose.orientation.w =-0.211;

  target_poses.push_back(init_pose);
  // TODO: Add more target poses as needed
  moveRobotArmThroughPoses(node,target_poses);
  rclcpp::shutdown();

  return 0;
}
