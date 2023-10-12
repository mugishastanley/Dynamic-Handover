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


int main(int argc, char** argv)
{

    // Initialize ROS and create the Node
    rclcpp::init(argc, argv);
    auto const node = std::make_shared<rclcpp::Node>(
    "moveit_eval",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
    );

    // Create a ROS logger
    auto const logger = rclcpp::get_logger("hello_moveit");

  // Define a set of 10 points to move the robot arm through
  std::vector<geometry_msgs::msg::Pose> target_poses;


  // Example target poses
  geometry_msgs::msg::Pose pose1;
  pose1.position.x = -0.431;
  pose1.position.y = -0.382;
  pose1.position.z = 0.344;
  pose1.orientation.x =0.233;
  pose1.orientation.y =0.685;
  pose1.orientation.z =-0.657;
  pose1.orientation.w =-0.211;

  // Set the orientation of the pose if needed
  // pose1.orientation = ...

  geometry_msgs::msg::Pose pose2;
  pose2.position.x = -0.260;
  pose2.position.y = -0.349;
  pose2.position.z = 0.822;
  pose2.orientation.x = -0.7148586;  
  pose2.orientation.y = 0.6319762; 
  pose2.orientation.z = 0.0167513;
  pose2.orientation.w = 0.2988356 ;
  // Set the orientation of the pose if needed
  // pose2.orientation = ...

  geometry_msgs::msg::Pose pose3;
  pose3.position.x = 0.174;
  pose3.position.y = 0.645;
  pose3.position.z = 0.204;
  pose3.orientation.x = 0.171991; 
  pose3.orientation.y = -0.7780546; 
  pose3.orientation.z = 0.591146;
  pose3.orientation.w = -0.1248857; 

  // Set the orientation of the pose if needed
  // pose2.orientation = ...
  geometry_msgs::msg::Pose pose4;
  pose4.position.x = -0.337;
  pose4.position.y = -0.548;
  pose4.position.z = 0.307;
  pose4.orientation.x = 0.139022;
  pose4.orientation.y = 0.732494; 
  pose4.orientation.z = -0.6542211; 
  pose4.orientation.w = -0.1269649;

  // Set the orientation of the pose if needed
  // pose2.orientation = ...
  geometry_msgs::msg::Pose pose5;
  pose5.position.x = -405.94;
  pose5.position.y = 0.442;
  pose5.position.z = 0.381;
  pose5.orientation.x = 0.6367472; 
  pose5.orientation.y = 0.318978; 
  pose5.orientation.z = -0.3118973; 
  pose5.orientation.w = -0.6289086;

  // Set the orientation of the pose if needed
  // pose2.orientation = ...

  geometry_msgs::msg::Pose pose6;
  pose6.position.x = -0.236;
  pose6.position.y = -0.074;
  pose6.position.z = 0.486;
  pose6.orientation.x = -0.5633848; 
  pose6.orientation.y = 0.0506488;
  pose6.orientation.z = -0.8186376; 
  pose6.orientation.w = -0.0993212;
  // Set the orientation of the pose if needed
  // pose2.orientation = ...

  geometry_msgs::msg::Pose pose7;
  pose7.position.x = -0.369;
  pose7.position.y = -0.602;
  pose7.position.z = 0.251;
  pose7.orientation.x = -0.8146877; 
  pose7.orientation.y = 0.3905693;  
  pose7.orientation.z = -0.2752025; 
  pose7.orientation.w = 0.3286382;
  // Set the orientation of the pose if needed
  // pose2.orientation = ...


  geometry_msgs::msg::Pose pose8;
  pose8.position.x = 0.355;
  pose8.position.y = -0.607;
  pose8.position.z = 0.242;
  pose8.orientation.x = -0.7646381; 
  pose8.orientation.y = -0.4032667; 
  pose8.orientation.z = 0.1583157;  
  pose8.orientation.w = 0.4771171;
  // Set the orientation of the pose if needed
  // pose2.orientation = ...

  geometry_msgs::msg::Pose pose9;
  pose8.position.x = 0.232;
  pose8.position.y = -0.171;
  pose8.position.z = 0.654;
  pose8.orientation.x =  0.180686; 
  pose8.orientation.y = -0.7900937; 
  pose8.orientation.z = -0.5794892; 
  pose8.orientation.w = 0.0854217;


  target_poses.push_back(pose1);
  target_poses.push_back(pose2);
  target_poses.push_back(pose3);
  target_poses.push_back(pose4);
  target_poses.push_back(pose5);
  target_poses.push_back(pose6);
  target_poses.push_back(pose7);
  target_poses.push_back(pose8);
  target_poses.push_back(pose9);

  // TODO: Add more target poses as needed
  moveRobotArmThroughPoses(node,target_poses);
  rclcpp::shutdown();

  return 0;
}
