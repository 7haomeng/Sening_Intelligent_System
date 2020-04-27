// Revise from the Moveit tutorial here:
// https://github.com/ros-planning/moveit_tutorials/blob/kinetic-devel/doc/move_group_interface/src/move_group_interface_tutorial.cpp
// Editor: Sean Lu at October, 2017

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "moveit_tutorial");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(1);
  spinner.start();
  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // Declare some Moveit related objects and print information
  static const std::string PLANNING_GROUP = "manipulator";
  moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  const robot_state::JointModelGroup* joint_model_group =
      move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);
  namespace rvt = rviz_visual_tools;
  moveit_visual_tools::MoveItVisualTools visual_tools("base_link");
  visual_tools.deleteAllMarkers();
  visual_tools.loadRemoteControl();
  Eigen::Affine3d TEXT_POSE = Eigen::Affine3d::Identity();
  TEXT_POSE.translation().z() = 1.0;
  visual_tools.publishText(TEXT_POSE, "MoveGroupInterface Demo", rvt::WHITE, rvt::XLARGE);
  visual_tools.trigger();
  ROS_INFO("Reference frame: %s", move_group.getPlanningFrame().c_str());
  ROS_INFO("End effector link: %s", move_group.getEndEffectorLink().c_str());
  move_group.setPlannerId("RRTConnectkConfigDefault");
  move_group.setPoseReferenceFrame("base_link");
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");
  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // Demo 1. Plan to a given pose goal
  geometry_msgs::Pose target_pose1;
  target_pose1.orientation.w = 1.0;
  target_pose1.position.x = 0.3;
  target_pose1.position.z = 0.5;
  move_group.setPoseTarget(target_pose1);
  bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO("Visualizing plan 1 (pose goal) %s", success ? "": "FAILED");
  ROS_INFO("Visualizing plan 1 as trajectory line");
  visual_tools.publishAxisLabeled(target_pose1, "pose1");
  visual_tools.publishText(TEXT_POSE, "Pose Goal", rvt::WHITE, rvt::XLARGE);
  visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
  visual_tools.trigger();
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");
  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // Demo 2. Move to a given pose goal
  ROS_INFO("Visualizing move to pose");
  move_group.execute(my_plan);
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");
  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // Demo 3. Plan to a given joint space goal
  moveit::core::RobotStatePtr current_state = move_group.getCurrentState();
  std::vector<double> joint_group_positions;
  current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);
  joint_group_positions[0] = 0.0;  // radians
  joint_group_positions[1] = -M_PI/2;
  joint_group_positions[2] = 0.0;
  joint_group_positions[3] = 0.0;
  joint_group_positions[4] = M_PI/2;
  joint_group_positions[5] = 0.0;
  move_group.setJointValueTarget(joint_group_positions);
  success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO("Visualizing plan 2 (joint space goal) %s", success ? "": "FAILED");
  visual_tools.deleteAllMarkers();
  visual_tools.publishText(TEXT_POSE, "Joint Space Goal", rvt::WHITE, rvt::XLARGE);
  visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
  visual_tools.trigger();
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");
  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // Demo 4. Planning with path contraints
  moveit_msgs::OrientationConstraint ocm;
  ocm.link_name = "ee_link";
  ocm.header.frame_id = "base_link";
  ocm.orientation.w = 1.0;
  ocm.absolute_x_axis_tolerance = 0.1;
  ocm.absolute_y_axis_tolerance = 0.1;
  ocm.absolute_z_axis_tolerance = 0.1;
  ocm.weight = 1.0;
  moveit_msgs::Constraints test_constraints;
  test_constraints.orientation_constraints.push_back(ocm);
  move_group.setPathConstraints(test_constraints);
  geometry_msgs::Pose target_pose2;
  target_pose2.orientation.w = 1.0;
  target_pose2.position.x = 0.6;
  target_pose2.position.y = 0.1;
  move_group.setPoseTarget(target_pose2);
  move_group.setPlanningTime(10.0);
  success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO_NAMED("tutorial", "Visualizing plan 3 (constraints) %s", success ? "" : "FAILED");
  visual_tools.deleteAllMarkers();
  visual_tools.publishAxisLabeled(target_pose1, "start");
  visual_tools.publishAxisLabeled(target_pose2, "goal");
  visual_tools.publishText(TEXT_POSE, "Constrained Goal", rvt::WHITE, rvt::XLARGE);
  visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
  visual_tools.trigger();
  move_group.clearPathConstraints();
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");
  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // Demo 5. Cartesian path to traver through a square
  std::vector<geometry_msgs::Pose> waypoints;
  target_pose2 = target_pose1;
  waypoints.push_back(target_pose2);
  target_pose2.position.z += 0.2;
  waypoints.push_back(target_pose2);
  target_pose2.position.y -= 0.2;
  waypoints.push_back(target_pose2); 
  target_pose2.position.z -= 0.2;
  waypoints.push_back(target_pose2);
  target_pose2.position.y += 0.2;
  waypoints.push_back(target_pose2);
  move_group.setMaxVelocityScalingFactor(0.1);
  moveit_msgs::RobotTrajectory trajectory;
  const double jump_threshold = 0.0;
  const double eef_step = 0.01;
  double fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
  ROS_INFO("Visualizing plan 4 (Cartesian path) (%.2f%% acheived)", fraction * 100.0);
  visual_tools.deleteAllMarkers();
  visual_tools.publishText(TEXT_POSE, "Cartesian path", rvt::WHITE, rvt::XLARGE);
  visual_tools.publishPath(waypoints, rvt::LIME_GREEN, rvt::SMALL);
  for (std::size_t i = 0; i < waypoints.size(); ++i)
    visual_tools.publishAxisLabeled(waypoints[i], "pt" + std::to_string(i), rvt::SMALL);
  visual_tools.trigger();
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");
  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // Demo 6. Use RRTConnect to avoid the obstacle
  moveit_msgs::CollisionObject collision_object[2];
  collision_object[0].header.frame_id = move_group.getPlanningFrame();
  collision_object[1].header.frame_id = move_group.getPlanningFrame();
  collision_object[0].id = "obstacle";
  collision_object[1].id = "floor";
  shape_msgs::SolidPrimitive primitive;
  primitive.type = primitive.BOX;
  primitive.dimensions.resize(3);
  // define obstacle
  primitive.dimensions[0] = 0.4;
  primitive.dimensions[1] = 0.1;
  primitive.dimensions[2] = 0.4;
  geometry_msgs::Pose box_pose;
  box_pose.orientation.w = 1.0;
  box_pose.position.x = 0.4;
  box_pose.position.y = -0.2;
  box_pose.position.z = 0.5;
  collision_object[0].primitives.push_back(primitive);
  collision_object[0].primitive_poses.push_back(box_pose);
  collision_object[0].operation = collision_object[0].ADD;
  // define floor
  box_pose.position.x = box_pose.position.y = box_pose.position.z = 0;
  primitive.dimensions[0] = primitive.dimensions[1] = 1;
  primitive.dimensions[2] = 0.01;
  collision_object[1].primitives.push_back(primitive);
  collision_object[1].primitive_poses.push_back(box_pose);
  collision_object[1].operation = collision_object[1].ADD;
  std::vector<moveit_msgs::CollisionObject> collision_objects;
  collision_objects.push_back(collision_object[0]); collision_objects.push_back(collision_object[1]);
  ROS_INFO("Add objects into the world");
  planning_scene_interface.addCollisionObjects(collision_objects);
  visual_tools.publishText(TEXT_POSE, "Add objects", rvt::WHITE, rvt::XLARGE);
  visual_tools.trigger();
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to once the collision object appears in RViz");
  geometry_msgs::Pose another_pose;
  another_pose.orientation.w = 1.0;
  another_pose.position.x = 0.3;
  another_pose.position.y = -0.3;
  another_pose.position.z = 0.5;
  move_group.setPoseTarget(another_pose);
  success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO("Visualizing plan 5 (obstacle avoidance with RRTConnection %s)", success ? "": "FAILED");
  visual_tools.deleteAllMarkers();
  visual_tools.publishText(TEXT_POSE, "Obstacle Goal", rvt::WHITE, rvt::XLARGE);
  visual_tools.publishText(another_pose, "Goal", rvt::WHITE, rvt::XLARGE);
  visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
  visual_tools.trigger();
  while(1) {
    // If you are not satisfy with the planning result, you can replan it
    char c ;
    std::cout << "Press 'r' to replan, 'c' to continue..." << std::endl;
    std::cin >> c;
    if(c == 'r') {
      move_group.plan(my_plan);
      visual_tools.deleteAllMarkers();
      visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
      visual_tools.trigger();
    }
    if(c == 'c') break;
  }
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to remove the objects");
  std::vector<std::string> obj;
  obj.push_back(collision_object[0].id);
  obj.push_back(collision_object[1].id);
  planning_scene_interface.removeCollisionObjects(obj);
  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // Demo 7. Attach an cube, move it to a pose and then detach it
  ros::Duration(1.0).sleep();
  collision_object[0].id = "object_to_pick";
  collision_object[0].header.frame_id = move_group.getEndEffectorLink();
  primitive.dimensions[0] = 0.1;
  primitive.dimensions[1] = 0.1;
  primitive.dimensions[2] = 0.1;
  box_pose.position.x = 0.05;
  box_pose.position.y = 
  box_pose.position.z = 0;
  box_pose.orientation.w = 1.0;
  collision_object[0].primitives.clear(); collision_object[0].primitive_poses.clear();
  collision_object[0].primitives.push_back(primitive);
  collision_object[0].primitive_poses.push_back(box_pose);
  collision_object[0].operation = collision_object[0].ADD;
  collision_objects.clear(); collision_objects.push_back(collision_object[0]);
  planning_scene_interface.addCollisionObjects(collision_objects);
  ros::Duration(2.0).sleep();
  move_group.attachObject(collision_object[0].id);
  visual_tools.deleteAllMarkers();
  visual_tools.publishText(TEXT_POSE, "Attach cube", rvt::WHITE, rvt::XLARGE);
  visual_tools.trigger();
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to once the collision object attaches to the robot");
  move_group.setPoseTarget(another_pose);
  success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO("Visualizing plan 6 (pick and place %s)", success ? "": "FAILED");
  visual_tools.deleteAllMarkers();
  visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
  visual_tools.trigger();
  move_group.move();
  visual_tools.deleteAllMarkers();
  visual_tools.publishText(TEXT_POSE, "Place the cube", rvt::WHITE, rvt::XLARGE);
  visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
  visual_tools.trigger();
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to detech the objects");
  move_group.detachObject(collision_object[0].id);
  visual_tools.trigger();
  obj.clear();
  obj.push_back(collision_object[0].id);
  planning_scene_interface.removeCollisionObjects(obj);
  visual_tools.trigger();
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to once the collision object disapears");
  ros::shutdown();
  return 0;
}

