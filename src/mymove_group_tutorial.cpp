#include <pluginlib/class_loader.h>
#include <ros/ros.h>
#include <mytutorialPkg/HandPose.h>

// MoveIt!
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/PlanningScene.h>
#include <moveit_msgs/RobotTrajectory.h>
#include <moveit_msgs/ExecuteTrajectoryAction.h>
#include <moveit_msgs/ExecuteTrajectoryGoal.h>
#include <boost/scoped_ptr.hpp>

#include <ctime>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

geometry_msgs::Pose poseTip2;//Right Finger tip
geometry_msgs::Pose poseLink6;//wrist
geometry_msgs::Pose poseTip1;//Left Finger tip
bool messageReceived = false;

// A tolerance of 0.01 m is specified in position
// and 0.01 radians in orientation
std::vector<double> tolerance_pose(3, 0.01);
std::vector<double> tolerance_angle(3, 0.01);//0.01 is normal


void updatePoseValues(const mytutorialPkg::HandPose::ConstPtr& msg){
  ROS_INFO_THROTTLE(1,"I received a message.. now processing...");
  poseTip2 = msg->poseTip2;
  poseLink6 = msg->poseLink6;
  poseTip1 = msg->poseTip1;
  messageReceived = true;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "move_group_tutorial");
  ros::AsyncSpinner spinner(1);
  spinner.start();
  ros::NodeHandle node_handle("~");

  static const std::string PLANNING_GROUP_ARM = "chainArm";
  static const std::string PLANNING_GROUP_ARMLEFT = "chainArmLeft";

  moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP_ARM);
  moveit::planning_interface::MoveGroupInterface move_group2(PLANNING_GROUP_ARMLEFT);

  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  const robot_state::JointModelGroup *joint_model_group = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP_ARM);

  const robot_state::JointModelGroup *joint_model_group2 = move_group2.getCurrentState()->getJointModelGroup(PLANNING_GROUP_ARMLEFT);


  // Visualization
  // ^^^^^^^^^^^^^
  //
  // The package MoveItVisualTools provides many capabilties for visualizing objects, robots,
  // and trajectories in Rviz as well as debugging tools such as step-by-step introspection of a script
  namespace rvt = rviz_visual_tools;
  moveit_visual_tools::MoveItVisualTools visual_tools("m1n6a200_link_base");
  visual_tools.deleteAllMarkers();

  // Remote control is an introspection tool that allows users to step through a high level script
  // via buttons and keyboard shortcuts in Rviz
  visual_tools.loadRemoteControl();

  // Rviz provides many types of markers, in this demo we will use text, cylinders, and spheres
  Eigen::Affine3d text_pose = Eigen::Affine3d::Identity();
  text_pose.translation().z() = 1.75; // above head of PR2
  visual_tools.publishText(text_pose, "MoveGroupInterface Demo", rvt::WHITE, rvt::XLARGE);

  // Batch publishing is used to reduce the number of messages being sent to Rviz for large visualizations
  visual_tools.trigger();





  //////////////////////////////////////////////////////////////////////////////////////////////////////
  //---------Loop of the MAIN PROGRAM------------------------------
  //////////////////////////////////////////////////////////////////////////////////////////////////////


  //////////////////////////////////////////////////////////////////////////////////////////////////////
  //get input Pose using subscriber--which calls updatesPoseValues
  //Only continue when a new pose is received
  //////////////////////////////////////////////////////////////////////////////////////////////////////
  geometry_msgs::Pose sensedPoseTip2;//Right Finger tip---Demo data
  //sensedPoseTip2.header.frame_id = "m1n6a200_link_base";
  sensedPoseTip2.position.x = 0.305735;
  sensedPoseTip2.position.y = 0.196963;
  sensedPoseTip2.position.z = 0.663391;
  sensedPoseTip2.orientation.x = 0.623481;
  sensedPoseTip2.orientation.y = 0.3335;
  sensedPoseTip2.orientation.z = 0.623522;
  sensedPoseTip2.orientation.w = 0.333571;

  geometry_msgs::Pose sensedPoseLink6;//wrist --- Demo Data
  //sensedPoseLink6.header.frame_id = "m1n6a200_link_base";
  sensedPoseLink6.position.x = 0.308219;
  sensedPoseLink6.position.y = 0.0494515;
  sensedPoseLink6.position.z = 0.727438;
  sensedPoseLink6.orientation.x = 0.615034;
  sensedPoseLink6.orientation.y = 0.0;
  sensedPoseLink6.orientation.z = 0.0;
  sensedPoseLink6.orientation.w = 0.788501;

  //////////////////////////////////////////////////////////////////////////////////////////////////////
  //PLANNING
  //////////////////////////////////////////////////////////////////////////////////////////////////////
    move_group.setPoseTarget(sensedPoseTip2);
    move_group.setPoseTarget(sensedPoseLink6);

    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool success = move_group.plan(my_plan);
    ROS_INFO("It took [%f] HEHEHHEHEHEHEHEHEHEHHHEHEHEHEH",my_plan.trajectory_.joint_trajectory.points[1].time_from_start);
    move_group.move();
    // Visualizing plans
    // ^^^^^^^^^^^^^^^^^
    // We can also visualize the plan as a line with markers in Rviz.
    ROS_INFO_NAMED("tutorial", "Visualizing plan 1 as trajectory line");
    visual_tools.publishAxisLabeled(poseTip2, "pose1");
    visual_tools.publishText(text_pose, "Pose Goal", rvt::WHITE, rvt::XLARGE);
    visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
    visual_tools.trigger();
    visual_tools.prompt("next step");





    //////////////////////////////////////////////////////////////////////////////////////////////////////
    //Reset
    //////////////////////////////////////////////////////////////////////////////////////////////////////


  ros::spinOnce();
  //Maybe add wait here...

  //Loop back to start of main Program

  // END_TUTORIAL
  ros::shutdown();

  return 0;
}
