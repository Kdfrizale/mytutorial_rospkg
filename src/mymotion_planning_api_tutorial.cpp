#include <pluginlib/class_loader.h>
#include <ros/ros.h>

// MoveIt!
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/PlanningScene.h>

#include <boost/scoped_ptr.hpp>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "move_group_tutorial");
  ros::AsyncSpinner spinner(1);
  spinner.start();
  ros::NodeHandle node_handle("~");

  // BEGIN_TUTORIAL
  // Start
  // ^^^^^
  // Setting up to start using a planner is pretty easy. Planners are
  // setup as plugins in MoveIt! and you can use the ROS pluginlib
  // interface to load any planner that you want to use. Before we
  // can load the planner, we need two objects, a RobotModel
  // and a PlanningScene.
  // We will start by instantiating a
  // `RobotModelLoader`_
  // object, which will look up
  // the robot description on the ROS parameter server and construct a
  // :moveit_core:`RobotModel` for us to use.
  //
  // .. _RobotModelLoader: http://docs.ros.org/indigo/api/moveit_ros_planning/html/classrobot__model__loader_1_1RobotModelLoader.html
  robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
  robot_model::RobotModelPtr robot_model = robot_model_loader.getModel();

  // Using the :moveit_core:`RobotModel`, we can construct a
  // :planning_scene:`PlanningScene` that maintains the state of
  // the world (including the robot).
  planning_scene::PlanningScenePtr planning_scene(new planning_scene::PlanningScene(robot_model));


  // We will now construct a loader to load a planner, by name.
  // Note that we are using the ROS pluginlib library here.
  boost::scoped_ptr<pluginlib::ClassLoader<planning_interface::PlannerManager>> planner_plugin_loader;
  planning_interface::PlannerManagerPtr planner_instance;
  std::string planner_plugin_name;

  // We will get the name of planning plugin we want to load
  // from the ROS param server, and then load the planner
  // making sure to catch all exceptions.
  if (!node_handle.getParam("planning_plugin", planner_plugin_name))
    ROS_FATAL_STREAM("Could not find planner plugin name");
  try
  {
    planner_plugin_loader.reset(new pluginlib::ClassLoader<planning_interface::PlannerManager>(
        "moveit_core", "planning_interface::PlannerManager"));
  }
  catch (pluginlib::PluginlibException& ex)
  {
    ROS_FATAL_STREAM("Exception while creating planning plugin loader " << ex.what());
  }
  try
  {
    planner_instance.reset(planner_plugin_loader->createUnmanagedInstance(planner_plugin_name));
    if (!planner_instance->initialize(robot_model, node_handle.getNamespace()))
      ROS_FATAL_STREAM("Could not initialize planner instance");
    ROS_INFO_STREAM("Using planning interface '" << planner_instance->getDescription() << "'");
  }
  catch (pluginlib::PluginlibException& ex)
  {
    const std::vector<std::string>& classes = planner_plugin_loader->getDeclaredClasses();
    std::stringstream ss;
    for (std::size_t i = 0; i < classes.size(); ++i)
      ss << classes[i] << " ";
    ROS_ERROR_STREAM("Exception while loading planner '" << planner_plugin_name << "': " << ex.what() << std::endl
                                                         << "Available plugins: " << ss.str());
  }

  /* Sleep a little to allow time to startup rviz, etc. */
  ros::WallDuration sleep_time(15.0);
  sleep_time.sleep();



  // Pose Goal
  // ^^^^^^^^^
  // We will now create a motion plan request for the right arm of the PR2
  // specifying the desired pose of the end-effector as input.
  planning_interface::MotionPlanRequest req;
  planning_interface::MotionPlanResponse res;

  geometry_msgs::PoseStamped pose;
  pose.header.frame_id = "m1n6a200_link_base";
  pose.pose.position.x = 0.305735; //pose is finger tip2
  pose.pose.position.y = 0.196963;
  pose.pose.position.z = 0.663391;
  pose.pose.orientation.x = 0.623481;
  pose.pose.orientation.y = 0.3335;
  pose.pose.orientation.z = 0.623522;
  pose.pose.orientation.w = 0.333571;

  geometry_msgs::PoseStamped poseLink6;
  poseLink6.header.frame_id = "m1n6a200_link_base";
  poseLink6.pose.position.x = 0.308219;
  poseLink6.pose.position.y = 0.0494515;
  poseLink6.pose.position.z = 0.727438;
  poseLink6.pose.orientation.x = 0.615034;
  poseLink6.pose.orientation.y = 0.0;
  poseLink6.pose.orientation.z = 0.0;
  poseLink6.pose.orientation.w = 0.788501;

  // A tolerance of 0.01 m is specified in position
  // and 0.01 radians in orientation
  std::vector<double> tolerance_pose(3, 0.01);
  std::vector<double> tolerance_angle(3, 0.01);

  // We will create the request as a constraint using a helper function available
  // from the
  // `kinematic_constraints`_
  // package.
  //
  // .. _kinematic_constraints: http://docs.ros.org/indigo/api/moveit_core/html/namespacekinematic__constraints.html#a88becba14be9ced36fefc7980271e132
  req.group_name = "chainArm";
  moveit_msgs::Constraints pose_goal =
      kinematic_constraints::constructGoalConstraints("m1n6a200_link_finger_tip_2", pose, tolerance_pose, tolerance_angle);
  req.goal_constraints.push_back(pose_goal);

  moveit_msgs::Constraints pose_goal2 =
      kinematic_constraints::constructGoalConstraints("m1n6a200_link_6", poseLink6, tolerance_pose, tolerance_angle);
  req.goal_constraints.push_back(pose_goal2);

  planning_interface::PlanningContextPtr context =
      planner_instance->getPlanningContext(planning_scene, req, res.error_code_);
  context->solve(res);
  if (res.error_code_.val != res.error_code_.SUCCESS)
  {
    ROS_ERROR("Could not compute plan successfully");
    return 0;
  }
  // Visualize the result
  // ^^^^^^^^^^^^^^^^^^^^
  ros::Publisher display_publisher =
      node_handle.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
  moveit_msgs::DisplayTrajectory display_trajectory;

  /* Visualize the trajectory */
  ROS_INFO("Visualizing the trajectory");
  moveit_msgs::MotionPlanResponse response;
  res.getMessage(response);

  display_trajectory.trajectory_start = response.trajectory_start;
  display_trajectory.trajectory.push_back(response.trajectory);
  display_publisher.publish(display_trajectory);

  ros::WallDuration waitForPlan_time(3.0);
  waitForPlan_time.sleep();

  //-------------Robot has planned to the first motion------------------------------------
  robot_state::RobotState& robot_state = planning_scene->getCurrentStateNonConst();
  planning_scene->setCurrentState(response.trajectory_start);
  const robot_state::JointModelGroup* joint_model_group = robot_state.getJointModelGroup("chainArm");
  robot_state.setJointGroupPositions(joint_model_group, response.trajectory.joint_trajectory.points.back().positions);

  //MUST SET ROBOT TO NEW THE LATEST STATE, EXMAPLE IS --
  /* First, set the state in the planning scene to the final state of the last plan */
  //robot_state.setJointGroupPositions(joint_model_group, response.trajectory.joint_trajectory.points.back().positions);
  /* Now, let's try to move to this new pose goal*/
  //req.goal_constraints.clear();
  //req.goal_constraints.push_back(pose_goal_2);
  planning_interface::MotionPlanRequest req2;
  planning_interface::MotionPlanResponse res2;
  geometry_msgs::PoseStamped pose2;
  pose2.header.frame_id = "m1n6a200_link_base";

  pose2.pose.position.x = 0.30573;//Left Finger tip
  pose2.pose.position.y = 0.211976;
  pose2.pose.position.z = 0.723024;
  pose2.pose.orientation.x = 0.648451;
  pose2.pose.orientation.y = 0.282066;
  pose2.pose.orientation.z = -0.648369;
  pose2.pose.orientation.w = -0.282078;

  req2.group_name = "chainArmLeft";

  moveit_msgs::Constraints pose_goal3 =
     kinematic_constraints::constructGoalConstraints("m1n6a200_link_finger_tip_1", pose2, tolerance_pose, tolerance_angle);
  req2.goal_constraints.push_back(pose_goal3);

  req2.goal_constraints.push_back(pose_goal2);
  ROS_INFO("I Made it here!!");
  // We now construct a planning context that encapsulate the scene,
  // the request and the response. We call the planner using this
  // planning context
  planning_interface::PlanningContextPtr context2 =
      planner_instance->getPlanningContext(planning_scene, req2, res2.error_code_);
  context2->solve(res2);
  if (res2.error_code_.val != res2.error_code_.SUCCESS)
  {
    ROS_ERROR("Could not compute plan successfully");
    return 0;
  }

  // Visualize the result
  // ^^^^^^^^^^^^^^^^^^^^
  /*ros::Publisher display_publisher =
      node_handle.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
  moveit_msgs::DisplayTrajectory display_trajectory;

  /* Visualize the trajectory */
  ROS_INFO("Visualizing the trajectory");
  //moveit_msgs::MotionPlanResponse response;
  res2.getMessage(response);

  display_trajectory.trajectory_start = response.trajectory_start;
  display_trajectory.trajectory.push_back(response.trajectory);
  display_publisher.publish(display_trajectory);

  waitForPlan_time.sleep();

  // END_TUTORIAL
  sleep_time.sleep();
  ROS_INFO("Done");
  planner_instance.reset();

  return 0;
}
