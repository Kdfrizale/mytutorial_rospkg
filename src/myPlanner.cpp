#include <pluginlib/class_loader.h>
#include <ros/ros.h>
#include <mytutorialPkg/HandStampedPose.h>

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

geometry_msgs::PoseStamped poseTip2;//Right Finger tip
geometry_msgs::PoseStamped poseLink6;//wrist
geometry_msgs::PoseStamped poseTip1;//Left Finger tip
bool messageReceived = false;

// A tolerance of 0.01 m is specified in position
// and 0.01 radians in orientation
std::vector<double> tolerance_pose(3, 0.01);
std::vector<double> tolerance_angle(3, 0.01);//0.01 is normal


void updatePoseValues(const mytutorialPkg::HandStampedPose::ConstPtr& msg){
  ROS_INFO("I received a message.. now processing...");
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
  ros::WallDuration waitForPlan_time(0.5);
  sleep_time.sleep();

  //////////////////////////////////////////////////////////////////////////////////////////////////////
  //---------Loop of the MAIN PROGRAM------------------------------
  //////////////////////////////////////////////////////////////////////////////////////////////////////
  while (ros::ok()){
  //////////////////////////////////////////////////////////////////////////////////////////////////////
  //get input Pose using subscriber--which calls updatesPoseValues
  //Only continue when a new pose is received
  //////////////////////////////////////////////////////////////////////////////////////////////////////

  ros::Subscriber sub = node_handle.subscribe("/handPoseTopic",10,updatePoseValues);
  if(messageReceived){
    ROS_INFO("Message was Received!");
  }
  else{
    ROS_INFO("Message not Received...");
  }
  //////////////////////////////////////////////////////////////////////////////////////////////////////
  //Then run the standard planning technique
  //////////////////////////////////////////////////////////////////////////////////////////////////////
  if(messageReceived){
    planning_interface::MotionPlanRequest req;
    planning_interface::MotionPlanResponse res;
    planning_interface::MotionPlanRequest req2;
    planning_interface::MotionPlanResponse res2;

    /*geometry_msgs::PoseStamped poseTip2;//Right Finger tip---Demo data
    poseTip2.header.frame_id = "m1n6a200_link_base";
    poseTip2.pose.position.x = 0.305735;
    poseTip2.pose.position.y = 0.196963;
    poseTip2.pose.position.z = 0.663391;
    poseTip2.pose.orientation.x = 0.623481;
    poseTip2.pose.orientation.y = 0.3335;
    poseTip2.pose.orientation.z = 0.623522;
    poseTip2.pose.orientation.w = 0.333571;

    geometry_msgs::PoseStamped poseLink6;//wrist --- Demo Data
    poseLink6.header.frame_id = "m1n6a200_link_base";
    poseLink6.pose.position.x = 0.308219;
    poseLink6.pose.position.y = 0.0494515;
    poseLink6.pose.position.z = 0.727438;
    poseLink6.pose.orientation.x = 0.615034;
    poseLink6.pose.orientation.y = 0.0;
    poseLink6.pose.orientation.z = 0.0;
    poseLink6.pose.orientation.w = 0.788501;

    geometry_msgs::PoseStamped poseTip1;//Left Finger tip---Demo data
    poseTip1.header.frame_id = "m1n6a200_link_base";
    poseTip1.pose.position.x = 0.30573;
    poseTip1.pose.position.y = 0.211976;
    poseTip1.pose.position.z = 0.723024;
    poseTip1.pose.orientation.x = 0.648451;
    poseTip1.pose.orientation.y = 0.282066;
    poseTip1.pose.orientation.z = -0.648369;
    poseTip1.pose.orientation.w = -0.282078;

    // A tolerance of 0.01 m is specified in position
    // and 0.01 radians in orientation
    std::vector<double> tolerance_pose(3, 0.01);
    std::vector<double> tolerance_angle(3, 0.01);
  */
    req.group_name = "chainArm";
    moveit_msgs::Constraints pose_goal_tip_2 = kinematic_constraints::constructGoalConstraints("m1n6a200_link_finger_tip_2", poseTip2, tolerance_pose, tolerance_angle);
    req.goal_constraints.push_back(pose_goal_tip_2);

    moveit_msgs::Constraints pose_goal_link6 = kinematic_constraints::constructGoalConstraints("m1n6a200_link_6", poseLink6, tolerance_pose, tolerance_angle);
    req.goal_constraints.push_back(pose_goal_link6);

    planning_interface::PlanningContextPtr context = planner_instance->getPlanningContext(planning_scene, req, res.error_code_);

    context->solve(res);
    if (res.error_code_.val != res.error_code_.SUCCESS){
      ROS_ERROR("Could not compute plan successfully");
      return 0;
    }

    //-------------Robot has planned to the first motion------------------------------------
    moveit_msgs::MotionPlanResponse midResponse;
    res.getMessage(midResponse);//Get the first motion plan

    robot_state::RobotState& robot_state = planning_scene->getCurrentStateNonConst();
    planning_scene->setCurrentState(midResponse.trajectory_start);
    const robot_state::JointModelGroup* joint_model_group = robot_state.getJointModelGroup("chainArm");
    robot_state.setJointGroupPositions(joint_model_group, midResponse.trajectory.joint_trajectory.points.back().positions);

    req2.group_name = "chainArmLeft";
    moveit_msgs::Constraints pose_goal_tip_1 = kinematic_constraints::constructGoalConstraints("m1n6a200_link_finger_tip_1", poseTip1, tolerance_pose, tolerance_angle);
    req2.goal_constraints.push_back(pose_goal_tip_1);
    req2.goal_constraints.push_back(pose_goal_link6);

    planning_interface::PlanningContextPtr context2 = planner_instance->getPlanningContext(planning_scene, req2, res2.error_code_);
    context2->solve(res2);
    if (res2.error_code_.val != res2.error_code_.SUCCESS){
      ROS_ERROR("Could not compute plan successfully");
      return 0;
    }

    moveit_msgs::MotionPlanResponse endResponse;
    res2.getMessage(endResponse);

    robot_state = planning_scene->getCurrentStateNonConst();
    planning_scene->setCurrentState(endResponse.trajectory_start);
    joint_model_group = robot_state.getJointModelGroup("chainArmLeft");
    robot_state.setJointGroupPositions(joint_model_group, endResponse.trajectory.joint_trajectory.points.back().positions);

    //////////////////////////////////////////////////////////////////////////////////////////////////////
    //show the result in Rviz
    //////////////////////////////////////////////////////////////////////////////////////////////////////
    ros::Publisher display_publisher = node_handle.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
    moveit_msgs::DisplayTrajectory display_trajectory;

    ROS_INFO("Visualizing the trajectory");
    moveit_msgs::MotionPlanResponse response;

    res.getMessage(response);//Get the first motion plan
    display_trajectory.trajectory_start = response.trajectory_start;
    display_trajectory.trajectory.push_back(response.trajectory);

    res2.getMessage(response);//Get the second motion plan
    display_trajectory.trajectory_start = response.trajectory_start;
    display_trajectory.trajectory.push_back(response.trajectory);
    display_publisher.publish(display_trajectory);
    waitForPlan_time.sleep();

    //////////////////////////////////////////////////////////////////////////////////////////////////////
    //Move the physical robot to the planned coordinates
    //////////////////////////////////////////////////////////////////////////////////////////////////////
    //IN here I need to publish a moveit_msgs/ExecuteTrajectoryActionGoal to the topic /execute_trajectory/goal
    //there are four other topics under exectute_trajectory that may be useful

    //TODO this I need to set up an action clint that can push a goal to the action server "exectute_trajectory"
    actionlib::SimpleActionClient<moveit_msgs::ExecuteTrajectoryAction> ac("execute_trajectory",false);
    ROS_INFO("Waiting for action server to start.");
    ac.waitForServer();

    //moveit_msgs::ExecuteTrajectoryAction goal;
    moveit_msgs::ExecuteTrajectoryGoal goal;
    goal.trajectory = midResponse.trajectory;
    //goal.state = "hello";
    ac.sendGoal(goal);

    bool finished_before_timeout = ac.waitForResult(ros::Duration(30.0));
    if (finished_before_timeout){
      actionlib::SimpleClientGoalState state = ac.getState();
      ROS_INFO("Action finished: %s",state.toString().c_str());
    }
    else{
      ROS_INFO("Action did not finish before the time out.");
    }

    moveit_msgs::ExecuteTrajectoryGoal goal2;
    goal2.trajectory = endResponse.trajectory;
    //goal.state = "hello";
    ac.sendGoal(goal2);

    bool finished_before_timeout2 = ac.waitForResult(ros::Duration(30.0));
    if (finished_before_timeout2){
      actionlib::SimpleClientGoalState state = ac.getState();
      ROS_INFO("Action finished: %s",state.toString().c_str());
    }
    else{
      ROS_INFO("Action did not finish before the time out.");
    }


    //////////////////////////////////////////////////////////////////////////////////////////////////////
    //Reset
    //////////////////////////////////////////////////////////////////////////////////////////////////////
    messageReceived = false;
  }
  ros::spinOnce();
  //Maybe add wait here...

  waitForPlan_time.sleep();
}
  //Loop back to start of main Program

  // END_TUTORIAL
  sleep_time.sleep();
  ROS_INFO("Done");
  planner_instance.reset();

  return 0;
}
