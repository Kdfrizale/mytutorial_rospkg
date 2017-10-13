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

#include <moveit/planning_pipeline/planning_pipeline.h>

#include <algorithm>
#include <iterator>
#include <string>
#include <std_msgs/Float64.h>

/////////////////////////////////////////GLOBALS//////////////////////////////////////////////


geometry_msgs::PoseStamped poseTip2;//Right Finger tip
geometry_msgs::PoseStamped poseLink6;//wrist
geometry_msgs::PoseStamped poseTip1;//Left Finger tip
bool messageReceived = false;

// A tolerance of 0.01 m is specified in position
// and 0.01 radians in orientation
std::vector<double> tolerance_pose(3, 0.01);
std::vector<double> tolerance_angle(3, 0.01);//0.01 is normal

/////////////////////////////////////FUNCTIONS/////////////////////////////////////////////
//Sets the desired poseTargets to the received input poses
void updatePoseValues(const mytutorialPkg::HandStampedPose::ConstPtr& msg){
  ROS_INFO_THROTTLE(1,"I received a message.. now processing...");
  poseTip2 = msg->poseTip2;
  poseLink6 = msg->poseLink6;
  poseTip1 = msg->poseTip1;
  //poseTip2.header.stamp = ros::Time(0);
  //poseLink6.header.stamp = ros::Time(0);
  //poseTip1.header.stamp = ros::Time(0);
  messageReceived = true;
}

//Combines two trajectories so that the action server can have just one move for two plans
moveit_msgs::RobotTrajectory combineTrajectories(const moveit_msgs::RobotTrajectory mainTrajectory, const moveit_msgs::RobotTrajectory secondaryTrajectory){
  moveit_msgs::RobotTrajectory combineTrajectories = mainTrajectory;
  //Then add additional joint_names and their corresponding values(pos,vel,accel) to the combineTrajectories
  //Also beaware of the time from start for each point, it may be neseccary to take the largest one out of main and secondary
  combineTrajectories.joint_trajectory.joint_names.push_back("m1n6a200_joint_finger_1");
  ROS_INFO("ABOUT TO COMBINE trajectories");
  ROS_INFO("There are [%zd] points to go through",combineTrajectories.joint_trajectory.points.size());
  ROS_INFO("There are [%zd] points to go through",secondaryTrajectory.joint_trajectory.points.size());

  int smallestSize = combineTrajectories.joint_trajectory.points.size();
  if (secondaryTrajectory.joint_trajectory.points.size() < smallestSize){
    smallestSize = secondaryTrajectory.joint_trajectory.points.size();
  }

  for (int i =0; i < smallestSize;i++){
    ROS_INFO("IM in a loop for combing...");
    auto positionValue = secondaryTrajectory.joint_trajectory.points[i].positions.back();
    auto  velocityValue = secondaryTrajectory.joint_trajectory.points[i].velocities.back();
    auto  accelValue = secondaryTrajectory.joint_trajectory.points[i].accelerations.back();

    //push back the last value from secondary onto each points last postion,vel,accel //maybe change start form time
    combineTrajectories.joint_trajectory.points[i].positions.push_back(positionValue);
    combineTrajectories.joint_trajectory.points[i].velocities.push_back(velocityValue);
    combineTrajectories.joint_trajectory.points[i].accelerations.push_back(accelValue);

    ROS_INFO("combines orginal starttime: [%f]", combineTrajectories.joint_trajectory.points[i].time_from_start.toSec());
    ROS_INFO("secondary orginal starttime: [%f]", secondaryTrajectory.joint_trajectory.points[i].time_from_start.toSec());


    if (combineTrajectories.joint_trajectory.points[i].time_from_start < secondaryTrajectory.joint_trajectory.points[i].time_from_start){
      combineTrajectories.joint_trajectory.points[i].time_from_start = secondaryTrajectory.joint_trajectory.points[i].time_from_start;
    }
    ROS_INFO("combines final starttime: [%f]", combineTrajectories.joint_trajectory.points[i].time_from_start.toSec());
  }
  return combineTrajectories;
}

//////////////////////////////////////////////////////////////////MAIN//////////////////////////////////////////////////////////////////////

int main(int argc, char** argv)
{
  ros::init(argc, argv, "myPlannerStart");
  ros::AsyncSpinner spinner(1);
  spinner.start();
  ros::NodeHandle node_handle("~");

  robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
  robot_model::RobotModelPtr robot_model = robot_model_loader.getModel();

  planning_scene::PlanningScenePtr planning_scene(new planning_scene::PlanningScene(robot_model));
  planning_scene::PlanningScenePtr planning_scene2(new planning_scene::PlanningScene(robot_model));

  planning_pipeline::PlanningPipelinePtr planning_pipeline(new planning_pipeline::PlanningPipeline(robot_model, node_handle, "planning_plugin", "request_adapters"));

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
    if(!messageReceived){
      ROS_INFO_THROTTLE(1,"Message not Received...");

    }
    else{//messageReceived so start planning
      ROS_INFO_THROTTLE(1,"Message was Received!");

      std::clock_t start;
      double duration;
      start = std::clock();

      //////////////////////////////////////////////////////////////////////////////////////////////////////
      //PLANNING
      //////////////////////////////////////////////////////////////////////////////////////////////////////
      planning_interface::MotionPlanRequest req;
      planning_interface::MotionPlanResponse res;
      planning_interface::MotionPlanRequest req2;
      planning_interface::MotionPlanResponse res2;

      req.group_name = "chainArm";
      moveit_msgs::Constraints pose_goal_tip_2 = kinematic_constraints::constructGoalConstraints("m1n6a200_link_finger_tip_2", poseTip2, tolerance_pose, tolerance_angle);
      req.goal_constraints.push_back(pose_goal_tip_2);

      moveit_msgs::Constraints pose_goal_link6 = kinematic_constraints::constructGoalConstraints("m1n6a200_link_6", poseLink6, tolerance_pose, tolerance_angle);
      req.goal_constraints.push_back(pose_goal_link6);

      ROS_INFO("about to start planning");
      planning_pipeline->generatePlan(planning_scene,req,res);

      if (res.error_code_.val != res.error_code_.SUCCESS){
        ROS_ERROR("Could not compute plan successfully");
        //sleep_time.sleep();
        continue;//Go back and get new poseTarget
      }

      //-------------Robot has planned to the first motion------------------------------------
      moveit_msgs::MotionPlanResponse midResponse;
      res.getMessage(midResponse);//Get the first motion plan

      duration = ( std::clock() - start ) / (double) CLOCKS_PER_SEC;
      ROS_INFO("It took [%f] seconds to get past the first plan", duration);

      /*//Update the robot model in planning space to the end of plan1's motion
      robot_state::RobotState& robot_state = planning_scene->getCurrentStateNonConst();
      planning_scene->setCurrentState(midResponse.trajectory_start);
      const robot_state::JointModelGroup* joint_model_group = robot_state.getJointModelGroup("chainArm");
      robot_state.setJointGroupPositions(joint_model_group, midResponse.trajectory.joint_trajectory.points.back().positions);
*/
      req2.group_name = "chainArmLeft";
      moveit_msgs::Constraints pose_goal_tip_1 = kinematic_constraints::constructGoalConstraints("m1n6a200_link_finger_tip_1", poseTip1, tolerance_pose, tolerance_angle);
      req2.goal_constraints.push_back(pose_goal_tip_1);
      req2.goal_constraints.push_back(pose_goal_link6);

      ROS_INFO("about to start planning2");
      planning_pipeline->generatePlan(planning_scene,req2,res2);

      if (res2.error_code_.val != res2.error_code_.SUCCESS){
        ROS_ERROR("Could not compute plan successfully");
        //sleep_time.sleep();
        continue;//Go Back and get new Pose Target
      }

      duration = ( std::clock() - start ) / (double) CLOCKS_PER_SEC;
      ROS_INFO("It took [%f] seconds to get past the second plan", duration);

      moveit_msgs::MotionPlanResponse endResponse;
      res2.getMessage(endResponse);

    /*  robot_state = planning_scene->getCurrentStateNonConst();
      planning_scene->setCurrentState(endResponse.trajectory_start);
      joint_model_group = robot_state.getJointModelGroup("chainArmLeft");
      robot_state.setJointGroupPositions(joint_model_group, endResponse.trajectory.joint_trajectory.points.back().positions);
*/
      //////////////////////////////////////////////////////////////////////////////////////////////////////
      //Update the Planning Scene Robot Model to show where the plans ended
      //////////////////////////////////////////////////////////////////////////////////////////////////////
      //Update the whole robot from first plan (main pose)
      robot_state::RobotState& robot_state = planning_scene->getCurrentStateNonConst();
      planning_scene->setCurrentState(midResponse.trajectory_start);
      const robot_state::JointModelGroup* joint_model_group = robot_state.getJointModelGroup("chainArm");
      robot_state.setJointGroupPositions(joint_model_group, midResponse.trajectory.joint_trajectory.points.back().positions);

      //Then update just the finger from the second plan
      ROS_INFO("the number is : [%f]",endResponse.trajectory.joint_trajectory.points.back().positions.back() );
      const double fingerPosition = endResponse.trajectory.joint_trajectory.points.back().positions.back();
      const std::string jointFingerName = "m1n6a200_joint_finger_1";
      const double* fingerPositionPointer= &fingerPosition;
      robot_state.setJointPositions(jointFingerName, fingerPositionPointer );


      //////////////////////////////////////////////////////////////////////////////////////////////////////
      //Move the physical robot to the planned coordinates
      //////////////////////////////////////////////////////////////////////////////////////////////////////
      ROS_INFO("ATTEMPTING TO SEND MOVE COMMANDS...");
      actionlib::SimpleActionClient<moveit_msgs::ExecuteTrajectoryAction> ac("execute_trajectory",false);
      ROS_INFO("Waiting for action server to start.");
      ac.waitForServer();

      moveit_msgs::ExecuteTrajectoryGoal goal;
      goal.trajectory = combineTrajectories(midResponse.trajectory, endResponse.trajectory);
      //waitForPlan_time.sleep();

      actionlib::SimpleClientGoalState waitState = ac.sendGoalAndWait(goal);
      if (waitState.toString().compare("ABORTED") == 0){
          ROS_INFO("it aborted, now sleeping for atime");
          sleep_time.sleep();
      }

      bool finished_before_timeout2 = ac.waitForResult(ros::Duration(30.0));
      if (finished_before_timeout2){
        actionlib::SimpleClientGoalState state = ac.getState();
        ROS_INFO("Action finished: %s",state.toString().c_str());
      }
      else{
        ROS_INFO("Action did not finish before the time out.");
      }

      duration = ( std::clock() - start ) / (double) CLOCKS_PER_SEC;
      ROS_INFO("It took [%f] seconds to get past the second move", duration);

      //////////////////////////////////////////////////////////////////////////////////////////////////////
      //show the result in Rviz
      //////////////////////////////////////////////////////////////////////////////////////////////////////
      ros::Publisher display_publisher = node_handle.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
      moveit_msgs::DisplayTrajectory display_trajectory;

      ROS_INFO("Visualizing the trajectory");
      //moveit_msgs::MotionPlanResponse response;

      //res.getMessage(response);//Get the first motion plan
      display_trajectory.trajectory_start = midResponse.trajectory_start;
      display_trajectory.trajectory.push_back(goal.trajectory);

      /*
      res2.getMessage(response);//Get the second motion plan
      display_trajectory.trajectory_start = response.trajectory_start;
      display_trajectory.trajectory.push_back(response.trajectory);
      */
      display_publisher.publish(display_trajectory);
      waitForPlan_time.sleep();

      duration = ( std::clock() - start ) / (double) CLOCKS_PER_SEC;
      ROS_INFO("It took [%f] seconds to get past the rviz display", duration);


      //////////////////////////////////////////////////////////////////////////////////////////////////////
      //Reset
      //////////////////////////////////////////////////////////////////////////////////////////////////////
      messageReceived = false;

      duration = ( std::clock() - start ) / (double) CLOCKS_PER_SEC;
      ROS_INFO("It took [%f] seconds for this cycle", duration);
  }
  ros::spinOnce();

  //Maybe add wait here...
  waitForPlan_time.sleep();
}//Loop back to start of main Program

  //Broke out of while
  sleep_time.sleep();
  ROS_INFO("Done");

  return 0;
}
