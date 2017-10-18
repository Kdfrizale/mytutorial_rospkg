#include "ros/ros.h"
#include <mytutorialPkg/HandStampedPose.h>
#include <geometry_msgs/PoseStamped.h>
//#include "sensor_msgs/JointState.h"
#include <cstdlib>
#include <iostream>
#include <stdio.h>

void ouch(int sig){
  ROS_INFO("I caught a signal");
}

int main(int argc, char **argv)
{
  ros::init(argc,argv, "myDummyPub");
  ros::NodeHandle n;
  ros::Publisher chatter_pub = n.advertise<mytutorialPkg::HandStampedPose>("/handPoseTopic", 10);
  ros::Rate loop_rate(10);
  int count =0;

  geometry_msgs::PoseStamped sensedPoseTip2;//Right Finger tip---Demo data
  sensedPoseTip2.header.frame_id = "m1n6a200_link_base";
  sensedPoseTip2.pose.position.x = 0.305735;
  sensedPoseTip2.pose.position.y = 0.196963;
  sensedPoseTip2.pose.position.z = 0.663391;
  sensedPoseTip2.pose.orientation.x = 0.623481;
  sensedPoseTip2.pose.orientation.y = 0.3335;
  sensedPoseTip2.pose.orientation.z = 0.623522;
  sensedPoseTip2.pose.orientation.w = 0.333571;

  geometry_msgs::PoseStamped sensedPoseLink6;//wrist --- Demo Data
  sensedPoseLink6.header.frame_id = "m1n6a200_link_base";
  sensedPoseLink6.pose.position.x = 0.308219;
  sensedPoseLink6.pose.position.y = 0.0494515;
  sensedPoseLink6.pose.position.z = 0.727438;
  sensedPoseLink6.pose.orientation.x = 0.615034;
  sensedPoseLink6.pose.orientation.y = 0.0;
  sensedPoseLink6.pose.orientation.z = 0.0;
  sensedPoseLink6.pose.orientation.w = 0.788501;

  geometry_msgs::PoseStamped sensedPoseTip1;//Left Finger tip---Demo data
  sensedPoseTip1.header.frame_id = "m1n6a200_link_base";
  sensedPoseTip1.pose.position.x = 0.30573;
  sensedPoseTip1.pose.position.y = 0.211976;
  sensedPoseTip1.pose.position.z = 0.723024;
  sensedPoseTip1.pose.orientation.x = 0.648451;
  sensedPoseTip1.pose.orientation.y = 0.282066;
  sensedPoseTip1.pose.orientation.z = -0.648369;
  sensedPoseTip1.pose.orientation.w = -0.282078;

  while (ros::ok()){
    sensedPoseTip2.pose.position.y += 0.001;
    sensedPoseLink6.pose.position.y += 0.001;
    sensedPoseTip1.pose.position.y += 0.001;
    sensedPoseTip2.pose.position.x += 0.001;
    sensedPoseLink6.pose.position.x += 0.001;
    sensedPoseTip1.pose.position.x += 0.001;

    mytutorialPkg::HandStampedPose msg;
    msg.poseTip2 = sensedPoseTip2;
    msg.poseLink6 = sensedPoseLink6;
    msg.poseTip1 = sensedPoseTip1;

  //  sensor_msgs::JointState msg;
  //  msg.name.push_back("j2n6s300_joint_1");
  //  msg.position.push_back(rand() %10);
  //  msg.name.push_back("j2n6s300_joint_2");
  //  msg.position.push_back(rand() %10);
    //msg.name = ['j2n6s300_joint_1', 'j2n6s300_joint_2', 'j2n6s300_joint_4', 'j2n6s300_joint_3'];
    //msg.position = [2, 3.59, 7.01, 3.07];

    chatter_pub.publish(msg);
    std::cout << "Press Enter to Continue" ;
    getchar();
    //ROS_INFO("%s",msg.data.c_str());
    ros::spinOnce();

    loop_rate.sleep();
    ++count;

  }
  return 0;
}
