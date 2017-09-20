#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/JointState.h"
#include <cstdlib>

#include <sstream>

int main(int argc, char **argv)
{
  ros::init(argc,argv, "mytalker");
  ros::NodeHandle n;
  ros::Publisher chatter_pub = n.advertise<sensor_msgs::JointState>("joint_states", 10);

  ros::Rate loop_rate(10);

  int count =0;

  while (ros::ok()){
    //std_msgs::String msg;
    //std::stringstream ss;
  //  ss << "hello world" << count;
    //msg.data = ss.str();



    sensor_msgs::JointState msg;
    msg.name.push_back("j2n6s300_joint_1");
    msg.position.push_back(rand() %10);
    msg.name.push_back("j2n6s300_joint_2");
    msg.position.push_back(rand() %10);
    //msg.name = ['j2n6s300_joint_1', 'j2n6s300_joint_2', 'j2n6s300_joint_4', 'j2n6s300_joint_3'];
    //msg.position = [2, 3.59, 7.01, 3.07];
    chatter_pub.publish(msg);

    //ROS_INFO("%s",msg.data.c_str());
    ros::spinOnce();

    loop_rate.sleep();
    ++count;

  }
  return 0;
}
