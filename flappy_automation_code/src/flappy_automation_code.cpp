#include "ros/ros.h"
#include "flappy_automation_code/flappy_automation_code.hpp"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Vector3.h"

void initNode()
{
  //Initialization of nodehandle
  nh_ = new ros::NodeHandle();
  //Init publishers and subscribers
  pub_acc_cmd = nh_->advertise<geometry_msgs::Vector3>("/flappy_acc",1);
  sub_vel = nh_->subscribe<geometry_msgs::Vector3>("/flappy_vel", 1, velCallback);
  sub_laser_scan = nh_->subscribe<sensor_msgs::LaserScan>("/flappy_laser_scan", 1, laserScanCallback);
}

void velCallback(const geometry_msgs::Vector3::ConstPtr& msg)
{
  // msg has the format of geometry_msgs::Vector3
  // Example of publishing acceleration command on velocity velCallback
  geometry_msgs::Vector3 acc_cmd;
  ROS_INFO("%f, %f", msg->x, msg->y);
  acc_cmd = state_machine.compute_acc(laser_scanner.get_ranges(),
                                      laser_scanner.get_angles(),
                                      msg);
  /**acc_cmd.x = 0;
  acc_cmd.y = 0;*/

  //ROS_INFO("%f, %f", acc_cmd.x, acc_cmd.y);

  pub_acc_cmd.publish(acc_cmd);
}

void laserScanCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
  laser_scanner.update(msg);
  //msg has the format of sensor_msgs::LaserScan
  //print laser angle and range
  /*ROS_INFO("%f, %f, %f, %f, %f, %f, %f, %f, %f", msg->ranges[0], msg->ranges[1],msg->ranges[2], msg->ranges[3], 
                                                 msg->ranges[4], msg->ranges[5], msg->ranges[6], msg->ranges[7], msg->ranges[8]);*/
}

int main(int argc, char **argv)
{
  ros::init(argc,argv,"flappy_automation_code");
  initNode();

  // Ros spin to prevent program from exiting
  ros::spin();
  return 0;
}
