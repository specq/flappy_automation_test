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

  // Get the acceleration command
  acc_cmd = state_machine.compute_acc(laser_scanner.get_ranges(),
                                      laser_scanner.get_angles(),
                                      msg);

  pub_acc_cmd.publish(acc_cmd);
}

void laserScanCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
  // Get laser measurements
  
  laser_scanner.update(msg);
}

int main(int argc, char **argv)
{
  ros::init(argc,argv,"flappy_automation_code");
  initNode();

  // Ros spin to prevent program from exiting
  ros::spin();
  return 0;
}
