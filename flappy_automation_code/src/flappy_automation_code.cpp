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

int StateMachine::imminent_collision(const std::vector<double> ranges, const std::vector<double> angles){
  if(!ranges.empty()){
    for(int i=0; i<ranges.size(); i++){
      if(ranges[i]*cos(angles[i]) < 0.75){
        return 1;
      } 
    }
  }
  return 0;
}

int StateMachine::no_barrier_detected(const std::vector<double> ranges){
  if(!ranges.empty()){
    for(int i=1; i<ranges.size()-1; i++){
      if(ranges[i] < 2.5){
        return 0;
      }
    }
  }
  return 1;
}

double StateMachine::compute_dist_from_barrier(const std::vector<double> ranges, const std::vector<double> angles){
  double min_dist = -1;
  if(!ranges.empty()){
    for(int i=2; i<ranges.size()-2; i++){
      double dist = ranges[i]*cos(angles[i]);
      if(dist < min_dist || min_dist == -1){
        min_dist = dist;
      }
    }
  }
  return min_dist;
}

int StateMachine::obstacle_close(const std::vector<double> ranges){
  if(!ranges.empty()){
    for(int i=0; i<ranges.size(); i++){
      if(ranges[i] < 1) return 1;
    }
  }
  return 0;
}

int StateMachine::front_is_clear(const std::vector<double> ranges){
  if(!ranges.empty()){
    if(ranges[ranges.size()/2] < 2 || ranges[ranges.size()/2-1] < 2 || ranges[ranges.size()/2+1] < 2){
      return 0;
    }
  }
  return 1;
}

int StateMachine::floor_close(const std::vector<double> ranges){
  return ranges.front() < 0.4;
}

int StateMachine::ceiling_close(const std::vector<double> ranges){
  return ranges.back() < 0.4;
}

int StateMachine::adjacent_beams_detector(const std::vector<double> ranges, const std::vector<double> angles){
  for(int i=0; i<ranges.size()-1; i++){
    if(ranges[i] > 2 && ranges[i+1] > 2){
      if((angles[i]+angles[i+1]/2 > 0)){
        return UP;
      }
      else{
        return DOWN;
      }
    }
  }
  return NONE;
}

double StateMachine::compute_direction(const std::vector<double> ranges, const std::vector<double> angles){
  double sum_ranges = 0;
  double weighted_sum_angles = 0;

  for(int i=0; i<ranges.size();i++){
    weighted_sum_angles += ranges[i]*angles[i];
    sum_ranges += ranges[i];
  }

  if(!sum_ranges){
    return 0;
  }
  return weighted_sum_angles/sum_ranges;
}

geometry_msgs::Vector3 StateMachine::compute_acc(const std::vector<double> ranges, 
                                                 const std::vector<double> angles, 
                                                 const geometry_msgs::Vector3::ConstPtr& vel_msg){

  geometry_msgs::Vector3 acc_cmd;               // Acceleration command
  double vx_goal;                               // Desired x velocity
  double vy_goal;                               // Desired y velocity
  static int vertical_dir;                      // UP or DOWN. Direction to take in the vertical scan state
  static int vertical_scan_initialized;         // Specify whether the vertical scan direction was chosen
  static double integral_vel_y;                 // Distance covered along the y-axis since the beginning of the search state
  static double y_pos;                          // Postion of flappy bird along the y-axis
  
  /**
   * In the cross_barrier state, distance covered along the x-axis
   * since the barrier is no longer detected 
   */
  static double dead_distance;

  /**
   * For each pair of adjacent beams, specifiy the number of
   * consecutive steps where the beam pair was not detecting the barrier
   */
  static std::vector<int> adjacent_iter;               
  if(!ranges.empty() && adjacent_iter.empty()){
    adjacent_iter.resize(ranges.size()-1);
  }

  ROS_INFO("%f, %f", vel_msg->x, vel_msg->y);

  // Intergrate the velocity along the y-axis to compute the position of the bird with respect to the middle
  y_pos += vel_msg->y/ROS_RATE;

  switch(m_state){
    /**
     * Search state: After having crossed a barrier, flappy bird looks for the opening
     * until it finds it or until it gets too close to the barrier. It either follows the 
     * direction of 2 adjacent free beams or, if that is impossible, it follows a trajetory 
     * based on the weighted average of the laser scan angles.
     */
    case SEARCH: 
      // Compute y distance covered since the begininng of the search state
      integral_vel_y += vel_msg->y/ROS_RATE;

      // Before the first barrier, reset the y-axis origin if flappy bird is centered
      if(!ranges.empty() && no_barrier_detected(ranges)){
        if(abs(ranges.front()-ranges.back())<0.05){
          y_pos = 0;
        }
      }

      // If flappy bird is too close from the barrier, enter the vertical scan state
      if(imminent_collision(ranges, angles)){
        m_state = VERTICAL_SCAN;
        ROS_INFO("vertical_scan");
        for(int i=0; i<adjacent_iter.size(); i++){
          adjacent_iter[i] = 0;
        }
      }

      // If flappy bird is in front of and close to the opening, enter the cross_barrier state
      else if(front_is_clear(ranges) && obstacle_close(ranges)){
        m_state = CROSS_BARRIER;
        ROS_INFO("cross_barrier");
        for(int i=0; i<adjacent_iter.size(); i++){
          adjacent_iter[i] = 0;
        }
      }

      // Search the opening
      else{
        // Compute the distance between flappy bird and the barrier
        double dist_from_barrier = compute_dist_from_barrier(ranges, angles);
        
        int adj_beams_found = 0; // Specify whether 2 free adjacent beams were found

        if(!ranges.empty() && dist_from_barrier < 2){
          // Look for free adjacent beams
          for(int i=0; i<ranges.size()-1; i++){
            if(ranges[i+1]*cos(angles[i+1]) > dist_from_barrier+0.3 && ranges[i]*cos(angles[i]) > dist_from_barrier+0.3){
              /**
               * If the number of consecutive steps where 2 adjacent beams are free is higher than a threshold,
               * follow their trajectory
              */
              if(++adjacent_iter[i] >= 3){
                //Choose which beam to follow according to whether it is pointing up or down
                double dir;
                if(i <= 3){
                  dir = angles[i];
                }
                else{
                  dir = angles[i+1];
                }
                vx_goal = 2.5;
                vy_goal = vx_goal*tan(dir);
                adj_beams_found = 1;
              }
            }
            else{
              adjacent_iter[i] = 0;
            }
          }
        }

        /**
        * If no free adjacent beams were found, compute the weighted average of the 
        * angles to orient the research
        */
        if(!adj_beams_found){
          // Compute the weighted average of the angles
          double dir = compute_direction(ranges, angles);

          // Fix and limit the x-velocity according to the distance to the barrier
          if(dist_from_barrier < 1.25){
            vx_goal = 1.6;
          }
          else{
            vx_goal = 2.25;
          }

          // Compute the y-velocity
          vy_goal = 20*vx_goal*dir;

          // Limit the y-velocity to avoid overshooting before facing the opening
          if(vy_goal > 2){
            vy_goal = 2;
          }
          else if(vy_goal < -2){
            vy_goal = -2;
          }
        }
        break;
      }
    
    /** Vertical scan state: If flappy bird is too close from the barrier while he did not find the openeing,
     *  he stops and scans the barrier vertically until it finds the opening
    */
    case VERTICAL_SCAN: 
      // Check whether Flappy bird is facing the opening
      if(front_is_clear(ranges)){ 
        m_state = CROSS_BARRIER;
        vertical_scan_initialized = 0;
        integral_vel_y = 0;
        ROS_INFO("cross_barrier");
      }

      // Vertical scan
      else{
        vx_goal = 0; // Stop Flappy bird

        // Choose scanning direction (up or down)
        if(!vertical_scan_initialized){   
          vertical_dir = adjacent_beams_detector(ranges, angles);
          vertical_scan_initialized = 1;
        }

        // Inverse direction and go up if the floor is detected
        else if(floor_close(ranges) && y_pos < -0.9 && vertical_dir == DOWN){   
          vertical_dir = UP;
        }

        // Inverse direction and go down if the ceiling is detected
        else if(ceiling_close(ranges) && y_pos > 0.9 && vertical_dir == UP){   
          vertical_dir = DOWN;
        }
        
        // Set the vertical speed according to the scanning direction
        if(vertical_dir == UP){   
          vy_goal = 2;
        }
        else if(vertical_dir == DOWN){
          vy_goal = -2;
        }
        /** If no scanning direction was found using the adjacent free beams dectector,
        /*  choose the scanning direction based on whether flappy bird has gone up or done during the search state
        */
        else if(vertical_dir == NONE){  
          if(integral_vel_y >= 0){
            vy_goal = 2;
            vertical_dir = UP;
          }
          else{
            vy_goal = -2;
            vertical_dir = DOWN;
          }
        }
        break;
      }

    /** Cross barrier state: Flappy bird moves horizontally until it no longer detects the barrier 
     *  and reaches the dead distance. The dead distance is the distance covered after flappy bird
     *  no longer detects the barrier. It allows avoiding collisions with the barrier in case flappy bird faces 
     *  the next opening to early.
    */
    case CROSS_BARRIER: 
      // Set the x and y velocities
      vy_goal = 0;
      vx_goal = 2.75;

      // If the barrier is no longer detected, cross a dead distance of 0.175 m before entering the serach state
      if(!obstacle_close(ranges)){
        dead_distance += vel_msg->x/ROS_RATE; // Integrate the velocity
        if(dead_distance > 0.175){
          dead_distance = 0;
          m_state = SEARCH;
          ROS_INFO("search");
        }
      }
      break;
    default: break;
  }

  // P controller to compute the acceleration command
  acc_cmd.x = 50*(vx_goal-vel_msg->x);
  acc_cmd.y = 50*(vy_goal-vel_msg->y);

  return acc_cmd;
}

void LaserScanner::compute_angles(void){
  m_angles.clear();
  for(int i=0; i<m_ranges.size(); i++){
    m_angles.push_back(m_angle_min+i*m_angle_increment);
  }
}

void LaserScanner::update(const sensor_msgs::LaserScan::ConstPtr& msg){
  m_angle_min = msg->angle_min;
  m_angle_increment = msg->angle_increment;
  m_ranges.clear();
  for(int i=0; i<msg->ranges.size(); i++){
    m_ranges.push_back(msg->ranges[i]);
  }
  compute_angles();
}

