#ifndef FLAPPY_AUTOMATION_CODE_H_
#define FLAPPY_AUTOMATION_CODE_H_

#include <ros/ros.h>
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Vector3.h"
#include "math.h"

#define VX_SEARCH 1
#define VY_VERTICAL_SCAN 2
#define VX_CROSS_BARRIER 3

class StateMachine {
  private:
  enum States {SEARCH, CROSS_BARRIER, VERTICAL_SCAN};
  enum Direction {UP, DOWN, NONE};
  int m_state;

  int imminent_collision(const std::vector<double> ranges){
    if(!ranges.empty()){
      for(int i=0; i<ranges.size(); i++){
        if(ranges[i] < 0.75){
          return 1;
        } 
      }
    }
    return 0;
  }

  int obstacle_close(const std::vector<double> ranges){
    if(!ranges.empty()){
      for(int i=0; i<ranges.size(); i++){
        if(ranges[i] < 1) return 1;
      }
    }
    return 0;
  }

  int front_is_clear(const std::vector<double> ranges){
    if(!ranges.empty()){
      if(ranges[ranges.size()/2] < 2 || ranges[ranges.size()/2-1] < 2 || ranges[ranges.size()/2+1] < 2){
        return 0;
      }
    }
    return 1;
  }

  int floor_close(const std::vector<double> ranges){
    return ranges.front() < 0.4;
  }

  int ceiling_close(const std::vector<double> ranges){
    return ranges.back() < 0.4;
  }

  int adjacent_beams_detector(const std::vector<double> ranges, const std::vector<double> angles){
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

  double compute_direction(const std::vector<double> ranges, const std::vector<double> angles){
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

  public:
  StateMachine() : m_state(SEARCH){}
  geometry_msgs::Vector3 compute_acc(const std::vector<double> ranges, 
                                     const std::vector<double> angles, 
                                     const geometry_msgs::Vector3::ConstPtr& vel_msg){
    geometry_msgs::Vector3 acc_cmd;
    double vx_goal;
    double vy_goal;
    static int vertical_dir;
    static int vertical_scan_initialized = 0;
    static int dead_distance_iter = 0;
    static double integral_acc_y = 0;

    switch(m_state){
      case SEARCH: 
        //ROS_INFO("Search");
        if(imminent_collision(ranges)){
          m_state = VERTICAL_SCAN;
          ROS_INFO("vertical_scan");
        }
        else if(front_is_clear(ranges) && obstacle_close(ranges)){
          m_state = CROSS_BARRIER;
          ROS_INFO("cross");
        }
        else{
          double dir = compute_direction(ranges, angles);
          vx_goal = VX_SEARCH;
          vy_goal = 20*vx_goal*dir;
          //vy_goal = 0;
          break;
        }
      case VERTICAL_SCAN: 
        //ROS_INFO("Vertical scan");

        if(front_is_clear(ranges)){
          m_state = CROSS_BARRIER;
          ROS_INFO("cross");
          vertical_scan_initialized = 0;
        }
        else{
          vx_goal = 0;
          // Choose direction (up or down)
          if(!vertical_scan_initialized){
            vertical_dir = adjacent_beams_detector(ranges, angles);
            vertical_scan_initialized = 1;
          }
          else if(floor_close(ranges) && (vertical_dir == DOWN || vertical_dir == NONE)){
            ROS_INFO("EHOH");
            vertical_dir = UP;
          }
          else if(ceiling_close(ranges) && vertical_dir == UP){
            vertical_dir = DOWN;
          }
          
          if(vertical_dir == UP){
            vy_goal = VY_VERTICAL_SCAN;
          }
          else if(vertical_dir == DOWN){
            vy_goal = -VY_VERTICAL_SCAN;
          }
          else if(vertical_dir == NONE){
            if(integral_acc_y >= 0){
              vy_goal = VY_VERTICAL_SCAN;
            }
            else{
              vy_goal = -VY_VERTICAL_SCAN;
            }
          }
          break;
        }

      case CROSS_BARRIER: 
        //ROS_INFO("Cross barrier");
        vy_goal = 0;
        if(!obstacle_close(ranges)){
          if(++dead_distance_iter == 2){
            dead_distance_iter = 0;
            m_state = SEARCH;
            integral_acc_y = 0;
            ROS_INFO("search");
          }
        }
        vx_goal = VX_CROSS_BARRIER;
        break;
      default: break;
    }

    acc_cmd.x = 20*(vx_goal-vel_msg->x);
    acc_cmd.y = 20*(vy_goal-vel_msg->y);
    integral_acc_y += acc_cmd.y;
    return acc_cmd;
  }
};

class LaserScaner {
  private:

  // Private attributes
  std::vector<double> m_ranges;
  std::vector<double> m_angles;
  double m_angle_increment;
  double m_angle_min;

  // Private methods
  void compute_angles(void){
    m_angles.clear();
    for(int i=0; i<m_ranges.size(); i++){
      this->m_angles.push_back(m_angle_min+i*m_angle_increment);
    }
  }

  public: 
  void update(const sensor_msgs::LaserScan::ConstPtr& msg){
    m_angle_min = msg->angle_min;
    m_angle_increment = msg->angle_increment;
    m_ranges.clear();
    for(int i=0; i<msg->ranges.size(); i++){
      this->m_ranges.push_back(msg->ranges[i]);
    }
    compute_angles();
  }
  
  std::vector<double> get_ranges(){
    return m_ranges;
  }

  std::vector<double> get_angles(){
    return m_angles;
  }
};

//Ros nodehandle
ros::NodeHandle* nh_= NULL;
//Publisher for acceleration command
ros::Publisher pub_acc_cmd;
//Subscriber for velocity
ros::Subscriber sub_vel;
//Subscriber for laser scan
ros::Subscriber sub_laser_scan;

LaserScaner laser_scanner;
StateMachine state_machine;

void initNode();
void velCallback(const geometry_msgs::Vector3::ConstPtr& msg);
void laserScanCallback(const sensor_msgs::LaserScan::ConstPtr& msg);

#endif
