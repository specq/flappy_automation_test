#ifndef FLAPPY_AUTOMATION_CODE_H_
#define FLAPPY_AUTOMATION_CODE_H_

#include <ros/ros.h>
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Vector3.h"
#include "math.h"

#define VX_SEARCH 2
#define VY_VERTICAL_SCAN 2
#define VX_CROSS_BARRIER 2

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

  int no_barrier_detected(const std::vector<double> ranges){
    if(!ranges.empty()){
      for(int i=1; i<ranges.size()-1; i++){
        if(ranges[i] < 2.5){
          return 0;
        }
      }
    }
    return 1;
  }

  double distance_from_barrier(const std::vector<double> ranges, const std::vector<double> angles){
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
    static int iter[8];
    static double integral_vel_y = 0;
    static double dead_distance = 0;
    static double y_pos = 0;

    //ROS_INFO("%f, %f", vel_msg->x, vel_msg->y);

    y_pos += vel_msg->y/30.0;

    switch(m_state){
      case SEARCH: 
        integral_vel_y += vel_msg->y/30.0;
        if(!ranges.empty() && no_barrier_detected(ranges)){
          if(abs(ranges.front()-ranges.back())<0.05){
            y_pos = 0;
          }
        }
        if(imminent_collision(ranges)){
          m_state = VERTICAL_SCAN;
          //ROS_INFO("vertical_scan");
        }
        else if(front_is_clear(ranges) && obstacle_close(ranges)){
          m_state = CROSS_BARRIER;
          //ROS_INFO("cross");
        }
        else{
          double dir = compute_direction(ranges, angles);
          double dist_from_barrier = distance_from_barrier(ranges, angles);
          /**if(dist_from_barrier < 1.25){
            vx_goal = 1.6;
          }
          else{
            vx_goal = 2.25;
          }
          vy_goal = 20*vx_goal*dir;
          if(vy_goal > 2){
            vy_goal = 2;
          }
          else if(vy_goal < -2){
            vy_goal = -2;
          }*/
          /**
           * @brief Si ne trouve pas les beams, aller cote oppose à la position en ligne droite avec un certain angle
           * garder centrage au début afin de trouver le y0
           * 
           */
          ROS_INFO("OK1");
          
          if(!ranges.empty() && dist_from_barrier < 2){
            for(int i=0; i<ranges.size()-1; i++){
              if(ranges[i+1] > 2 && ranges[i] > 2){
                if(++iter[i] >= 1){
                  ROS_INFO("OK2");
                  dir = (angles[i+1]+angles[i])/2;
                  //ROS_INFO("%f", dir*180/3.14);
                  vx_goal = 3*cos(dir);
                  vy_goal = 3*sin(dir);
                }
              }
              else{
                iter[i] = 0;
              }
            }
          }
          /**if(!ranges.empty() && integral_vel_y < 0 && ranges[ranges.size()/2] > 2 && ranges[ranges.size()/2-1] > 2){
            iter_up = 0;
            if(++iter_down >= 3){
              ROS_INFO("DOWN");
              dir = (angles[angles.size()/2-1]+angles[angles.size()/2])/2;
              vx_goal = 2*cos(dir);
              vy_goal = 2*sin(dir);
            }
          } 
          else if(!ranges.empty() && integral_vel_y > 0 && ranges[ranges.size()/2] > 2 && ranges[ranges.size()/2+1] > 2){
            iter_down = 0;
            if(++iter_up >= 3){
              ROS_INFO("UP");
              dir = (angles[angles.size()/2+1]+angles[angles.size()/2])/2;
              vx_goal = 2*cos(dir);
              vy_goal = 2*sin(dir);
            }
          }
          else{
            iter_down = 0;
            iter_up = 0;
          }*/

          break;
        }
      
      /** Vertical scan state: If flappy bird is too close from the barrier while he did not find the openeing,
       *  he stops and scans the barrier vertically until he finds the openeing
      */
      case VERTICAL_SCAN: 
        if(front_is_clear(ranges)){  // Check if Flappy bird is facing the opening
          m_state = CROSS_BARRIER;
          vertical_scan_initialized = 0;
          integral_vel_y = 0;
          //ROS_INFO("cross");
        }
        else{
          vx_goal = 0; // Stop Flappy bird
          if(!vertical_scan_initialized){   // Choose scanning direction (up or down)
            vertical_dir = adjacent_beams_detector(ranges, angles);
            vertical_scan_initialized = 1;
          }
          else if(floor_close(ranges) && y_pos < -0.9 && vertical_dir == DOWN){   // Go up if the floor is detected
            vertical_dir = UP;
          }
          else if(ceiling_close(ranges) && y_pos > 0.9 && vertical_dir == UP){   // Go down if the ceiling is detected
            vertical_dir = DOWN;
          }
          
          // Set the vertical speed according to the scanning direction
          if(vertical_dir == UP){   
            vy_goal = VY_VERTICAL_SCAN;
          }
          else if(vertical_dir == DOWN){
            vy_goal = -VY_VERTICAL_SCAN;
          }
          /** If no scanning direction was found using the adjacent free beams dectector,
          /*  choose the scanning direction based on whether flappy bird has gone up or done during the search state
          */
          else if(vertical_dir == NONE){  
            if(integral_vel_y >= 0){
              vy_goal = VY_VERTICAL_SCAN;
              vertical_dir = UP;
            }
            else{
              vy_goal = -VY_VERTICAL_SCAN;
              vertical_dir = DOWN;
            }
          }
          break;
        }
      /** Cross barrier state: Flappy bird moves horizontally until he no longer detects the barrier 
       *  and reaches the dead distance. The dead distance is the distance covered after flappy bird
       *  no longer detects the barrier. It avoids collisions with the barrier in case flappy bird faces 
       *  the next opening to early.
      */
      case CROSS_BARRIER: 
        vy_goal = 0;
        if(!obstacle_close(ranges)){
          dead_distance += vel_msg->x/30.0;
          if(dead_distance > 0.15){
            dead_distance = 0;
            m_state = SEARCH;
            //ROS_INFO("search");
          }
          vx_goal = 2.75;
        }
        else{
          vx_goal = 2.75;
        }
        
        break;
      default: break;
    }

    acc_cmd.x = 50*(vx_goal-vel_msg->x);
    acc_cmd.y = 50*(vy_goal-vel_msg->y);
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
