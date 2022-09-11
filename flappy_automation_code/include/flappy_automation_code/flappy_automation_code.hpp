#ifndef FLAPPY_AUTOMATION_CODE_H_
#define FLAPPY_AUTOMATION_CODE_H_

#include <ros/ros.h>
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Vector3.h"
#include "math.h"

#define ROS_RATE 30.0

class StateMachine {
  private:
  enum States {SEARCH, CROSS_BARRIER, VERTICAL_SCAN};
  enum Direction {UP, DOWN, NONE};
  int m_state;

  /**
   * Return 1 if flappy is less than 0.75 m away from the barrier, Return 0 otherwise
   */
  int imminent_collision(const std::vector<double> ranges, const std::vector<double> angles);

  /**
   * Return 0 if a barrier is detected, 1 otherwise
   * It allows knowing whether the game is starting or not
   */
  int no_barrier_detected(const std::vector<double> ranges);

  /**
   * Compute the distance between flappy bird and the barrier
   */
  double compute_dist_from_barrier(const std::vector<double> ranges, const std::vector<double> angles);

  /**
   * Return 1 if flappy bird is less than 1 m away from an obstacle, 0 otherwise
   */
  int obstacle_close(const std::vector<double> ranges);

  /**
   * Return 1 if the 3 front beams do not detect the barrier, 0 otherwise
   */
  int front_is_clear(const std::vector<double> ranges);

  /**
   * Return 1 if flappy bird is less than 0.4 m away from the floor, 0 otherwise
   */
  int floor_close(const std::vector<double> ranges);
  
  /**
   * Return 1 if flappy bird is less than 0.4 m away from the ceiling, 0 otherwise
   */
  int ceiling_close(const std::vector<double> ranges);

  /**
   * Determine whether there exists 2 adjacent beams not hitting the barrier (free beams).
   * Return UP if both adjacent beams are pointing up
   * Retun DOWN if both adjacent beams are pointing down
   * Return NONE if no adjacent free beams were detected
   */
  int adjacent_beams_detector(const std::vector<double> ranges, const std::vector<double> angles);

  /**
   * Compute a weighted average of the angles. The weights are their respective range.
   * It allows choosing the direction (up or down) during the search state. The returned angle
   * will tend to point toward a direction where the ranges are larger, and therefore where the 
   * path is free from obstacles
   */
  double compute_direction(const std::vector<double> ranges, const std::vector<double> angles);

  public:
  StateMachine() : m_state(SEARCH){}

  /**
   * Compute and return the acceleration command based on the laser scans and the velocity
   */
  geometry_msgs::Vector3 compute_acc(const std::vector<double> ranges, 
                                     const std::vector<double> angles, 
                                     const geometry_msgs::Vector3::ConstPtr& vel_msg);
};

class LaserScanner {
  private:

  std::vector<double> m_ranges;
  std::vector<double> m_angles;
  double m_angle_increment;
  double m_angle_min;

  // Compute laser beam angles
  void compute_angles(void);

  public: 
  
  // Update laser scans
  void update(const sensor_msgs::LaserScan::ConstPtr& msg);

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

LaserScanner laser_scanner;
StateMachine state_machine;

void initNode();
void velCallback(const geometry_msgs::Vector3::ConstPtr& msg);
void laserScanCallback(const sensor_msgs::LaserScan::ConstPtr& msg);

#endif
