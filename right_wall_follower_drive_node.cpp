#include <cstdio>
#include <string.h>
#include <stdint.h>
#include <vector>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <sensor_msgs/msg/laser_scan.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <memory>
#include <math.h>

/*Most likely in m/s*/
#define FAST 0.5
#define MEDIUM 0.4
#define SLOW 0.3

/* Radiant to Degree and vice versa*/
#define RAD2DEG(x) ((x)*180. / M_PI)
#define DEG2RAD(x) ((x)*M_PI / 180.)

using namespace std::chrono_literals;

class right_wall_follower : public rclcpp::Node
{
public:
  right_wall_follower()
      : Node("right_wall_follower_drive_node")
  {

    auto qos = rclcpp::QoS(rclcpp::KeepLast(10));

    // Initialise publishers
    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", qos);

    // Initialize subscribers
    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "scan",
        rclcpp::SensorDataQoS(),
        std::bind(
            &right_wall_follower::scan_callback,
            this,
            std::placeholders::_1));
    
    //Initialise ROS timers
    update_timer_ = this->create_wall_timer(10ms, std::bind(&right_wall_follower::update_callback, this));
    RCLCPP_INFO(this->get_logger(), "Wall follower right drive node has been initialized");
  }

private:

  // ROS timer
  rclcpp::TimerBase::SharedPtr update_timer_;
  
  float right_side, left_side, front_right_side, front_left_side, theta1, theta2, kp_, ki_, kd_;
  double integral_, derivative_, prev_error_;
  int count1, count2, count3, count4;
  float sum1, sum2, sum3, sum4, range1, range2, range3, range4, distance1, distance2, dright, dleft, alfa1, alfa2;
  float last_sum1, last_sum2, last_sum3, last_sum4, last_count1, last_count2, last_count3, last_count4;
  float front_view, total_left, total_right, dright_backup, dleft_backup;
  double steer_gap;
  float gap_index_left, gap_index_right, gap_left, gap_right, avg_index_left, avg_index_right;
  
  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  { 
    count1 = count2 = count3 = count4 = 0;
    range1 = range2 = range3 = range4 = 0;
    sum1 = sum2 = sum3 = sum4 = 0;

    std::vector<float> laser_ranges;
    laser_ranges = msg->ranges;
    
    float count = msg->scan_time / msg->time_increment;
    
    front_view = laser_ranges[count/2];
    
    if (front_view > 16.0)
    {
      front_view = 16.0;
    }
    
    else if (front_view < 16.0)
    {
      front_view = front_view;
    } 
    
    else
    {
      front_view = 16.0;
    }
    
    /* FOR LOOP*/
    for (int i = 0; i < count; i++)
    {
      float degree = RAD2DEG(msg->angle_min + msg->angle_increment * i);   
      
      /*************************************
      ** When DEGREE is between -89 & -91 **
      *************************************/
      if (degree < -89.0 && degree > -91.0)
      {        
        if (laser_ranges[i] > 0)
        {
          sum1 = sum1 + msg->ranges[i];
          count1++; 
          last_sum1 = sum1;
          last_count1 = count1;
        }
        else
        {
          sum1 = last_sum1;
          count1 = last_count1;
        }
        dright_backup = laser_ranges[i];
      }
      
      /*************************************
      ** When DEGREE is between -29 & -31 **
      *************************************/
      if (degree < -29.0 && degree > -31.0)
      { 
        if (laser_ranges[i] > 0)
        {
          sum2 = sum2 + msg->ranges[i];
          count2++;
          last_sum2 = sum2;
          last_count2 = count2;
        }
        else
        {
          sum2 = last_sum2;
          count2 = last_count2;
        }
      }

      /*****************************************
      ** When DEGREE is between 91 & 89 **
      ******************************************/
      if (degree < 91.0 && degree > 89.0)
      { 
        if (laser_ranges[i] > 0)
        {
          sum3 = sum3 + msg->ranges[i];
          count3++; 
          last_sum3 = sum3;
          last_count3 = count3;
        }
        else
        {
          sum3 = last_sum3;
          count3 = last_count3;
        }
        dleft_backup = laser_ranges[i];
      }
     
      /*****************************************
        ** When DEGREE is between 31 & 29 **
      *****************************************/
      if (degree < 31.0 && degree > 29.0)
      { 
        if (laser_ranges[i] > 0)
        {
          sum4 = sum4 + msg->ranges[i];
          count4++;
          last_sum4 = sum4;
          last_count4 = count4;
        }
        else
        {
          sum4 = last_sum4;
          count4 = last_count4;
        }
      }
      
      /***************************************
            ** To scan from 80 to -80 **   
      ***************************************/
      if (degree > -80.0 && degree < 80.0)
      { 
        if (laser_ranges[i] > 16.0)
        {
          laser_ranges[i] = 5.0;
        }
        
        if (degree > 10.0 && degree <= 60.0)
        {
          total_left += laser_ranges[i];
          
          if (laser_ranges[i] >= 0.7) //Value 0.7 can be adjusted. If late turn ++ if too fast -- //with 0.02 rad or 1.145 deg need 13 to 17 arrays
          {
            gap_index_left += i;
            gap_left++;
          }
        }
          
        else if (degree < -10.0 && degree >= -60.00)
        {
          total_right += laser_ranges[i];
          
          if (laser_ranges[i] >= 0.7)
          {
            gap_index_right += i;
            gap_right++;
          }
        }        
      }   
    }  
    
    avg_index_right = gap_index_right / gap_right;
    avg_index_left = gap_index_left / gap_left; 
    
    if (total_left > total_right)
    {
      steer_gap = -1 * (msg->angle_increment * ((count/2) - avg_index_left));
      //printf("steer LEFT is %f\n", steer_gap);
    }

    else if (total_right > total_left)
    {
      steer_gap = msg->angle_increment * (avg_index_right - (count/2));
      //printf("steer RIGHT is %f\n", steer_gap);
    }
    
    /****************************************
    ** Calculation for prediction distance **
    ****************************************/
    range1 = sum1 / count1;
    range2 = sum2 / count2;
    range3 = sum3 / count3;
    range4 = sum4 / count4;
    
    right_side = range1; 
    front_right_side = range2;
    left_side = range3;
    front_left_side = range4;

    theta1 = (-30.0) - (-90.0);
    theta2 = 30.0 - 90.0;
    
    float a = range2 * cos(DEG2RAD(theta1)); //To find range1
    float b = range2 * sin(DEG2RAD(theta1)); //To find height of the two scanned point b and a
    
    float e = range4 * cos(DEG2RAD(theta2)); //To find range3
    float f = range4 * sin(DEG2RAD(theta2)); //To find height of the two scanned point f and e
    
    alfa1 = RAD2DEG(atan((a - range1) / (b)));
    alfa2 = RAD2DEG(atan((e - range3) / (f)));

    distance1 = range1 * cos(DEG2RAD(alfa1)); //Instance distance between center of the lidar with right wall
    distance2 = range3 * cos(DEG2RAD(alfa2)); //Instance distance between center of the lidar with left wall

    float c = 0.3;

    dright = distance1 + c * sin(DEG2RAD(alfa1));
    dleft = distance2 + c * sin(DEG2RAD(alfa2));
  }

  // Function prototypes
  void update_callback()
  {
    float mid_distance;    
    
    if (dright < 16.0 && dleft < 16.0)
    {
      mid_distance = (dright + dleft) / 2;
    }
    
    else
    {
      dright = dright_backup;
      dleft = dleft_backup;
      
      if (dright < 16.0 && dleft < 16.0)
      {
      mid_distance = (dright + dleft) / 2;
      }
      
      else
      {
      dright = 5.0;
      dleft = 5.0;
      
      mid_distance = (dright + dleft) / 2;
      }
    }
    
    float front_dist = 0.5; // Set to LOW for PID TEST Only..........

    double error;
    double pid; 
    
    kp_ = 3.27;	//How fast the car to steer to the desired point. Higher is better but to much will lead to spinning
    ki_ = 0.75;	//When there is an obstacle distraction. It is used to correct the lane ofset to its desired point. Too much will makes it oscilated in the desired point.
    kd_ = 0.18;	//Damp the car to steer to the desired point. Too high lead to overdamp which makes it too long to get to the desired point. Too low makes it oscilate
    
    error = mid_distance - dright;
    
    if (mid_distance == 0 && error == 0)
    {
      integral_ = 0;
    }
    
    integral_ += error; 
    
    derivative_ = error - prev_error_;
    
    pid = (kp_ * error) + (ki_ * integral_) + (kd_ * derivative_);
    prev_error_ = error;

    /*******************************
      ******** CONSTRAINT ********  
    *******************************/
    for (int constraint = 0; constraint < 1; constraint++)
    {
      if (pid > 0.0 || steer_gap > 0.0)
      {
        if (pid > 0.0)
        {
          if (pid < 0.3)
          {
            pid = 0.3;
          }
      
          else if (pid > 0.6)
          {
            pid = 0.6;
          }
        }
      
        if (steer_gap > 0.0)
        {
          if (steer_gap < 0.3)
          {
            steer_gap = 0.3;
          }
      
          else if (steer_gap > 0.6)
          {
            steer_gap = 0.6;
          }
        }
      }     

      else if (pid < 0.0 || steer_gap < 0.0)
      {
        if (pid < 0.0)
        {
          if (pid > -0.3)
          {
            pid = -0.3;
          }
      
          else if (pid < -0.8)
          {
            pid = -0.8;
          }
        }
      
        if (steer_gap < 0.0)
        {
          if (steer_gap > -0.3)
          {
            steer_gap = -0.3;
          }
        
          else if (steer_gap < -0.8)
          {
            steer_gap = -0.8;
          }      
        }
      }
    
      else
      {
        pid = 0.0;
        steer_gap = 0.0;
      }
    }
    
    /*******************************
    ******** Motion Planning *******
    *******************************/
    
    if (front_view <= 0.2)
    {
      full_stop();
      //printf("EMERGENCY BREAK!");
    }
    
    else if (front_view <= front_dist) 
    {
      drive_slow(steer_gap);
    }
    
    else
    {
      if (pid >= 0.5 || pid <= -0.5)
      {
        drive_slow(pid);
      }
      
      else if (pid >= 0.4 || pid <= -0.4) 
      {
        drive_medium(pid);
      }

      else
      {
        drive_fast(pid);
      }
    }

    RCLCPP_INFO(this->get_logger(), "%f,%f,%f,%f,%f,%f", front_view, dright, dleft, mid_distance, pid, steer_gap);
  }

  void update_cmd_vel(double linear, double angular)
  {
    geometry_msgs::msg::Twist cmd_vel;
    cmd_vel.linear.x = linear;
    cmd_vel.angular.z = angular;

    cmd_vel_pub_->publish(cmd_vel);
  }
  
  void drive_fast(float ang)
  {
    update_cmd_vel(FAST, ang);
  }
  
  void drive_medium(float ang)
  {
    update_cmd_vel(MEDIUM, ang);
  }
  
  void drive_slow(float ang)
  {
    update_cmd_vel(SLOW, ang);
  }
  
  void full_stop()
  {
    update_cmd_vel(0.0, 0.0);
  }

  // ROS topic publishers
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  // rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;

  // ROS topic subscribers
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<right_wall_follower>());
  rclcpp::shutdown();

  return 0;
}
