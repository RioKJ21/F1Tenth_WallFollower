# F1Tenth_WallFollower
This is the primary code to build F1Tenth Racing Car based on ESP-IDF FreeRTOS. LIDAR sensor is used as the only input device here. These codes are collected from other respositories.

// 1st Code for control and provide information needed in microcontroller. Please follow tutorial from https://medium.com/@SameerT009/connect-esp32-to-ros2-foxy-5f06e0cc64df

f1tent_diff_drive //Put it inside the FreeRTOS firmware applications

// 2nd Code for wall_following and navigation using LIDAR. It is taken from ATaufikR repositories : https://github.com/ATaufikR/F1tenth_wall_follower_ROS2

Additional code for RP LIDAR A2 driver:
rplidar_ros2 //Put it inside the src folder

Modified code for wall following and navigation:
right_wall_follower_drive_node.cpp //Put it inside the src folder
