/*
Code for DT1
V. Sieben
Version 1.0
Date: Feb 4, 2023
License: GNU GPLv3
*/

// Include important C++ header files that provide class
// templates for useful operations.
#include <chrono>		// Timer functions
#include <functional>		// Arithmetic, comparisons, and logical operations
#include <memory>		// Dynamic memory management
#include <string>		// String functions
#include <cmath>
#include <iostream>

// ROS Client Library for C++
#include "rclcpp/rclcpp.hpp"
 
// Message types
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/bool.hpp"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

using namespace std;
using namespace std::chrono_literals;
using std::placeholders::_1;


// Create the node class named SquareRoutine
// It inherits rclcpp::Node class attributes and functions
class SquareRoutine : public rclcpp::Node
{
  public:
	// Constructor creates a node named Square_Routine. 
	SquareRoutine() : Node("Square_Routine")
	{
		// Create the subscription
		// The callback function executes whenever data is published to the 'topic' topic.
		subscription_ = this->create_subscription<nav_msgs::msg::Odometry>("odom", 10, std::bind(&SquareRoutine::topic_callback, this, _1));
          
	  	// Create the timer
	  	timer_ = this->create_wall_timer(500ms, std::bind(&SquareRoutine::timer_callback, this)); 	  
	}

  private:
	
	void topic_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
	{
		tf2::Quaternion q(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
		tf2::Matrix3x3 m(q);
		double roll, pitch, yaw;
		m.getRPY(roll, pitch, yaw);
		theta_now = yaw;
		if (theta_now < 0)
		{
			theta_now += 6.28;
		}
		
		//RCLCPP_INFO(this->get_logger(), "Odom Acquired.");
	}
	
	void timer_callback()
	
	{
		cout << "\nCurrent Heading: ";
		cout << theta_now		
	}

	// Declaration of subscription_ attribute
	rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;
	
	// Declaration of the timer_ attribute
	rclcpp::TimerBase::SharedPtr timer_;
	
	// Declaration of Class Variables
	double theta_now = 0;
};
    	
//------------------------------------------------------------------------------------
// Main code execution
int main(int argc, char * argv[])
{
	// Initialize ROS2
	rclcpp::init(argc, argv);
  
	// Start node and callbacks
	rclcpp::spin(std::make_shared<SquareRoutine>());
 
	// Stop node 
	rclcpp::shutdown();
	return 0;
}



