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
          
		// Create the publisher
		// Publisher to a topic named "topic". The size of the queue is 10 messages.
		publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel",10);
      
	  	// Create the timer
	  	timer_ = this->create_wall_timer(100ms, std::bind(&SquareRoutine::timer_callback, this)); 	  
	}

  private:
	
	void topic_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
	{
		x_now = msg->pose.pose.position.x;
		y_now = msg->pose.pose.position.y;
		tf2::Quaternion q(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
		tf2::Matrix3x3 m(q);
		double roll, pitch, yaw;
		m.getRPY(roll, pitch, yaw);
		theta_now = yaw;
		delta_theta = fabs(theta_now - last_theta);
		total_theta += delta_theta;
		last_theta = theta_now;
		
		//RCLCPP_INFO(this->get_logger(), "Odom Acquired.");
	}
	
	void timer_callback()
	
	{
		geometry_msgs::msg::Twist msg;
        	
		// Calculate distance travelled from initial
		d_now =	pow( pow(x_now - x_init, 2) + pow(y_now - y_init, 2), 0.5 );
		
		// Keep moving if not reached last distance target
		if (theta_now < 0)
		{
			float current_heading = 6.28318 - theta_now;
		}
		else
		{
			float current_heading = theta_now;
		}
		
		float remaining_angle = theta_target - total_theta;
		if (remaining_angle > 0)
		{
			msg.linear.x = 0;
			msg.angular.z = theta_vel;
			publisher_->publish(msg);
		}
		
		else if (d_now < d_aim)
		{	
			if (theta_now < heading)
			{
				heading_correction = 0.1;
				}
			else if (theta_now < heading)
			{
				heading_correction = -0.1;
				}
			else
			{
				heading_correction = 0;
				}
			msg.linear.x = x_vel;
			msg.angular.z = heading_correction;
			publisher_->publish(msg);		
		}
		// If done step, stop
		else
		{	
			cout << theta_now;
			cout << '\n';
			msg.linear.x = 0; //double(rand())/double(RAND_MAX); //fun
			msg.angular.z = 0; //2*double(rand())/double(RAND_MAX) - 1; //fun
			publisher_->publish(msg);
			last_state_complete = 1;
		}


		sequence_statemachine();		
		

		//RCLCPP_INFO(this->get_logger(), "Published cmd_vel.");
	}

	void sequence_statemachine()
	{	
		geometry_msgs::msg::Twist msg;
		if (last_state_complete == 1)
		{
			switch(count_) 
			{
			  case 0:
			  	heading = 0;
			  	theta_target = 1.40;
			    move_distance(1.0);
			    break;
			  case 1:
			  	heading = 1.57;
			  	theta_target = 1.40;
			    move_distance(1.0);
			    break;
			  case 2:
			  	heading = 3.14;
			  	theta_target = 1.40;
			    move_distance(1.0);
			    break;
			  case 3:
			  	heading = 4.71;
				theta_target = 1.40;
			    move_distance(1.0);
			    break; 
			  default:
			    break;
			}
		}			
	}
	
	// Set the initial position as where robot is now and put new d_aim in place

	void move_distance(double distance)
	{
		d_aim = distance;
		x_init = x_now;
		y_init = y_now;
		count_++;		// advance state counter
		last_state_complete = 0;
		total_theta = 0;	
	}
	

	// Declaration of subscription_ attribute
	rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;
         
	// Declaration of publisher_ attribute      
	rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
	
	// Declaration of the timer_ attribute
	rclcpp::TimerBase::SharedPtr timer_;
	
	// Declaration of Class Variables
	double x_vel = 0.2;
	double theta_vel = 0.5;
	double x_now = 0, x_init = 0, y_now = 0, y_init = 0;
	double theta_now = 0, theta_target = 0;
	double total_theta = 0, delta_theta = 0, last_theta = 0;
	double d_now = 0, d_aim = 0;
	double heading = 0, heading_correction = 0;
	size_t count_ = 0;
	int last_state_complete = 1;
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



