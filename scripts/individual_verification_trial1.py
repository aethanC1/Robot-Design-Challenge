#! /usr/bin/env python3
# Copyright 2021 Samsung Research America
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from copy import deepcopy

from geometry_msgs.msg import PoseStamped, Quaternion
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import math
import serial
import rclpy

"""
Basic stock inspection demo. In this demonstration, the expectation
is that there are cameras or RFID sensors mounted on the robots
collecting information about stock quantity and location.
"""

def euler_from_quaternion(x, y, z, w):
        """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)
        """
        t0 = 2.0 * (w * x + y * z)
        t1 = 1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)
     
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)
     
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
     
        return roll_x, pitch_y, yaw_z # in radians
        
def quaternion_from_euler(roll, pitch, yaw):
	"""
	Convert euler angles back into a quaternion to be
	used for orientation in ros2
	"""
	cos_y = math.cos(yaw * 0.5)
	sin_y = math.sin(yaw * 0.5)
	cos_p = math.cos(pitch * 0.5)
	sin_p = math.sin(pitch * 0.5)
	cos_r = math.cos(roll * 0.5)
	sin_r = math.sin(roll * 0.5)

	w = cos_r * cos_p * cos_y + sin_r * sin_p * sin_y
	x = sin_r * cos_p * cos_y - cos_r * sin_p * sin_y
	y = cos_r * sin_p * cos_y + sin_r * cos_p * sin_y
	z = cos_r * cos_p * sin_y - sin_r * sin_p * cos_y

	return [x, y, z, w]

def navigate_to(navigator, route):

    waypoints = []
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.header.stamp = navigator.get_clock().now().to_msg()
    for point in route:
        x, y, z, w = quaternion_from_euler(0, 0, point[2])
        pose.pose.position.x = point[0]
        pose.pose.position.y = point[1]
        pose.pose.orientation.x = x
        pose.pose.orientation.y = y
        pose.pose.orientation.z = z
        pose.pose.orientation.w = w
        waypoints.append(deepcopy(pose))
    navigator.followWaypoints(waypoints)

    i = 0
    while not navigator.isTaskComplete():
        i += 1
        feedback = navigator.getFeedback()
        if feedback and i % 5 == 0:
            print('Executing current waypoint: ' +
                  str(feedback.current_waypoint + 1) + '/' + str(len(waypoints)))

    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        return True
    elif result == TaskResult.CANCELED:
        return False
    elif result == TaskResult.FAILED:
        return False
    
def recover_position(navigator, position):

    last_valid_position = position[len(position) - 1]
    pose = PoseStamped()
    x, y, z, w = quaternion_from_euler(0, 0, last_valid_position[2])
    pose.pose.position.x = last_valid_position[0]
    pose.pose.position.y = last_valid_position[1]
    pose.pose.orientation.x = x
    pose.pose.orientation.y = y
    pose.pose.orientation.z = z
    pose.pose.orientation.w = w
    pose.append(deepcopy(pose))
    navigator.goToPose(pose)
    while not navigator.isTaskComplete():
        pass

 
def main():
 
    rclpy.init()

    navigator = BasicNavigator()

    #Points for route, points are x, y, yaw respectively
    point_1 = [[0.16, 1.83, 0.0]]
    point_2 = [[0.66, 1.83, 0.0]]
    point_3 = [[1.16, 1.83, 0.0]]
    point_4 = [[1.3, 3.33, 1.57],
               [3.0, 3.35, 0]]
    point_5 = [[1.3, 3.33, 3.14],
               [1.3, 1.2, -1.57]]
    point_6 = [[1.3, 0.7, -1.57]]
    point_7 = [[1.8, 1.83, 1.57]]
    point_8 = [[2.3, 1.83, 0]]
    point_9 = [[2.3, 1.2, -1.57],
    		[3.05, 1.2, 0]]
    point_10 = [[3.05, 1.7, 1.57]]
    point_11 = [[3.05, 2.3, 1.57]]
    point_12 = [[3.6, 1.7, 0]]
    point_13 = [[3.6, 0.0, 0.0]]
    
    		

    inspection_route = [point_1, point_2, point_3, point_4, point_5, point_6, point_7, point_8, point_9, point_10, point_11, point_12, point_13]

    # Set our demo's initial pose
    initial_pose = PoseStamped()
    initial_pose.header.frame_id = 'map'
    initial_pose.header.stamp = navigator.get_clock().now().to_msg()
    initial_pose.pose.position.x = 0.0
    initial_pose.pose.position.y = 0.0
    initial_pose.pose.orientation.z = 0.0
    initial_pose.pose.orientation.w = 1.0
    navigator.setInitialPose(initial_pose)

    # Wait for navigation to fully activate
    navigator.waitUntilNav2Active()

    # Send our route
    max_steps = len(inspection_route)
    step = 0
    while step < max_steps:
        if navigate_to(navigator, inspection_route[step]):
            step += 1
            print('Initating Leak Check Procedure')
        else:
            if step:
                recover_position(navigator, inspection_route[step - 1])
            else:
                navigator.goToPose(initial_pose)
                while not navigator.isTaskComplete():
                    pass
            

    exit(0)


if __name__ == '__main__':
    main()
    
