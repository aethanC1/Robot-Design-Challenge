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
	used for orientation
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

def main():
    print('Started demo inspection')
    rclpy.init()

    navigator = BasicNavigator()

    # Inspection route, probably read in from a file for a real application
    # from either a map or drive and repeat.
    inspection_route = [
        [1.0, 0.0, 0],
        [1.0, 1.0, 1.57],
        [0.0, 1.0, 3.14],
        [0.0, 0.0, -1.57]
        ]

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
    inspection_points = []
    inspection_pose = PoseStamped()
    inspection_pose.header.frame_id = 'map'
    inspection_pose.header.stamp = navigator.get_clock().now().to_msg()
    #inspection_pose.pose.orientation.z = 0.0
    #inspection_pose.pose.orientation.w = 1.0
    for pt in inspection_route:
        # Takes the euler angle for yaw and gets the quaternion
        x, y, z, w = quaternion_from_euler(0, 0, pt[2])
        # Specifies the x pose for the specific point
        inspection_pose.pose.position.x = pt[0]
        # Specifies the y pose for the specific point
        inspection_pose.pose.position.y = pt[1]
        # Specifies the orientation for the robot at the end
        inspection_pose.pose.orientation.x = x
        inspection_pose.pose.orientation.y = y
        inspection_pose.pose.orientation.z = z
        inspection_pose.pose.orientation.w = w
        inspection_points.append(deepcopy(inspection_pose))
        print('inspection points loaded')
    navigator.followWaypoints(inspection_points)

    # Do something during our route (e.x. AI to analyze stock information or upload to the cloud)
    # Simply the current waypoint ID for the demonstation
    i = 0
    while not navigator.isTaskComplete():
        i += 1
        feedback = navigator.getFeedback()
        if feedback and i % 5 == 0:
            print('Executing current waypoint: ' +
                  str(feedback.current_waypoint + 1) + '/' + str(len(inspection_points)))

    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        print('Inspection of shelves complete! Returning to start...')
    elif result == TaskResult.CANCELED:
        print('Inspection of shelving was canceled. Returning to start...')
    elif result == TaskResult.FAILED:
        print('Inspection of shelving failed! Returning to start...')

    # go back to start
    initial_pose.header.stamp = navigator.get_clock().now().to_msg()
    navigator.goToPose(initial_pose)
    while not navigator.isTaskComplete():
        pass

    exit(0)


if __name__ == '__main__':
    main()
    
