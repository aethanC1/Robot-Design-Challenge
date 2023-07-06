#!/usr/bin/env python3
from copy import deepcopy
from geometry_msgs.msg import PoseStamped, Quaternion, Twist
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import math
import serial
import rclpy
import time
from rclpy.node import Node

class trial_one(Node):

    def __init__(self):
        super().__init__('challenge_node')
        self.navigator = BasicNavigator()
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.message = Twist()
      
    	#Points for route, points are x, y, yaw respectively
        point_1 = [[0.16, 1.83, 0.0]]
        point_2 = [[0.66, 1.83, 0.0]]
        point_3 = [[1.16, 1.83, 0.0]]
        point_4 = [[1.3, 3.33, 1.57],[3.0, 3.35, 0]]
        point_5 = [[1.3, 3.33, 3.14],[1.3, 1.2, -1.57]]
        point_6 = [[1.3, 0.7, -1.57]]
        point_7 = [[1.8, 1.83, 1.57]]
        point_8 = [[2.3, 1.83, 0]]
        point_9 = [[2.3, 1.2, -1.57],[3.05, 1.2, 0]]
        point_10 = [[3.05, 1.7, 1.57]]
        point_11 = [[3.05, 2.3, 1.57]]
        point_12 = [[3.6, 1.7, 0]]
        point_13 = [[3.6, 0.0, 0.0]]
        self.inspection_route = [point_1, point_2, point_3, point_4, point_5, point_6, point_7, point_8, point_9, point_10, point_11, point_12, point_13]
    	# Wait for navigation to fully activate
        self.navigator.waitUntilNav2Active()
        self.main()

    def euler_from_quaternion(self, x, y, z, w):
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
            
    def quaternion_from_euler(self, roll, pitch, yaw):
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

    def navigate_to(self, route):
        """
        Takes a list of lists in the form of x,y,theta coordinates 
        navigates to each point in the list, uses ros2 nav2 for 
        waypoint navigation

        -------------------
        Returns 
        True if navigation successful
        False if navigation unsuccessful
        """

        waypoints = []
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.navigator.get_clock().now().to_msg()
        for point in route:
            x, y, z, w = self.quaternion_from_euler(0, 0, point[2])
            pose.pose.position.x = point[0]
            pose.pose.position.y = point[1]
            pose.pose.orientation.x = x
            pose.pose.orientation.y = y
            pose.pose.orientation.z = z
            pose.pose.orientation.w = w
            waypoints.append(deepcopy(pose))
        self.navigator.followWaypoints(waypoints)

        i = 0
        while not self.navigator.isTaskComplete():
            i += 1
            feedback = self.navigator.getFeedback()
            if feedback and i % 5 == 0:
                print('Executing current waypoint: ' +
                    str(feedback.current_waypoint + 1) + '/' + str(len(waypoints)))

        result = self.navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            return True
        elif result == TaskResult.CANCELED:
            return False
        elif result == TaskResult.FAILED:
            return False
        
    def recover_position(self, position):
        """
        In the event the robot can't get to a certain point,
        will set a waypoint to the last valid coordinate it was
        at so it can try again
        """

        last_valid_position = position[len(position) - 1]
        pose = PoseStamped()
        x, y, z, w = self.quaternion_from_euler(0, 0, last_valid_position[2])
        pose.pose.position.x = last_valid_position[0]
        pose.pose.position.y = last_valid_position[1]
        pose.pose.orientation.x = x
        pose.pose.orientation.y = y
        pose.pose.orientation.z = z
        pose.pose.orientation.w = w
        pose.append(deepcopy(pose))
        self.navigator.goToPose(pose)
        while not self.navigator.isTaskComplete():
            pass

    def main(self):
        """
        Goes through a list of points on the pipeline and feeds them into a waypoint
        follower, if the follower is successful, begins leak detection/mitagion procedures
        if unable to navigate, will try to navigate back to last valid position and then continue
        on
        """
        initial_pose = PoseStamped()
        initial_pose.header.frame_id = 'map'
        initial_pose.header.stamp = self.navigator.get_clock().now().to_msg()
        initial_pose.pose.position.x = 0.0
        initial_pose.pose.position.y = 0.0
        initial_pose.pose.orientation.z = 0.0
        initial_pose.pose.orientation.w = 1.0
        self.navigator.setInitialPose(initial_pose)
        max_steps = len(self.inspection_route)
        step = 0
        while step < max_steps:
            if self.navigate_to(self.inspection_route[step]):
                self.message.angular.z = 0.1
                self.publisher.publish(self.message)
                time.sleep(1.5)
                self.message.angular.z = 0.0
                self.publisher.publish(self.message)
                step += 1
            else:
                if step:
                    self.recover_position(self.inspection_route[step - 1])
                else:
                    self.navigator.goToPose(initial_pose)
                    while not self.navigator.isTaskComplete():
                        pass
                

        


if __name__ == '__main__':
    rclpy.init()
    _trial = trial_one()
    rclpy.spin(_trial)
    rclpy.shutdown()
    
