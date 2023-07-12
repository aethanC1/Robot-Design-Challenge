#!/usr/bin/env python3
from copy import deepcopy
from geometry_msgs.msg import PoseStamped, Quaternion, Twist
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import math
import serial
import rclpy
import time
from rclpy.node import Node

class challenge_node(Node):

    def __init__(self):
        super().__init__('challenge_node')
        self.navigator = BasicNavigator()
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.message = Twist()
        try:
            self.nano = serial.Serial('/dev/ttyUSB4', 9600, timeout = 1)
        except Exception:
            self.nano = serial.Serial('/dev/ttyUSB4', 9600, timeout = 1)
        time.sleep(1)
        self.nano.readline()
    	#Points for route, points are x, y, yaw respectively
        point_1 = [[0.0, 0.0, 1.57]]
        point_2 = [[-0.03, 1.81, 2.2]]
        point_3 = [[1.78, 1.81, 2.2]] 
        #point_3 = [[1.3, 3.33, 1.57],[3.0, 3.35, 0]]
        #point_4 = [[1.3, 3.33, 3.14],[1.3, 1.2, -1.57]]
        #point_5 = [[1.3, 0.7, -1.57]]
        #point_6 = [[1.8, 1.83, 1.57]]        
        #point_7 = [[3.6, 0.0, 0.0]]
        self.inspection_route = [point_1, point_2, point_3]#, point_4, point_5, point_6, point_7]
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

    def response_listen(self, timeout= 0.12):
        time.sleep(timeout)
        response = self.nano.readline().decode('utf-8').rstrip()
        print('\n\n\n\n\n\n'+response+'\n\n\n\n\n\n')
        return response
    def response_listen_2(self, timeout_limit = 25):
        """
        Will wait for a response and return a formatted string from
        the serial output buffer of the connected arduino.
        
        Parameters
        ------------------------------------------------
        timeout_limit: the max number of 10ms sleep cycles it waits
        for a response, default is 25 cycles/250 ms

        ------------------------------------------------
        Returns:
        response: string recieved from arduino serial comm
        """
        timeout = 0
        response = ''
        while not self.nano.in_waiting:
            if timeout == timeout_limit:
                return response
            timeout += 1
            time.sleep(0.01)
        else:
            response = self.nano.readline().decode('utf-8').rstrip()
            print('\n\n\n\n\n\nresponse\n\n\n\n\n\n')
            return response


    def detect_ground(self):
        """
        Continously queries the pipe/ground detector subsystem to
        see if the sensor is positioned over the pipe/grate, will either
        stop when over the pipe, or after approximate timeout of 5 seconds
        """
    
        timeout = 0
        self.nano.write(b'light_on')
        self.response_listen()
        time.sleep(1)
        self.nano.write(b'ground')
        ground = self.response_listen()
        time.sleep(0.15)
        while ground == 'wood' and timeout < 25:
            self.nano.write(b'ground')
            ground = self.response_listen()
            timeout += 1
        self.nano.write(b'light_off')
        self.response_listen()

    def detect_humidity(self):
        """
        Queries the humidity sensor multiple times to get multiple readings,
        if the readings are over a certain threshold will return an indication 
        leak is detected

        -----------------------------------------------------------------
        Returns:
            True if leak detected
            False otherwise
        """
        num_samples = 0
        positive_samples = 0
        while num_samples < 10:
            self.nano.write(b'humidity')
            result = self.response_listen()
            if result == "true":
                positive_samples += 1
            num_samples += 1
            if positive_samples > 3:
                return True
        return False
    
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
                if not step:
                    self.nano.write(b'light_on')
                    self.response_listen()
                    time.sleep(1)
                    self.nano.write(b'baseline')
                    self.response_listen()
                    time.sleep(1)
                    self.nano.write(b'light_off')
                    self.response_listen()
                    step+=1
                else:
                    self.message.angular.z = -0.1
                    self.publisher.publish(self.message)
                    self.detect_ground()
                    self.message.angular.z = 0.0
                    self.publisher.publish(self.message)
                    leak = self.detect_humidity()
                    if leak:
                        self.nano.write(b'green_on')
                        self.response_listen()
                        self.nano.write(b'mitigate')
                        self.response_listen()
                        time.sleep(2)
                        self.nano.write(b'light_off')
                        self.response_listen()
                    else:
                        self.nano.write(b'red_on')
                        self.response_listen()
                        time.sleep(2)
                        self.nano.write(b'light_off')
                        self.response_listen()
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
    _challenge = challenge_node()
    rclpy.spin(_challenge)
    rclpy.shutdown()
    
