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

from array import array
import sys
from telnetlib import STATUS
import time
import math
import threading

from geometry_msgs.msg import PoseStamped

from sensor_msgs.msg import LaserScan

from nav2_msgs.action import NavigateToPose

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

import logging
from tuya_connector import TuyaOpenAPI, TUYA_LOGGER
import os
from dotenv import load_dotenv

from __future__ import division
import RPi.GPIO as GPIO
import Adafruit_PCA9685

load_dotenv()

# Enable debug log
TUYA_LOGGER.setLevel(logging.DEBUG)

# Init OpenAPI and connect
openapi = TuyaOpenAPI(os.environ.get('API_ENDPOINT'), os.environ.get('ACCESS_ID'), os.environ.get('ACCESS_KEY'))
openapi.connect()


global status
global status_idx
status=("start", "infront of elevator", "elevator is open", "take the elevator", "on the elevator", "arrive at the destination floor", "go to destination", "arrived")
status_idx=0

class GoalPoseActionClient(Node):
    def __init__(self):
        super().__init__("goal_pose_action_client")
        #초기 설정
        self.action_client = ActionClient(self, NavigateToPose, "navigate_to_pose") #메세지 타입, topic 경로 지정
        self.get_logger().info("=== Goal Pose Action Client Started ====")


    def send_goal(self, goal_pose):
      goal_msg = NavigateToPose.Goal()
      goal_msg.pose = goal_pose

    #서버를 10초동안 찾는다.
      if self.action_client.wait_for_server(10) is False:
          self.get_logger().error("Server Not exists")

    #서버가 있으면, feedback_callback으로
      self._send_goal_future = self.action_client.send_goal_async(
          goal_msg, feedback_callback=self.feedback_callback
      )

    #서버가 request에 대한 response를 주면 goal_response_callback으로
      self._send_goal_future.add_done_callback(self.goal_response_callback)


    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        print(f"Received feedback: {feedback.current_pose.pose.position}")


    def goal_response_callback(self, future):
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().info("Goal rejected")
            return

        self.get_logger().info("Goal accepted")
				
        self._get_result_future = goal_handle.get_result_async()
        #result를 준 경우
        self._get_result_future.add_done_callback(self.get_result_callback)


    def get_result_callback(self, future):
        global status_idx

        result = future.result().result
        self.get_logger().warn(f"Action Done !! Result: {result}")
        status_idx=status_idx+1

        # rclpy.shutdown()


class LaserSubscriber(Node):

    def __init__(self):
        super().__init__('laser_sub_node')
        queue_size = 10
        self.subscriber = self.create_subscription(
            LaserScan, '/scan', self.sub_callback, queue_size
        )
        self.subscriber  # prevent unused variable warning

    def sub_callback(self, msg):
       #엘레베이터 앞에 도착한 경우, 핑거봇 작동
        if status[status_idx]=="infront of elevator":
            commands = {'commands': [{'code': 'switch', 'value': True}]}
            openapi.post('/v1.0/iot-03/devices/{}/commands'.format(os.environ.get('ONE_FLOOR_BTN')), commands)
        elif status[status_idx]=="on the elevator":
            commands = {'commands': [{'code': 'switch', 'value': True}]}
            openapi.post('/v1.0/iot-03/devices/{}/commands'.format(os.environ.get('DOWN_BTN')), commands)

        if status[status_idx]=="infront of elevator" or status[status_idx]=="on the elevator":
            #엘레베이터 열렸는지 LiDAR의 정보로 확인
            flag =True
            for i in range(291,430):
                if msg.ranges[i]<0.5:
                    flag=False
            
            if flag==True:
                status_idx=status_idx+1
                break

        # self.get_logger().info(f'Raw Laser Data : {msg.ranges[291],msg.ranges[429]}')
        # self.get_logger().info(f'Raw Laser Data : {msg.ranges}')


def main(args=None):
    rclpy.init(args=args)

    #엘레베이터 좌표
    elevator=(42.5, 3, 0)
    in_elevator = (44.5, 3, 180)

    goal_pose_action_client = GoalPoseActionClient()
    laser_subscriber = LaserSubscriber()
		
    executor=rclpy.excutors.MultiThreadedExecutor()
    executor.add_node(goal_pose_action_client)
    executor.add_node(laser_subscriber)

    executor_thread = threading.Thread(target=executor.spin, daemon=True)
    executor_thread.start()

    
    goal_pose=PoseStamped()
    goal_pose.header.frame_id = 'map'
    goal_pose.pose.position.x=float(elevator[0])
    goal_pose.pose.position.y=float(elevator[1])

    angle=float(elevator[2])
    goal_pose.pose.orientation.z=float(math.sin(math.pi*(angle/180)/2))
    goal_pose.pose.orientation.w=float(math.cos(math.pi*(angle/180)/2))

    future = goal_pose_action_client.send_goal(goal_pose)

    while(1):
        if status[status_idx]=="elevator is open":
            goal_pose.header.frame_id = 'map'
            goal_pose.pose.position.x=float(in_elevator[0])
            goal_pose.pose.position.y=float(in_elevator[1])

            angle=float(in_elevator[2])
            goal_pose.pose.orientation.z=float(math.sin(math.pi*(angle/180)/2))
            goal_pose.pose.orientation.w=float(math.cos(math.pi*(angle/180)/2))

            future = goal_pose_action_client.send_goal(goal_pose)

            status_idx=status_idx+1
        elif status[status_idx]=="arrive at the destination floor":
            goal_pose.header.frame_id = 'map'
            goal_pose.header.frame_id = 'map'
            goal_pose.pose.position.x=float(sys.argv[1])
            goal_pose.pose.position.y=float(sys.argv[2])

            angle=float(sys.argv[3])
            goal_pose.pose.orientation.z=float(math.sin(math.pi*(angle/180)/2))
            goal_pose.pose.orientation.w=float(math.cos(math.pi*(angle/180)/2))

            future = goal_pose_action_client.send_goal(goal_pose)

            status_idx=status_idx+1

        #목적지에 도착
        elif status[status_idx]=="arrived":
            pwm = Adafruit_PCA9685.PCA9685(address=0x40, busnum=1)

            #  서보모터의 펄스 길이를 최소, 중간, 최대로 설정
            # servo_min = 130  # Min pulse length out of 4096

            servo_open =  255 # Middle pulse length out of 4096  ##390 
            servo_close = 570  # Max pulse length out of 4096  ##650

            pwm.set_pwm_freq(60)

            pwm.set_pwm(0, 0, servo_open)

            # for 1st Motor on ENA
            ENA = 37
            IN1 = 35
            IN2 = 33

            # set pin numbers to the board's
            GPIO.setmode(GPIO.BOARD)

            # initialize EnA, In1 and In2
            GPIO.setup(ENA, GPIO.OUT, initial=GPIO.LOW)
            GPIO.setup(IN1, GPIO.OUT, initial=GPIO.LOW)
            GPIO.setup(IN2, GPIO.OUT, initial=GPIO.LOW)

            # Stop
            GPIO.output(ENA, GPIO.HIGH)
            GPIO.output(IN1, GPIO.LOW)
            GPIO.output(IN2, GPIO.LOW)
            time.sleep(1)

            # Forward
            GPIO.output(IN1,GPIO.HIGH)
            GPIO.output(IN2, GPIO.LOW)
            time.sleep(3)

            # Stop
            GPIO.output(IN1, GPIO.LOW)
            GPIO.output(IN2, GPIO.LOW)
            time.sleep(1)

            pwm.set_pwm(0, 0, servo_close)

            GPIO.cleanup()



if __name__ == "__main__":
    main()