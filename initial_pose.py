# Copyright 2021 Seoul Business Agency Inc.
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

# Referenced from Below Link
# https://docs.ros2.org/foxy/api/rclpy/api/init_shutdown.html#rclpy.create_node

# !/usr/bin/env/ python3
from geometry_msgs.msg import PoseWithCovarianceStamped
import rclpy
from rclpy.node import Node

import math


class InitialPosePublisher(Node):

    def __init__(self):
        super().__init__('amcl_node')
        self.publisher = self.create_publisher(
            PoseWithCovarianceStamped, '/initialpose', 10
        )  # queue size
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.publish_callback)
        self.get_logger().info(
            'DriveForward node Started, move forward during 5 seconds \n'
        )

    def publish_callback(self):
        initial_msg = PoseWithCovarianceStamped()
        
        initial_msg.header.frame_id = 'map'
        initial_msg.pose.pose.position.x=float(46.7)
        initial_msg.pose.pose.position.y=float(5.6)

        angle=float(181.0)
        initial_msg.pose.pose.orientation.z=float(math.sin(math.pi*(angle/180)/2))
        initial_msg.pose.pose.orientation.w=float(math.cos(math.pi*(angle/180)/2))

        print("message : ")
        print(initial_msg)

        self.publisher.publish(initial_msg)

    # def stop_robot(self):
    #     stop_msg = Twist()S
    #     stop_msg.linear.x = 0.0
    #     stop_msg.angular.z = 0.0
    #     self.publisher.publish(stop_msg)


def main(args=None):
    rclpy.init(args=args)

    initial_pose_publisher = InitialPosePublisher()

    rclpy.spin_once(initial_pose_publisher)

    initial_pose_publisher.get_logger().info('\n==== Stop Publishing ====')
    initial_pose_publisher.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()