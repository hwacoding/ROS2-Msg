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

# !/usr/bin/env/ python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class LaserSubscriber(Node):

    def __init__(self):
        super().__init__('laser_sub_node')
        queue_size = 10
        self.subscriber = self.create_subscription(
            LaserScan, '/scan', self.sub_callback, queue_size
        )
        self.subscriber  # prevent unused variable warning

    def sub_callback(self, msg):
        for i in range(291,430):
            if msg.ranges[i]>1.0:
                print(i,": ",msg.ranges[i] )
            

        # self.get_logger().info(f'Raw Laser Data : {msg.ranges[291],msg.ranges[429]}')
        # self.get_logger().info(f'Raw Laser Data : {msg.ranges}')


def main(args=None):
    rclpy.init(args=args)

    laser_subscriber = LaserSubscriber()

    rclpy.spin(laser_subscriber)

    laser_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()