#!/usr/bin/env python3

# Copyright (c) 2025, Autonomous Robots Lab, Norwegian University of Science and Technology
# All rights reserved.

# This source code is licensed under the BSD-style license found in the
# LICENSE file in the root directory of this source tree.

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

import argparse
parser = argparse.ArgumentParser()
parser.add_argument("topic", help="The topic to subscribe to")
parser.add_argument("file", help="The file to write to")
args = parser.parse_args()

rclpy.init()
node = Node('topic_to_file')
received = []
subscription = node.create_subscription(String, args.topic, received.append, 1)
while rclpy.ok() and not received:
    rclpy.spin_once(node, timeout_sec=0.1)
if received:
    with open(args.file, 'w', encoding='utf-8') as f:
        f.write(received[0].data)
node.destroy_node()
rclpy.shutdown()
