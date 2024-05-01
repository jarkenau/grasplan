#!/usr/bin/env python3

# Copyright (c) 2024 DFKI GmbH
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

import rospy
import rosbag
from pathlib import Path


def play_bag(bag_path: str, percentage_start: float, percentage_end: float):
    """
    Replays a ROS bag file within a specified time range.

    Args:
        bag_path (str): The path to the ROS bag file.
        percentage_start (float): Start time percentage of the bag file to play.
        percentage_end (float): End time percentage of the bag file to play.

    Raises:
        FileNotFoundError: If the bag file is not found.
    """
    bag_path = Path(bag_path)
    if not bag_path.exists() or not bag_path.is_file() or bag_path.suffix != ".bag":
        raise FileNotFoundError(f"Bag file not found at: {bag_path.absolute()}")

    with rosbag.Bag(bag_path) as bag:
        start = bag.get_start_time()
        end = bag.get_end_time()
        duration = end - start

        start_time = percentage_start * duration / 100.0 + start
        end_time = percentage_end * duration / 100.0 + start

        for topic, msg, t in bag.read_messages(start_time=start_time, end_time=end_time):
            pub = rospy.Publisher(topic, type(msg), queue_size=10)
            pub.publish(msg)
