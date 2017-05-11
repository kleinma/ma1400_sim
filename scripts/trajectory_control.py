#!/usr/bin/env python
#
# The MIT License (MIT)
#
# Copyright (c) 2017 Matthew Klein
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
from std_msgs.msg import Float64
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
import math

if __name__ == "__main__":
  rospy.init_node('trajectory_control')
  trajPub = rospy.Publisher('/ma1400/joint_path_command', JointTrajectory,
                            queue_size=1)

  # Wait for time to not equal zero. A zero time means that no message has
  # been received on the /clock topic
  timeZero = rospy.Time.from_sec(0.0)
  while rospy.get_rostime() == timeZero:
    # Sleep for a small time to make sure publishing and subscribing works.
    rospy.sleep(rospy.Duration(1))

  # Create a JointTrajectory object
  traj = JointTrajectory()
  # traj.header.stamp = rospy.get_time()
  traj.header.frame_id = 'base_link'
  # Add the joint names (in any order, although  order corresponds to positions,
  # velocity, and effort indicie in the JointTrajectoryPoint. Whatever the
  # order, the simulator will control the correct joint based on name.)
  traj.joint_names = ['joint_b','joint_s','joint_t','joint_r','joint_u','joint_l']

  angle_range = 0.2
  time_range = 10
  # create a list of JointTrajectoryPoint objects
  points = []
  N = 100 # number of points
  for i in xrange(N):
    point = JointTrajectoryPoint()
    positions = []
    time_from_start = rospy.Duration.from_sec(time_range*i/float(N-1))
    point.time_from_start = time_from_start
    position = angle_range*math.sin(2*math.pi*i/(N-1))
    for j in xrange(6):
      positions.append(position)
    point.positions = positions
    points.append(point)
  traj.points= points

  rospy.loginfo(traj)
  trajPub.publish(traj)
  rospy.loginfo('published?')
