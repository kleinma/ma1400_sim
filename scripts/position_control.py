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
import math

if __name__ == "__main__":
  rospy.init_node('position_control')
  muffler_pub = rospy.Publisher('/muffler_00/joint_revolute_controller/command',Float64,queue_size=1)

  muffler_pos = Float64()

  rate = rospy.Rate(10) # 10hz

  while not rospy.is_shutdown():
    t = rospy.get_time()
    muffler_pos.data = 2*math.pi*math.sin(t)

    muffler_pub.publish(muffler_pos)
    rate.sleep()
