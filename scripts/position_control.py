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
  base2s_pub = rospy.Publisher('/ma1400/base2s_controller/command', Float64,
                               queue_size=1)
  s2l_pub = rospy.Publisher('/ma1400/s2l_controller/command', Float64,
                               queue_size=1)
  l2u_pub = rospy.Publisher('/ma1400/l2u_controller/command', Float64,
                               queue_size=1)
  u2r_pub = rospy.Publisher('/ma1400/u2r_controller/command', Float64,
                               queue_size=1)
  r2b_pub = rospy.Publisher('/ma1400/r2b_controller/command', Float64,
                               queue_size=1)
  b2t_pub = rospy.Publisher('/ma1400/b2t_controller/command', Float64,
                               queue_size=1)
  base2s_pos = Float64()
  s2l_pos = Float64()
  l2u_pos = Float64()
  u2r_pos = Float64()
  r2b_pos = Float64()
  b2t_pos = Float64()

  rate = rospy.Rate(10) # 10hz

  while not rospy.is_shutdown():
    t = rospy.get_time()
    base2s_pos.data = 0.1*math.sin(t)
    s2l_pos.data = 0.1*math.sin(t)
    l2u_pos.data = 0.1*math.sin(t)
    u2r_pos.data = 0.1*math.sin(t)
    r2b_pos.data = 0.1*math.sin(t)
    b2t_pos.data = 0.1*math.sin(t)

    base2s_pub.publish(base2s_pos)
    s2l_pub.publish(s2l_pos)
    l2u_pub.publish(l2u_pos)
    u2r_pub.publish(u2r_pos)
    r2b_pub.publish(r2b_pos)
    b2t_pub.publish(b2t_pos)
    rate.sleep()
