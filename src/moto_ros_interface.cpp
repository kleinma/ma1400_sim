/******************************************************************************
moto_ros_interface.cpp
The MotoRosNode Class provides an interface between the ROS's JointTrajectory
messages and a ROS Control JointPositionController, simulating Motoplus-ROS
Incremental Motion Interface.
http://wiki.ros.org/motoman_driver?action=AttachFile&do=get&target=MotoRos_EDS.pdf
*******************************************************************************
The MIT License (MIT)

  Copyright (c) 2017 Matthew A. Klein

  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files (the "Software"), to deal
  in the Software without restriction, including without limitation the rights
  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
  copies of the Software, and to permit persons to whom the Software is
  furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
******************************************************************************/
#include <ros/ros.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <std_msgs/Float64.h>
#include "ma1400_sim/moto_ros_interface.h"

void MotoRosNode::trajSubCB(const trajectory_msgs::JointTrajectory& msg) {
  ROS_INFO_STREAM(msg.header.stamp);
}

MotoRosNode::MotoRosNode(): private_nh_("~") {
  base2sPub_ = public_nh_.advertise<std_msgs::Float64>("ma1400/base2s_controller/command",1);
  trajSub_ = public_nh_.subscribe("ma1400/trajectory",10,&MotoRosNode::trajSubCB, this);
}

/* Destructor */
MotoRosNode::~MotoRosNode() {

}


int main(int argc, char **argv) {
  // Initialize ROS
  ros::init(argc, argv, "moto_ros_interface");
  // Create an MotoRosNode Object
  MotoRosNode motoRosNode;
  // And spin
  ros::spin();

  return 0;
}
