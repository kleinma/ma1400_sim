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
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include "ma1400_sim/moto_ros_interface.h"
#include <vector>
#include <algorithm>
#include <string>

// When a JointTrajectory message is recieved, currently, iterate through the
// points at the desired speed. Do not implement smooth interpolation, yet.
void MotoRosNode::trajSubCB(const trajectory_msgs::JointTrajectory::ConstPtr& msg) {
  ROS_INFO_STREAM(*msg);

  // Create a vector that contains the standard order of the joint_names
  std::vector<std::string> jointNamesStdOrder;
  jointNamesStdOrder.push_back("joint_s");
  jointNamesStdOrder.push_back("joint_l");
  jointNamesStdOrder.push_back("joint_u");
  jointNamesStdOrder.push_back("joint_r");
  jointNamesStdOrder.push_back("joint_b");
  jointNamesStdOrder.push_back("joint_t");

  // iterate over the joint names
  std::vector<int> indicies;
  int j = 0;
  for(std::vector<std::string>::const_iterator it =
        jointNamesStdOrder.begin(); it != jointNamesStdOrder.end(); it++)
    {
      // look in msg->joint_names and see at which indicie that name exists
      // **currently no check for a missing name**
      indicies.push_back(std::find(msg->joint_names.begin(),
                                   msg->joint_names.end(),
                                   *it)
                         - msg->joint_names.begin());
      j++;
    }

  // DEBUG
  for(std::vector<int>::const_iterator it = indicies.begin(); it != indicies.end(); it++)
    {
      ROS_DEBUG_STREAM("joint order in message:");
      ROS_DEBUG_STREAM(*it);
    }

  // Record when the message is received
  ros::Duration timeSinceLast = ros::Duration(0);
  ros::Duration last_time_from_start = ros::Duration(0);
  // Iterate through each JointTrajectoryPoint
  int i = 0;
  for(std::vector<trajectory_msgs::JointTrajectoryPoint>::const_iterator it =
        msg->points.begin(); it != msg->points.end(); it++)
    {
      trajectory_msgs::JointTrajectoryPoint pnt;
      pnt = *it;
      timeSinceLast = pnt.time_from_start - last_time_from_start;
      last_time_from_start = pnt.time_from_start;
      timeSinceLast.sleep();
      ROS_INFO_STREAM(timeSinceLast);
      // Command joints to position in pnt->positions[indicies[n]];
      std_msgs::Float64 command;
      command.data = pnt.positions[indicies[0]];
      joint_sPub_.publish(command);
      command.data = pnt.positions[indicies[1]];
      joint_lPub_.publish(command);
      command.data = pnt.positions[indicies[2]];
      joint_uPub_.publish(command);
      command.data = pnt.positions[indicies[3]];
      joint_rPub_.publish(command);
      command.data = pnt.positions[indicies[4]];
      joint_bPub_.publish(command);
      command.data = pnt.positions[indicies[5]];
      joint_tPub_.publish(command);
      ROS_INFO_STREAM("i = " << i << ", time = " << ros::Time::now().toSec());
      i++;
    }
}

/*Constructor */
MotoRosNode::MotoRosNode(): private_nh_("~") {

  // Create publisher to send commands to ros_control controllers
  joint_sPub_ = public_nh_.advertise<std_msgs::Float64>("joint_s_controller/command",1);
  joint_lPub_ = public_nh_.advertise<std_msgs::Float64>("joint_l_controller/command",1);
  joint_uPub_ = public_nh_.advertise<std_msgs::Float64>("joint_u_controller/command",1);
  joint_rPub_ = public_nh_.advertise<std_msgs::Float64>("joint_r_controller/command",1);
  joint_bPub_ = public_nh_.advertise<std_msgs::Float64>("joint_b_controller/command",1);
  joint_tPub_ = public_nh_.advertise<std_msgs::Float64>("joint_t_controller/command",1);

  // Create a subscriber to receive the JointTrajectory message
  trajSub_ = public_nh_.subscribe("joint_path_command",10,&MotoRosNode::trajSubCB, this);
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
