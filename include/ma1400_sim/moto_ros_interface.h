/******************************************************************************
moto_ros_interface.h
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
#include <sensor_msgs/JointState.h>
#include <control_msgs/FollowJointTrajectoryFeedback.h>
#include <industrial_msgs/RobotStatus.h>

#ifndef __MOTO_ROS_INTERFACE_H_INCLUDED__
#define __MOTO_ROS_INTERFACE_H_INCLUDED__

class MotoRosNode {

 private:
  ros::NodeHandle public_nh_;
  ros::NodeHandle private_nh_;
  ros::Publisher joint_sPub_;
  ros::Publisher joint_lPub_;
  ros::Publisher joint_uPub_;
  ros::Publisher joint_rPub_;
  ros::Publisher joint_bPub_;
  ros::Publisher joint_tPub_;
  ros::Publisher trajFeedbackPub_;
  ros::Publisher robotStatusPub_;
  ros::Subscriber trajSub_;
  ros::Subscriber jointSub_;

  void trajSubCB(const trajectory_msgs::JointTrajectory::ConstPtr& msg);
  void jointSubCB(const sensor_msgs::JointState::ConstPtr& msg);

  // Create a vector that contains the standard order of the joint_names
  std::vector<std::string> jointNamesStdOrder_;

  // Time increment for interpolation.
  double dt_;
  // Current position and velocity (used as initial point in trajectory)
  std::vector<double> currJointPos_;
  std::vector<double> currJointVel_;
  // Don't receive trajectories until you've found out where the arm is
  bool firstJointStateReceived_;

  // Feedback message containing current position, desired position, and error
  control_msgs::FollowJointTrajectoryFeedback trajFeedback_;
  industrial_msgs::RobotStatus robotStatus_;

 public:
  MotoRosNode();
  ~MotoRosNode();
};

#endif
