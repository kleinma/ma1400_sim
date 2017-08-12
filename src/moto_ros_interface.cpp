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
#include <control_msgs/FollowJointTrajectoryFeedback.h>
#include <sensor_msgs/JointState.h>
#include <industrial_msgs/RobotStatus.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include "ma1400_sim/moto_ros_interface.h"
#include <vector>
#include <cmath>
#include <algorithm>
#include <string>

// When a JointTrajectory message is recieved, currently, iterate through the
// points at the desired speed. Do not implement smooth interpolation, yet.
void MotoRosNode::trajSubCB(const trajectory_msgs::JointTrajectory::ConstPtr& msg) {
  // iterate over the joint names
  std::vector<int> indicies;
  for(std::vector<std::string>::const_iterator it =
        jointNamesStdOrder_.begin(); it != jointNamesStdOrder_.end(); it++)
    {
      // look in msg->joint_names and see at which indicie that name exists
      // **currently no check for a missing name**
      indicies.push_back(std::find(msg->joint_names.begin(),
                                   msg->joint_names.end(),
                                   *it)
                         - msg->joint_names.begin());
    }

  // Set initial tPre, xPre, vPre by subscribing to joint_states
  std::vector<double> xPre = currJointPos_;
  std::vector<double> vPre = currJointVel_;
  double tPre = 0;
  std::vector<double> xTarg, vTarg, a1, a2;
  a1.resize(6);
  a2.resize(6);
  xTarg.resize(6);
  vTarg.resize(6);
  double tTarg;
  // x and v are will be a 2d vector that stores each interpolated x and v
  // throughout the move from xPre to xTarg. The inner vector will span the
  // joints, which the outer vector will span time.
  std::vector<std::vector<double> > x, v;
  std::vector<double> t;
  x.push_back(xPre);
  v.push_back(vPre);
  t.push_back(tPre);

  // Iterate through each JointTrajectoryPoint and add all the interpolated
  // time, position, and velocity values to t, x, and v.
  for(std::vector<trajectory_msgs::JointTrajectoryPoint>::const_iterator it =
        msg->points.begin(); it != msg->points.end(); it++)
    {
      trajectory_msgs::JointTrajectoryPoint pnt;
      pnt = *it;
      tTarg = pnt.time_from_start.toSec();
      // Make sure joints are loaded in the correct order.
      for (int i = 0; i < 6; ++i) {
        xTarg.at(i) = pnt.positions[indicies[i]];
        vTarg.at(i) = pnt.velocities[indicies[i]];
      }

      // Find a1 and a2 for each joint
      for(int i=0; i<6; i++) {
        a1.at(i) =   6 * (xTarg.at(i) - xPre.at(i)) / pow(tTarg - tPre, 2)
                   - 2 * (vTarg.at(i) + 2 * vPre.at(i)) / (tTarg - tPre);

        a2.at(i) = -12 * (xTarg.at(i) - xPre.at(i)) / pow(tTarg - tPre, 3)
                   + 6 * (vTarg.at(i) + vPre.at(i)) / pow(tTarg - tPre, 2);
      }

      // Add epsilon to dt_ to avoid rounding errors. The code was running into
      // issues with timesteps of ~1e-300 seconds.
      double epsilon = 0.0000001;
      while(t.at(t.size()-1) + dt_ + epsilon < tTarg) {
        // Increment the time
        t.push_back(t.at(t.size()-1) + dt_);
        // temporary vectors to store x and v at a single time increment.
        std::vector<double> xInterp (6);
        std::vector<double> vInterp (6);

        double delT = t.at(t.size()-1)-tPre;
        for(int i=0; i<6; i++) {
          xInterp.at(i) = xPre.at(i)
                        + vPre.at(i)*delT
                        + a1.at(i)*pow(delT,2)/2.0
                        + a2.at(i)*pow(delT,3)/6.0;

          vInterp.at(i) = vPre.at(i)
                        + a1.at(i)*delT
                        + a2.at(i)*pow(delT,2)/2.0;
        }
        x.push_back(xInterp);
        v.push_back(vInterp);
      }

      // Finally add the target t, x, and v to the list
      t.push_back(tTarg);
      x.push_back(xTarg);
      v.push_back(vTarg);

      // And set the subsequent Pre as this iterations Targ.
      tPre = tTarg;
      xPre = xTarg;
      vPre = vTarg;
    }

  // Set RobotStatus to moving
  robotStatus_.in_motion.val = 1;
  robotStatusPub_.publish(robotStatus_);

  // Iterate trhough all the iterpolated positions and send that command to the
  // joint controllers.
  for (int i = 0; i < t.size(); ++i ) {
    if (i == 0) {
      ros::Duration(t[0]).sleep();
    }
    else {
      ros::Duration(t[i]-t[i-1]).sleep();
    }
    // Command joints to next position
    std_msgs::Float64 command;
    command.data = x.at(i).at(0);
    joint_sPub_.publish(command);
    command.data = x.at(i).at(1);
    joint_lPub_.publish(command);
    command.data = x.at(i).at(2);
    joint_uPub_.publish(command);
    command.data = x.at(i).at(3);
    joint_rPub_.publish(command);
    command.data = x.at(i).at(4);
    joint_bPub_.publish(command);
    command.data = x.at(i).at(5);
    joint_tPub_.publish(command);
  }

  // Set RobotStatus to not moving
  robotStatus_.in_motion.val = 0;
  robotStatusPub_.publish(robotStatus_);

}


// Store the current position and velcoity when a JointState message is
// recieved.
void MotoRosNode::jointSubCB(const sensor_msgs::JointState::ConstPtr& msg) {
  firstJointStateReceived_ = true;
  // iterate over the joint names
  std::vector<int> indicies;
  int j = 0;
  for(std::vector<std::string>::const_iterator it =
        jointNamesStdOrder_.begin(); it != jointNamesStdOrder_.end(); it++)
    {
      // look in msg->name and see at which indicie that name exists
      // **currently no check for a missing name**
      int ind = std::find(msg->name.begin(),msg->name.end(),*it)- msg->name.begin();
      currJointPos_.at(j) = msg->position.at(ind);
      currJointVel_.at(j) = msg->position.at(ind);
      j++;
    }
  trajFeedback_.actual.positions = currJointPos_;
  trajFeedback_.actual.velocities = currJointVel_;
  trajFeedbackPub_.publish(trajFeedback_);
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
  trajFeedbackPub_ = public_nh_.advertise<control_msgs::FollowJointTrajectoryFeedback>("feedback_states",1);
  robotStatusPub_ = public_nh_.advertise<industrial_msgs::RobotStatus>("robot_status",1);

  // Create a subscriber to receive the JointTrajectory message
  trajSub_ = public_nh_.subscribe("joint_path_command",10,&MotoRosNode::trajSubCB, this);

  // Create a subscriber to receive the JointState message
  jointSub_ = public_nh_.subscribe("joint_states",1,&MotoRosNode::jointSubCB, this);

  // Initialize the time increment for interpoloation (default = 0.01 sec)
  if(!private_nh_.getParam("dt", dt_))
    dt_ = 0.01;

  // Set this to false. It is set to true in jointSubCB
  firstJointStateReceived_ = false;

  // Create a vector with the joint names in order. ROS messages containing
  // info about joints can list them in any order. By finding which order they
  // are listed, we can quickly publish to the correct joint.
  jointNamesStdOrder_.push_back("joint_s");
  jointNamesStdOrder_.push_back("joint_l");
  jointNamesStdOrder_.push_back("joint_u");
  jointNamesStdOrder_.push_back("joint_r");
  jointNamesStdOrder_.push_back("joint_b");
  jointNamesStdOrder_.push_back("joint_t");

  trajFeedback_.joint_names = jointNamesStdOrder_;

  // Initilize with six entries (one for each joint
  currJointPos_.resize(6);
  currJointVel_.resize(6);
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
  ros::MultiThreadedSpinner spinner(4);
  spinner.spin();

  return 0;
}
