# :sparkles: MA1400 Simulator :sparkles:

The goal of this package is to provide a simulated Motoman MA1400 Welding Robot in Gazebo and simulated low level DX100 controller and expose them with an interface that is described by the motoman_driver ROS driver.

## Introduction

This package provides a simulated implementation of a 6-axis [MOTOMAN MA1400](http://www.motoman.co.uk/en/products/robots/product-view/?tx_catalogrobot_pi1%5Buid%5D=2508&cHash=d3ce061255ce9c779404fd7022030526) industrial robot arm, including a [URDF](http://wiki.ros.org/urdf), visual and collision meshes, and launch files to get it up and running in [Gazebo](http://gazebosim.org/).

Also included is an interface for controlling the trajectory of the robot that mimcs the interface used by [mototman_driver ROS driver](http://wiki.ros.org/motoman_driver). The low level trajectory following algorithm done by the actual robot controller (such as the DX1000) is also simulated.

## URDF Creation

