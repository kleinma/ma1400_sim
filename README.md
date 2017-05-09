# :sparkles: MA1400 Simulator :sparkles:

The goal of this package is to provide a simulated Motoman MA1400 Welding Robot in Gazebo and simulated low level DX100 controller and expose them with an interface that is described by the motoman_driver ROS driver.

## Introduction

This package provides a simulated implementation of a 6-axis [MOTOMAN MA1400](http://www.motoman.co.uk/en/products/robots/product-view/?tx_catalogrobot_pi1%5Buid%5D=2508&cHash=d3ce061255ce9c779404fd7022030526) industrial robot arm, including a [URDF](http://wiki.ros.org/urdf), visual and collision meshes, and launch files to get it up and running in [Gazebo](http://gazebosim.org/).

Also included is an interface for controlling the trajectory of the robot that mimcs the interface used by [mototman_driver ROS driver](http://wiki.ros.org/motoman_driver). The low level trajectory following algorithm done by the actual robot controller \(such as the DX1000\) is also simulated.

## Basic URDF Creation

The [SolidWorks to URDF Exporter](http://wiki.ros.org/sw_urdf_exporter) made URDF creation much easier. After installing the plugin and opening the \*.stp filein SolidWorks \(2014 in my case\), a coordinate system must be defined at each joint and material properties of each link defined. By selecting `File->Export as URDF`, a wizard pops up which allows you to define each link in parent, child order, starting from the base_link.

At this point the plugin automatically generates the origins for each link and joint, as well as mass and inertia properties. It is up to the user to define the axis of rotation, as well as limits on rotation, velocity, and effort. The plugin also generates \*.stl files and links to them in the URDF. \(\*\*Note: I had the issue that the \*.stl files were saved with capitalized extension, which caused problems. After changing this, everything was fine.\)

I found that it was necessary to add a damping element to each joint. If this is left out, the robot is very springy in simulation.

### Using xacro files instead of urdf

At this point, the user must manually change some parts of the URDF. First, the user may want to use a \*.xacro file instead of a \*.urdf. These two files are very similar, except xacro files can use macros to generate repetitive blocks of text, making the file cleaner and less prone to mistakes. For more imformation, check out this [tutorial](http://wiki.ros.org/urdf/Tutorials/Using%20Xacro%20to%20Clean%20Up%20a%20URDF%20File).

### Additional gazebo specific elements

At this point, the simulated robot can be viewed inside rviz. However, a few more things must be defined before it is ready for simulation in Gazebo. This [tutorial](http://gazebosim.org/tutorials/?tut=ros_urdf) describes how to add specific Gazebo elements such as material (for color) and friction.

## Position control using ros_control
