# README - RT1 - Assignment 2 - Final Assignment

**Francesco Ganci - 4143910** - Robotics Engineering - A.A. 2020/2021

> Take a look at the video demo of the project!
> - [video demo here]()
> 
> Doxygen Documentation: 
> - [Doxygen Documentation here]()
> - better: **branch *gh-pages***, download from there.

# How to set up the project

Here are the instructions for setting up the project. 

## Compatibility

The project is compatible with **ROS Noetic**. Not yet tested with *ROS kinetic*: probably, it doesn't work there.

## Dependencies

### Slam - GMapping

In order to run the project, you need the two packages you can find [here](https://github.com/CarmineD8/slam_gmapping.git). SLAM and GMapping are tools for managing the movement of a robot with noisy odometry: their purpose is to correct odometry in a way that the robot can get its position as precisely as possible. 

Copy the packages into the workspace you prefer. **use branch : _noetic_**

- [GitHub : SLAM-GMapping Packages](https://github.com/CarmineD8/slam_gmapping.git)

Also these packages are required. Please install them. 

```bash
sudo apt-get install ros-noetic-openslam-gmapping
sudo apt-get install ros-noetic-navigation
```

### MoveBase

MoveBase is a motion planner: given a goal, it can retrieve a path from the actual position to the desired one, recomputing the path depending on the informations gathered by sensors in conjunction with Slam-GMapping. 

Simply install it:

```bash
sudo apt-get install ros-noetic-move-base
```

### Robot Description

In the repository [here](https://github.com/CarmineD8/robot_description) you can find a package which contains the data for simulating a robot with disturbed odometry. This package is needed in order to run the simulation in Rviz and Gazebo. 

Copy the packages inside the workspace you prefer. **use branch : _noetic_**

- [GitHub : Robot Description](https://github.com/CarmineD8/robot_description)

## Setup

1. I suggest you to create a new empty workspace in which install the project. Create the new workspace, and then copy the dependencies inside it. 

2. Download the repository, then add in your *src* folder the package *final_assignment*

3. Go to the root of the workspace, and from there launch `catkin_make`

4. Restart the console. That's all! 


# How to run the project

Inside the package, in the folder *launch*, you can find several launch files. Because of the huge quantity of nodes you have to launch for running the project, i strongly suggest to launch everything from these files. 

First of all, launch the simulation environment. You can ignore the large amount of warnings on the console: if Gazebo and Rviz run well, you need nothing else. Otherwise, close everything and relaunch. 

```bash
roslaunch final_assignment start.launch
```

Done this, launch another terminal and type this:

```bash
roslaunch final_assignment final_assignment.launch
```

This will launch the command line interface and all the other components. See the documentation, section *services*, for further informations about these components. Remember that you can interact with them via `rosservice call` if you note something strange during the simulation. 

At this point, you can start typing commands. See the documentation, section *Command Line Interface*, for further infos about the commands. If you don't want to bother in reading documentation, simply type `help` and start playing with it. Have fun!

# Structure of the project

## Package File System

Here are the significant folders inside this package:

- **scripts** 
- **src** 
- **include/robot_game** 
- **docs** 
- **launch** 
- **srv** 
- **world** 

...

### Architecture of the project

... 
