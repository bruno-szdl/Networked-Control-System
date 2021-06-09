# Networked Control Systems
Repository for the Networked Control Systems course of PPGEAS (UFSC)


![melodic-devel Status](https://img.shields.io/static/v1?label=melodic-devel&message=passing&color=brightgreen&style=plastic&logo=github)

## Requirements
### ROS Melodic
Follow the steps in http://wiki.ros.org/melodic/Installation/Ubuntu

The code was not tested on different versions of ROS



## Installation
Create a folder for the catkin workspace
```
mkdir -p ~/catkin_ws/src
```
Donwload the repo
```
cd ~/catkin_ws/src
git clone https://github.com/bruno-szdl/Networked-Control-Systems.git
```
Build the packages
```
cd ..
catkin_make
```
Add turtlebot model to your .bashrc
```
echo 'export TURTLEBOT3_MODEL=burger' >> ~/.bashrc
source ~/.bashrc
```
## How to run
Go to the catkin workspace folder
```
cd ~/catkin_ws
```
Source setup.bash
```
source devel/setup.bash
```
### Rendezvous
```
roslaunch networked_control_systems multi_turtlebot3_rendezvous.launch 
```
### Leader Follower
```
roslaunch networked_control_systems multi_turtlebot3_leader_follower.launch 
```
You can control the leader robot using the keyboard

wasd: to move around

w/x : increase/decrease linear velocity (Burger : ~ 0.22, Waffle and Waffle Pi : ~ 0.26)

a/d : increase/decrease angular velocity (Burger : ~ 2.84, Waffle and Waffle Pi : ~ 1.82)

space key, s : force stop




