# planning
Repository for planning code

![](./planning.png)
Might not be working 100% after update to noetic June 2022
##Setting up the repository
###If you don't have ROS:
1. sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu focal main" > /etc/apt/sources.list.d/ros-latest.list'
2. sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
3. sudo apt-get update
4. sudo apt-get install ros-noetic-desktop-full
5. sudo rosdep init
6. rosdep update
7. sudo apt-get install python-catkin-tools
8. echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
9. source ~/.bashrc
10. sudo apt install python-rosinstall python-rosinstall-generator python-wstool build-essential
###If you have ROS already installed:
1. rosdep update
2. sudo apt-get install ros-noetic-roslisp-repl
3. sudo apt-get install python-rosinstall python-wstool
4. mkdir -p planning_ws/src
5. cd ~/planning_ws
6. catkin build
7. source devel/setup.bash
8. cd src

Follow the planning section on this guide for further installation instructions and how to start the cleanup and wipe plan.
https://github.com/suturo21-22/suturo-installation


## Overview over how to launch everything
### Start the HSR:
Plug in the power supply for the HSR.
Plug the PS2 controller into the USB port.(optional)
Check the switches on the right. Turn them both on.
Press the huge power button for a couple seconds.

From remote: connect to the Alienware via ssh:
ssh suturo@192.168.237
suturo19

Open a byobu session:
byobu

You can now start the launchfiles. Here's a cheat sheet for byobu:
F2 open a new terminal
CTRL-D close current terminal
F6 disconnect from byobu without killing the session
F3/F4 go to terminal left/right
F8 rename the current terminal
