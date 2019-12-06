# planning
Repository for planning code

## Overview over how to launch everything
### Start the HSR:
Plug in the power supply for the HSR.
Plug the PS2 controller into the USB port.
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

Use the command 'fix' to source most of the workspaces. It is currently in the bashrc, so this is done already.

In the terminals to start the nodes.
**Map and Lokalisation:**
roslaunch hsr_navigation hsr_map_and_snap_map.launch

**Environment with table and shelf:**
roslaunch iai_hsr_robocup hsr_robocup_with_state_publisher.launch

**Perception server:**
cd ~/suturo/perception_ws
source devel/setup.bash
rosrun hsr_perception perception_server

**Beliefstate:**
roslaunch object_state object_state.launch

**Giskard:**
roslaunch move giskardpy_hsr.launch

**Move server:**
roslaunch move move_server.launch

