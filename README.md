UBR-1 Reloaded
==============
This is a fork of the ubr1_preview repo, updated to work with an actual robot
on newer versions of ROS.

The closest thing to documentation would be to read the
[posts about the UBR-1 on my blog](https://www.robotandchisel.com/tag/ubr1/)

## Launch Files

### ubr1_bringup/robot.launch.py
This launches the full robot with drivers and teleoperation.
Usually done as part of a system service installed via ansible.

### ubr1_navigation/build_map.launch.py
Used to build a map with ``slam_toolbox``.
Can not be run at the same time as ``localization.launch.py``.

### ubr1_navigation/localization.launch.py
Launches ``amcl`` node, configured for UBR-1.

### ubr1_navigation/navigation.launch.py
Launches the navigation stack.
Requires ``localization.launch.py`` to also be running.

### ubr1_moveit/move_group.launch.py
Launches MoveIt2 move_group, with the MoveIt Task Constructor.

### ubr1_demo/simple_grasping.launch.py
Launches simple grasping's ``basic_grasping_perception`` node,
properly configured for the UBR-1 gripper.

### ubr1_demo/pick_place.launch.py
Launches the pick and place demo using MTC.
