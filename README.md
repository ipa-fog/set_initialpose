# package to set the initial robot pose under ROS

Option 1 (`use_last_known_pose = false`):
  - Set initial 2D-Pose in launch-file.
  - 5s after start publish the init pose as topic `/initialpose` out of the launch file data.

  
  
Option 2 (`use_last_known_pose = true`):
 - Take last known 2D-Pose to set initial pose.
 - Subscribe the topic `/state_ekf` to get the pose simultaneos. The latest pose is saved into `cfg/pose.yaml`. 
 - 5s after start publish the last known pose as topic `/initialpose` out of the yaml-file data.




