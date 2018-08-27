# package to set the initial robot pose under ROS

Subscribes the topic `/state_ekf` to get the pose simultaneos. The latest post is saved into `cfg/pose.yaml`.
At start we publish the init pose as topic `/initialpose` out of the file data (last known pose).



