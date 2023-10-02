# cse

## Description
This repo contains the code for collaborative sensor fusion.

Target robot: when global position (such as GPS) is unavailable, localization relies on the global position provided by Detector robot.

Detector robot: global localization is always working and relative position between two robots is available.

Therefore global position of Target robot can be computed by the concatenation of Detector robot's global position and the relative position.
This global measurement makes the localization of Target robot more safe and resilent to the loss of global position due to the environment interference or the failures of onboard sensors.

## Installation
Tested in ROS Melodic

## How to use?

```
roslaunch ov_cse_filter estimator.launch
```

Please specify these ros topics in estimator.launch.

topic_imu: imu topic of Target robot.

topic_pos: global position of Target robot provided by Detector robot.

topic_gps_pos: global position of Target robot provided by onboard sensor, such as GPS.

topic_signal: trigger signal which indicates the the loss of global position of Target robot.

topic_pub_pose: output pose of Target robot.

## Acknowledgment
Thanks for [GeomInertiaEstimator](https://github.com/arplaboratory/GeomInertiaEstimator).
