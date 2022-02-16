# RoboCar 1/10 Core Nodes

It's a set of ROS nodes common for robot and simulation.

| Name                                                                     | Description                                             |
|:-------------------------------------------------------------------------|:--------------------------------------------------------|
| [**common**](rc110_common/README.md)                                     | Common launch files                                     |
| [**laserscans_to_pointcloud**](rc110_laserscans_to_pointcloud/README.md) | Conversion from two LaserScan to PointCloud2            |
| [**master_hold**](rc110_master_hold/README.md)                           | Node that does reconnection to roscore when it restarts |
| **msgs**                                                                 | Robot specific messages                                 |
| [**rviz**](rc110_rviz/README.md)                                         | RViz control panel                                      |
| [**teleop**](rc110_teleop/README.md)                                     | Joystick manipulation                                   |
| [**twist_to_ackermann**](rc110_twist_to_ackermann/README.md)             | Conversion from special Twist to Ackermann              |
