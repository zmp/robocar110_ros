# ROS Master Hold

A node that holds master connection (ROS1 workaround).

When roscore is stopped / started, the hold node stops / starts the nodes that was passed as arguments. Without this trick, nodes cannot reconnect automatically now (ROS Melodic).

## Usage

```
<launch>
    <node pkg="rc110_master_hold" type="rc110_master_hold" name="master_hold" args="space separated list of nodes"/>
```

For each node specified above, `respawn="true"` should be set in node tag of launch file.

## Notes

Currently, it works only for linux. But windows support can be implemented.
