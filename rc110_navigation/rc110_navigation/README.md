# Robot 2D Navigation

Node Based on [ROS Navigation Stack](http://wiki.ros.org/navigation)

## UI
![](docs/navigation.gif)

## Makefile
```
make deps         # check dependencies
make              # build
make run          # run the node
make show         # show rviz
make monitor      # show rviz on remote pc
```

## Known Issues

**ros-melodic-navigation** v1.16.7

* Localization is performed only during drive, but not when repositioned.
* Local scan obstacles are [not cleared](https://answers.ros.org/question/257286/obstacles-are-not-cleared-completely-in-costmap/).
* Movement precision is not good (>10cm distance to obstacle).
* Localization gets broken, if robot is stuck in obstacle.
