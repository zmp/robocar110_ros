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

* Use `2D Pose Estimate` button to position robot on map.
* Use `2D Nav Goal` button to specify goal position.
* Use `AD` button to switch between joystick and autonomous driving.

## Another Map
By default the map name is `map`. If you want to use more maps, you can select them as follows:
```
make select-map map_name=my_custom_map
```

* The map should be saved first with `save-map-he` or `save-map-cg`.

## Known Issues

**ros-melodic-navigation** v1.16.7

* Localization is performed only during drive, but not when repositioned.
* Local scan obstacles are [not cleared](https://answers.ros.org/question/257286/obstacles-are-not-cleared-completely-in-costmap/).
* Movement precision is not good (>10cm distance to obstacle).
* Localization gets broken, if robot is stuck in obstacle.
