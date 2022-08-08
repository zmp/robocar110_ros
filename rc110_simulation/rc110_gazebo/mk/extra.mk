run-model:
	source $$(catkin locate rc110)/host_setup.bash
	roslaunch --wait rc110_gazebo model.launch $(ros_args)

run-navigation-zmp:
	source $$(catkin locate rc110)/auto_setup.bash
	mon launch rc110_navigation main.launch map_name:=$$(catkin locate rc110_gazebo)/maps/zmp.yaml $(ros_args)
