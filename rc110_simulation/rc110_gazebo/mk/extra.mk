run-model:
	source $$(catkin locate rc110)/host_setup.bash
	roslaunch --wait rc110_gazebo model.launch $(ros_args)
