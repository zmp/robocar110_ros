**Annotations**  
For clarity of explanation, the procedure for using https://github.com/aws-robotics/aws-robomaker-small-house-world is written.    
Open a terminal and use git clone to download the data for the map you want to use  
```
cd /home/zmp/ros/src/
git clone https://github.com/aws-robotics/aws-robomaker-small-house-world.git
```  

After stopping the robot car, type the following command to run gazebo and check if the map is reflected.
```
cd aws-robomaker-small-house-world
export GAZEBO_MODEL_PATH=/home/zmp/aws-robomaker-small-house-world/models
gazebo worlds/small_house.world --verbose
```  
If the reflection was successful, you will see an image like this.  
[![](images/norobocar.png)]  

After confirmation, stop the operation and make the following changes to `/home/zmp/ros/src/robocar110_ros/rc110_simulation/rc110_gazebo/launch/main.launch`.  
```
    <arg name="root" default="$(find rc110_gazebo)"/>
    <arg name="world" default="worlds/maze.world"/>

    <env name="GAZEBO_RESOURCE_PATH" value="$(arg root)"/>
    <env name="GAZEBO_MODEL_PATH" value="$(arg root)/models"/>
```  
Open a new terminal and type the following command to execute it.
```
cd aws-robomaker-small-house-world
export GAZEBO_MODEL_PATH=/home/zmp/aws-robomaker-small-house-world/models
cd ~/ros/src/robocar110_ros/
make run-gazebo root:=${HOME}/ros/src/aws-robomaker-small-house-world world:=worlds/small_house.world
```  
If the operation is successful, it will look like the following image.  
[![](images/robocar_in.png)]  

To quickly check robocar model on new world, you can add a model and reflect it in gazebo by doing the following:  
1. Click Insert tab.  
2. Select "Add Path.  
3. Apply `home/zmp/ros/src/robocar110_ros/rc110_core/rc110_common/models`.  
4. Click "Robocar 1/10" row, then click on world to paste it.
