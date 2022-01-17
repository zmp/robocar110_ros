## Gazebo World Switching

Below you can find an explanation for [aws-robomaker-small-house-world](https://github.com/aws-robotics/aws-robomaker-small-house-world). Other worlds can be used in a similar way.

Open a terminal and use git clone to download the data for the map you want to use  
```
cd /home/zmp/ros/src/
git clone https://github.com/aws-robotics/aws-robomaker-small-house-world.git
```  

If you run gazebo on robot, don't forget to stop the driver first (`make stop`).

Type the following command to run gazebo and check if the map is reflected.
```
cd aws-robomaker-small-house-world
export GAZEBO_MODEL_PATH=/home/zmp/aws-robomaker-small-house-world/models
gazebo worlds/small_house.world --verbose
```  
If the reflection was successful, you will see an image like this.  
![](images/norobocar.png)

To use the world with RoboCar 1/10, execute it as following:
```
cd ~/ros/src/robocar110_ros/
make run-gazebo root:=${HOME}/ros/src/aws-robomaker-small-house-world world:=worlds/small_house.world
```  
If the operation is successful, it will look like the following image.  
![](images/robocar_in.png)  

To quickly check robocar model on a new world, you can add a model and reflect it in gazebo by doing the following:  
1. Click Insert tab.  
2. Select "Add Path.  
3. Apply `home/zmp/ros/src/robocar110_ros/rc110_core/rc110_common/models`.  
4. Click "Robocar 1/10" row, then click on world to paste it.
