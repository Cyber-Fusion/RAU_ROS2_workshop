# RAU_ROS2_workshop
***This project represents communication between nodes showen in this diagram.***

<img width="972" src="https://github.com/Cyber-Fusion/RAU_ROS2_workshop/blob/main/rau-bot-diagram.png">

***Before building you must setup your root environment.(on Ubuntu/bash)***
```
source /opt/ros/humble/setup.bash
```
***Build***
```
colcon build
```

***Before running you must setup your local environment.(on Ubuntu/bash)***
```
source install/local_setup.bash
```
***Run head node.***
```
ros2 run head_package head_node
```
***Run leg node.***
```
ros2 run leg_package leg_node
```
***Run vision node.***
```
ros2 run vision_package vision_node
```
