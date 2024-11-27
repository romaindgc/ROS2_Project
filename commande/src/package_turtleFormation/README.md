# Commands

## Open turtleSim and control the turtle with the keyboard
```cpp
ros2 run turtlesim turtlesim_node &
ros2 run turtlesim turtle_teleop_key
```
## Spawn of a second turtle
```cpp
ros2 service call /spawn turtlesim/srv/Spawn "{x: 5, y: 5, theta: 0.2, name: 'turtleFollower'}"
```

## Run the node node_cmdTurtle1
```cpp
cd ~/ROS2_Project/commande/
colcon build
source install/setup.bash
ros2 run package_turtleFormation node_cmdTurtle1
```

## Run the node node_follow_trajectoire

```cpp
cd ~/ROS2_Project/commande/
colcon build
source install/setup.bash
ros2 run package_turtleFormation node_follow_trajectoire
```