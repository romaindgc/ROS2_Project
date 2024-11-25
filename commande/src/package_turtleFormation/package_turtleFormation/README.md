# Commandes
## Ouvrir turtle sim
```cpp
ros2 run turtlesim turtlesim_node &
ros2 run turtlesim turtle_teleop_key
```
## Faire spaw une deuxième tortue et run le node
```cpp
ros2 service call /spawn turtlesim/srv/Spawn "{x: 5, y: 5, theta: 0.2, name: 'turtleFollower'}"
```

```cpp
cd ~/ROS2_Project/commande/
colcon build
source install/setup.bash
ros2 run package_turtleFormation node_cmdTurtle1
```
```cpp
cd ~/ROS2_Project/commande/
colcon build
source install/setup.bash
ros2 run package_turtleFormation node_follow_trajectoire
```