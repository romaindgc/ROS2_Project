# Commandes
## Ouvrir turtle sim
```cpp
ros2 run turtlesim turtlesim_node &
ros2 run turtlesim turtle_teleop_key
```
## Faire spaw une deuxième tortue et run le node
```cpp
ros2 service call /spawn turtlesim/srv/Spawn "{x: 2, y: 2, theta: 0.2, name: 'turtleFolower'}"
```

```cpp
cd ~/ros2_ws/
colcon build
source install/setup.bash
ros2 run package_turtleFormation node_cmdTurtle1
```
