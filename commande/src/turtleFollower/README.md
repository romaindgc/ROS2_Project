# Commands

## Open turtleSim and control the turtle with the keyboard
```bash
ros2 run turtlesim turtlesim_node & ros2 run turtlesim turtle_teleop_key
```
## Spwan of a second turtle

```bash
ros2 service call /spawn turtlesim/srv/Spawn "{x: 2, y: 2, theta: 0.2, name: 'turtleFollower'}"
```
## Run the node Follow

```bash
cd ~/ROS2_Project/commande/
colcon build
source install/setup.bash
ros2 run turtleFollower Follow
```

## Run the node Follow_etat

```bash
cd ~/ROS2_Project/commande/
colcon build
source install/setup.bash
ros2 run turtleFollower Followetat
```

## Run the node node_cmdTurtle1

```bash
cd ~/ROS2_Project/commande/
colcon build
source install/setup.bash
ros2 run turtleFollower node_cmdTurtle1
```

## Run the node node_follow_trajectoire
```bash
cd ~/ROS2_Project/commande/
colcon build
source install/setup.bash
ros2 run turtleFollower node_follow_trajectoire
```

## Run the node centered_cmd

```bash
cd ~/ROS2_Project/commande/
colcon build
source install/setup.bash
ros2 run turtleFollower centered_cmd
```
