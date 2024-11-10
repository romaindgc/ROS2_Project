# Commandes
## Ouvrir turtle sim
```bash
ros2 run turtlesim turtlesim_node & ros2 run turtlesim turtle_teleop_key
```
## Faire spaw une deuxième tortue et run le node
```bash
ros2 service call /spawn turtlesim/srv/Spawn "{x: 2, y: 2, theta: 0.2, name: 'turtleFollower'}"
```
## Run the node
```bash
cd ~/ROS2_Project/commande/
colcon build
source install/setup.bash
ros2 run turtleFollower Follow
```