# Robot follower

On définit que :
* Le robot **leader** est le robot **bleu**
* Le robot **follower** est le robot **vert**


## Run the node for the leade 

```bash
cd ~/ROS2_Project/commande/
colcon build
source install/setup.bash
ros2 run robotFollower robotLeader
```

## Run the node for the follower
```bash
cd ~/ROS2_Project/commande/
colcon build
source install/setup.bash
ros2 run robotFollower robotFollower
```