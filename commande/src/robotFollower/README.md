# Robot follower

We have chosen that :
* The **leader** is the **blue** robot
* The **follower** is the **green** robot

## Make two robots follow each other

### Run the node for the leader 

In one terminal : 

```bash
cd ~/ROS2_Project/commande/
colcon build
source install/setup.bash
ros2 run robotFollower robotLeader
```

### Run the node for the follower

In another one terminal : 

```bash
cd ~/ROS2_Project/commande/
colcon build
source install/setup.bash
ros2 run robotFollower robotFollower
```

## Make two robots follow a middle point between them contained in a trajectory

```bash
cd ~/ROS2_Project/commande/
colcon build
source install/setup.bash
ros2 run robotFollower centered_cmd
```
