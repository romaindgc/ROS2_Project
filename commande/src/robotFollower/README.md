# Robot follower

On définit que :
* Le robot **leader** est le robot **bleu**
* Le robot **follower** est le robot **vert**


## Commande pour créer les bridges

* Bridge pour le topic cmd_vel
```bash
ros2 run ros_gz_bridge parameter_bridge /model/vehicle_green/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist
```

* Bridge pour le topic odometry (vehicle_blue)
```bash
ros2 run ros_gz_bridge parameter_bridge /model/vehicle_blue/odometry@nav_msgs/msg/Odometry@gz.msgs.Odometry
```

* Bridge pour le topic odometry (vehicle_green)
```bash
ros2 run ros_gz_bridge parameter_bridge /model/vehicle_green/odometry@nav_msgs/msg/Odometry@gz.msgs.Odometry
```